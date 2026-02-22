#!/usr/bin/python3
import sys
import threading
from threading import Event
import os

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from robot_common.config_manager import ConfigManager

if os.name == "nt":
    import msvcrt
else:
    import termios
    import tty


# Hàm đọc phím 1 ký tự
def get_key():
    if os.name == "nt":
        key = msvcrt.getch()
        try:
            return key.decode("utf-8", errors="ignore")
        except Exception:
            return ""

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class MQTTBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_ros2')

        config = ConfigManager("mqtt_bridge", logger=self.get_logger()).load()
        broker_cfg = config.get("broker", {})
        topics_cfg = config.get("topics", {})
        keyboard_cfg = config.get("keyboard", {})
        plan_keys = config.get("plan_keys", {})

        self.broker_address = broker_cfg.get("address", "127.0.0.1")
        self.broker_port = broker_cfg.get("port", 1883)
        self.topic_vr = topics_cfg.get("vr_control", "VR_control")
        self.topic_pick = topics_cfg.get("pick_robot", "pick_robot")
        self.topic_plan = topics_cfg.get("plan_select", "plan_select")
        self.keyboard_map = {
            keyboard_cfg.get("forward", "w"): "Forward",
            keyboard_cfg.get("backward", "s"): "Backward",
            keyboard_cfg.get("left", "a"): "Left",
            keyboard_cfg.get("right", "d"): "Right",
            keyboard_cfg.get("stop", " "): "Stop",
            keyboard_cfg.get("rotate_left", "j"): "RotateLeft",
            keyboard_cfg.get("rotate_right", "p"): "RotateRight",
        }
        self.key_toggle_auto = keyboard_cfg.get("toggle_auto", "k")
        self.key_quit = keyboard_cfg.get("quit", "q")
        self.plan_key_map = plan_keys

        # ROS 2 publisher
        self.ros_pub = self.create_publisher(String, self.topic_vr, 10)
        self.ros_pick_pub = self.create_publisher(String, self.topic_pick, 10)
        self.ros_plan_pub = self.create_publisher(String, self.topic_plan, 10)

        if mqtt is None:
            raise RuntimeError("paho-mqtt is not installed")

        # MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self._mqtt_connected = False

        self.client.connect(self.broker_address, self.broker_port, 60)

        # MQTT chạy ở thread riêng
        self._stop_event = Event()
        self._mqtt_thread = threading.Thread(target=self.client.loop_forever, daemon=True)
        self._mqtt_thread.start()

        # Keyboard đọc ở worker thread để không block executor
        self._keyboard_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self._keyboard_thread.start()

        self.get_logger().info("MQTT ↔ ROS 2 bridge started")

    # MQTT callbacks
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._mqtt_connected = True
            self.get_logger().info("Connected to MQTT broker")
            client.subscribe(self.topic_vr)
            client.subscribe(self.topic_pick)
            client.subscribe(self.topic_plan)
        else:
            self.get_logger().error(f"MQTT connect failed: {rc}")

    def on_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        self.get_logger().warning(f"Disconnected from MQTT broker: rc={rc}")

    def on_message(self, client, userdata, msg):
        data = msg.payload.decode()
        self.get_logger().info(f"MQTT → ROS2: {data}")

        ros_msg = String()
        ros_msg.data = data
        if msg.topic == self.topic_vr:
            self.ros_pub.publish(ros_msg)
        elif msg.topic == self.topic_pick:
            self.ros_pick_pub.publish(ros_msg)
        elif msg.topic == self.topic_plan:
            self.ros_plan_pub.publish(ros_msg)

    # Keyboard → MQTT
    def keyboard_loop(self):
        self.get_logger().info("Điều khiển WASD | q để thoát")
        autoMode = False
        while rclpy.ok() and not self._stop_event.is_set():
            key = get_key().lower()
            cmd = ""

            if key in self.keyboard_map:
                cmd = self.keyboard_map[key]
            elif key == self.key_toggle_auto:
                autoMode = not autoMode
                mode_msg = "1" if autoMode else "0"
                self.client.publish(self.topic_pick, mode_msg)
                self.get_logger().info(f"ROS2 → MQTT: {mode_msg}")
            elif key in self.plan_key_map:
                plan_name = self.plan_key_map[key]
                if self._mqtt_connected:
                    self.client.publish(self.topic_plan, plan_name)
                    self.get_logger().info(f"ROS2 → MQTT: {plan_name}")
                else:
                    # Fallback local publish so operator key still works when MQTT is down.
                    ros_msg = String()
                    ros_msg.data = plan_name
                    self.ros_plan_pub.publish(ros_msg)
                    self.get_logger().warning(
                        f"MQTT unavailable, published plan locally: {plan_name}"
                    )
            elif key == self.key_quit:
                self.get_logger().info("Thoát chương trình")
                self._stop_event.set()
                rclpy.shutdown()
                break

            if cmd:
                self.client.publish(self.topic_vr, cmd)
                self.get_logger().info(f"ROS2 → MQTT: {cmd}")

        self.client.disconnect()

    def destroy_node(self):
        self._stop_event.set()
        try:
            self.client.disconnect()
        except Exception:
            pass
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
