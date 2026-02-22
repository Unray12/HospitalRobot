#!/usr/bin/python3
import json
import sys
import termios
import threading
import tty
from importlib import resources

import paho.mqtt.client as mqtt
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# Hàm đọc phím 1 ký tự
def get_key():
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

        config = self._load_config()
        broker_cfg = config.get("broker", {})
        topics_cfg = config.get("topics", {})
        keyboard_cfg = config.get("keyboard", {})

        self.broker_address = broker_cfg.get("address", "127.0.0.1")
        self.broker_port = broker_cfg.get("port", 1883)
        self.topic_vr = topics_cfg.get("vr_control", "VR_control")
        self.topic_pick = topics_cfg.get("pick_robot", "pick_robot")
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

        # ROS 2 publisher
        self.ros_pub = self.create_publisher(String, self.topic_vr, 10)
        self.ros_pick_pub = self.create_publisher(String, self.topic_pick, 10)

        # MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(self.broker_address, self.broker_port, 60)

        # MQTT chạy ở thread riêng
        mqtt_thread = threading.Thread(target=self.client.loop_forever, daemon=True)
        mqtt_thread.start()

        self.get_logger().info("MQTT ↔ ROS 2 bridge started")
        self.keyboard_loop()

    # MQTT callbacks
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker")
            client.subscribe(self.topic_vr)
            client.subscribe(self.topic_pick)
        else:
            self.get_logger().error(f"MQTT connect failed: {rc}")

    def on_message(self, client, userdata, msg):
        data = msg.payload.decode()
        self.get_logger().info(f"MQTT → ROS2: {data}")

        ros_msg = String()
        ros_msg.data = data
        if msg.topic == self.topic_vr:
            self.ros_pub.publish(ros_msg)
        elif msg.topic == self.topic_pick:
            self.ros_pick_pub.publish(ros_msg)

    # Keyboard → MQTT
    def keyboard_loop(self):
        self.get_logger().info("Điều khiển WASD | q để thoát")
        autoMode = False
        while rclpy.ok():
            key = get_key().lower()
            cmd = ""

            if key in self.keyboard_map:
                cmd = self.keyboard_map[key]
            elif key == self.key_toggle_auto:
                autoMode = not autoMode
                mode_msg = "1" if autoMode else "0"
                self.client.publish(self.topic_pick, mode_msg)
                self.get_logger().info(f"ROS2 → MQTT: {mode_msg}")
            elif key == self.key_quit:
                self.get_logger().info("Thoát chương trình")
                break

            if cmd:
                self.client.publish(self.topic_vr, cmd)
                self.get_logger().info(f"ROS2 → MQTT: {cmd}")

        self.client.disconnect()

    def _load_config(self):
        try:
            path = resources.files("mqtt_bridge").joinpath("config.json")
            with path.open("r", encoding="utf-8") as f:
                return json.load(f)
        except Exception as exc:
            self.get_logger().warn(f"config.json not loaded, using defaults: {exc}")
            return {}


def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
