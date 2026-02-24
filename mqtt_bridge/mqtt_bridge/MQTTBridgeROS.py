#!/usr/bin/python3
import sys
import threading
from threading import Event
import os
import re

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter

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
        self.log = LogAdapter(self.get_logger(), "mqtt_bridge")

        config = ConfigManager("mqtt_bridge", logger=self.log).load()
        broker_cfg = config.get("broker", {})
        topics_cfg = config.get("topics", {})
        keyboard_cfg = config.get("keyboard", {})
        plan_keys = config.get("plan_keys", {})
        room_plans = config.get("room_plans", {})

        self.broker_address = broker_cfg.get("address", "127.0.0.1")
        self.broker_port = broker_cfg.get("port", 1883)
        self.topic_vr = topics_cfg.get("vr_control", "VR_control")
        self.topic_pick = topics_cfg.get("pick_robot", "pick_robot")
        self.topic_plan = topics_cfg.get("plan_select", "plan_select")
        self.topic_debug_toggle = topics_cfg.get("debug_toggle", "/debug_logs_toggle")
        self.keyboard_map = {
            str(keyboard_cfg.get("forward", "w")).lower(): "Forward",
            str(keyboard_cfg.get("backward", "s")).lower(): "Backward",
            str(keyboard_cfg.get("left", "a")).lower(): "Left",
            str(keyboard_cfg.get("right", "d")).lower(): "Right",
            str(keyboard_cfg.get("stop", " ")).lower(): "Stop",
            str(keyboard_cfg.get("rotate_left", "j")).lower(): "RotateLeft",
            str(keyboard_cfg.get("rotate_right", "p")).lower(): "RotateRight",
        }
        self.key_toggle_auto = str(keyboard_cfg.get("toggle_auto", "k")).lower()
        self.key_toggle_debug_logs = str(keyboard_cfg.get("toggle_debug_logs", "e")).lower()
        self.key_quit = str(keyboard_cfg.get("quit", "q")).lower()
        self.plan_key_map = plan_keys
        self.room_plan_map = {str(k).strip(): str(v).strip() for k, v in room_plans.items()}
        if not self.room_plan_map:
            # Backward compatible: if no room_plans, reuse numeric plan hotkeys.
            self.room_plan_map = {
                str(k).strip(): str(v).strip()
                for k, v in self.plan_key_map.items()
                if str(k).strip().isdigit()
            }
        self._known_plans = set(self.room_plan_map.values()) | set(self.plan_key_map.values())
        self._debug_logs_enabled = False

        # ROS 2 publisher
        self.ros_pub = self.create_publisher(String, self.topic_vr, 10)
        self.ros_pick_pub = self.create_publisher(String, self.topic_pick, 10)
        self.ros_plan_pub = self.create_publisher(String, self.topic_plan, 10)
        self.ros_debug_pub = self.create_publisher(Bool, self.topic_debug_toggle, 10)

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

        self.log.info("MQTT <-> ROS2 bridge started", event="BOOT")

    # MQTT callbacks
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._mqtt_connected = True
            self.log.info("Connected to MQTT broker", event="MQTT")
            client.subscribe(self.topic_vr)
            client.subscribe(self.topic_pick)
            client.subscribe(self.topic_plan)
        else:
            self.log.error(f"MQTT connect failed: {rc}", event="MQTT")

    def on_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        self.log.warning(f"Disconnected from MQTT broker: rc={rc}", event="MQTT")

    def on_message(self, client, userdata, msg):
        data = msg.payload.decode()
        self.log.info(f"MQTT -> ROS2: {data}", event="BRIDGE_IN")

        ros_msg = String()
        if msg.topic == self.topic_vr:
            ros_msg.data = data
            self.ros_pub.publish(ros_msg)
        elif msg.topic == self.topic_pick:
            ros_msg.data = data
            self.ros_pick_pub.publish(ros_msg)
        elif msg.topic == self.topic_plan:
            resolved = self._resolve_plan_command(data)
            if not resolved:
                self.log.warning(f"Ignored invalid plan command: {data}", event="PLAN_MQTT")
                return
            ros_msg.data = resolved
            self.ros_plan_pub.publish(ros_msg)
            if resolved != data:
                self.log.info(f"Normalized plan command: {data} -> {resolved}", event="PLAN_MQTT")

    # Keyboard → MQTT
    def keyboard_loop(self):
        self.log.info("Dieu khien WASD | q de thoat", event="KEYBOARD")
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
                self.log.info(f"ROS2 -> MQTT: {mode_msg}", event="BRIDGE_OUT")
            elif key == self.key_toggle_debug_logs:
                self._debug_logs_enabled = not self._debug_logs_enabled
                self.ros_debug_pub.publish(Bool(data=self._debug_logs_enabled))
                state = "ON" if self._debug_logs_enabled else "OFF"
                self.log.info(
                    f"Debug logs toggled: {state} (key={self.key_toggle_debug_logs})",
                    event="DEBUG_TOGGLE",
                )
            elif key in self.plan_key_map:
                plan_name = self.plan_key_map[key]
                if self._mqtt_connected:
                    mqtt_plan_cmd = f"room:{key}" if key.isdigit() else plan_name
                    self.client.publish(self.topic_plan, mqtt_plan_cmd)
                    self.log.info(f"ROS2 -> MQTT: {mqtt_plan_cmd}", event="BRIDGE_OUT")
                else:
                    # Fallback local publish so operator key still works when MQTT is down.
                    ros_msg = String()
                    ros_msg.data = plan_name
                    self.ros_plan_pub.publish(ros_msg)
                    self.log.warning(
                        f"MQTT unavailable, published plan locally: {plan_name}",
                        event="BRIDGE_FALLBACK",
                    )
            elif key == self.key_quit:
                self.log.info("Thoat chuong trinh", event="KEYBOARD")
                self._stop_event.set()
                rclpy.shutdown()
                break

            if cmd:
                self.client.publish(self.topic_vr, cmd)
                self.log.info(f"ROS2 -> MQTT: {cmd}", event="BRIDGE_OUT")

        self.client.disconnect()

    def destroy_node(self):
        self._stop_event.set()
        try:
            self.client.disconnect()
        except Exception:
            pass
        super().destroy_node()

    def _resolve_plan_command(self, payload: str):
        text = (payload or "").strip()
        if not text:
            return None

        lower = text.lower()
        if lower in {"0", "clear", "room:0", "room/0"}:
            return "clear"

        if text in self.plan_key_map:
            return self.plan_key_map[text]
        if text in self.room_plan_map:
            return self.room_plan_map[text]

        m = re.match(r"^(?:room|phong|plan)\s*[:/_-]?\s*(\d+)$", lower)
        if m:
            room_id = m.group(1)
            if room_id == "0":
                return "clear"
            if room_id in self.room_plan_map:
                return self.room_plan_map[room_id]
            self.log.warning(f"Room id has no mapped plan: {room_id}", event="PLAN_MQTT")
            return None

        if lower.startswith("plan:") or lower.startswith("plan/"):
            plan_name = text.split(":", 1)[1].strip() if ":" in text else text.split("/", 1)[1].strip()
            return plan_name or None

        if text in self._known_plans:
            return text

        # Keep compatibility: allow direct plan names even if not listed in mappings.
        return text



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
