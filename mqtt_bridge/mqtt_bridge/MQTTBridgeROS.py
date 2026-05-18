#!/usr/bin/python3
"""MQTT/ROS2 bridge with keyboard control, plan routing, and status forwarding."""

import json
import queue
import re
import threading
import time
from threading import Event

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import Log
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter

from .keyboard_input import KeyboardInput
from .log_bridge import LogBridge


class MQTTBridgeNode(Node):
    """Bridge MQTT commands, ROS topics, keyboard input, and UI-facing status flows."""

    def __init__(self):
        super().__init__('mqtt_bridge_ros2')
        self.log = LogAdapter(self.get_logger(), "mqtt_bridge")

        config = ConfigManager("mqtt_bridge", logger=self.log).load()
        broker_cfg = config.get("broker", {})
        topics_cfg = config.get("topics", {})
        plan_keys = config.get("plan_keys", {})
        room_plans = config.get("room_plans", {})
        log_control_cfg = config.get("log_control", {})
        camera_face_forward_cfg = config.get("camera_face_forward", {})
        camera_plan_cfg = config.get("camera_face_plan_message", {})

        self.broker_address = broker_cfg.get("address", "127.0.0.1")
        self.broker_port = broker_cfg.get("port", 1883)
        self.topic_vr = topics_cfg.get("vr_control", "VR_control")
        self.topic_pick = topics_cfg.get("pick_robot", "pick_robot")
        self.topic_plan = topics_cfg.get("plan_select", "plan_select")
        self.topic_debug_toggle = topics_cfg.get("debug_toggle", "/debug_logs_toggle")
        self.topic_log_out = topics_cfg.get("robot_logs", "robot_logs")
        self.topic_camera_ros = topics_cfg.get("camera_face_ros", "/face/camera")
        self.topic_camera_mqtt = topics_cfg.get("camera_face_mqtt", "face/camera")
        self.topic_plan_status_ros = topics_cfg.get("plan_status_ros", "/plan_status")
        self.topic_plan_status_mqtt = topics_cfg.get("plan_status_mqtt", "plan_status")
        self.topic_plan_message_ros = topics_cfg.get("plan_message_ros", "/plan_message")
        self.topic_plan_message_mqtt = topics_cfg.get("plan_message_mqtt", "plan_message")
        self.topic_huskylens_frame_ros = topics_cfg.get("huskylens_frame_ros", "/huskylens/frame")
        self.topic_huskylens_frame_mqtt = topics_cfg.get("huskylens_frame_mqtt", "huskylens/frame")
        self.topic_huskylens_valid_ros = topics_cfg.get("huskylens_valid_ros", "/huskylens/valid")
        self.topic_huskylens_valid_mqtt = topics_cfg.get("huskylens_valid_mqtt", "huskylens/valid")
        self.plan_key_map = plan_keys
        self.room_plan_map = {str(k).strip().lower(): str(v).strip() for k, v in room_plans.items()}
        if not self.room_plan_map:
            self.room_plan_map = {
                str(k).strip().lower(): str(v).strip()
                for k, v in self.plan_key_map.items()
                if str(k).strip().isdigit()
            }
        self._known_plans = set(self.room_plan_map.values()) | set(self.plan_key_map.values())
        self._debug_logs_enabled = False
        self._echo_suppress_window_sec = float(config.get("echo_suppress_window_sec", 0.35))
        self._recent_local_pub = {}
        self._camera_face_plan_enabled = bool(camera_plan_cfg.get("enabled", False))
        self._camera_face_require_plan_autoline = bool(
            camera_plan_cfg.get("require_plan_autoline", True)
        )
        self._camera_face_forward_enabled = bool(camera_face_forward_cfg.get("enabled", True))
        self._camera_face_plan_messages = {
            str(k).strip(): str(v)
            for k, v in camera_plan_cfg.get("id_messages", {}).items()
            if str(k).strip() and str(v).strip()
        }
        self._plan_autoline_active = False
        self._active_plan_name = None
        self._log_control = {
            "bridge_in_enabled": bool(log_control_cfg.get("bridge_in_enabled", False)),
            "bridge_out_enabled": bool(log_control_cfg.get("bridge_out_enabled", False)),
            "bridge_traffic_period": float(max(0.0, log_control_cfg.get("bridge_traffic_period", 1.0))),
            "plan_status_enabled": bool(log_control_cfg.get("plan_status_enabled", True)),
            "plan_status_period": float(max(0.0, log_control_cfg.get("plan_status_period", 0.5))),
            "plan_message_enabled": bool(log_control_cfg.get("plan_message_enabled", True)),
            "plan_message_period": float(max(0.0, log_control_cfg.get("plan_message_period", 0.5))),
            "huskylens_frame_enabled": bool(log_control_cfg.get("huskylens_frame_enabled", False)),
            "huskylens_frame_period": float(max(0.0, log_control_cfg.get("huskylens_frame_period", 2.0))),
            "huskylens_valid_enabled": bool(log_control_cfg.get("huskylens_valid_enabled", True)),
            "huskylens_valid_log_on_change": bool(log_control_cfg.get("huskylens_valid_log_on_change", True)),
        }
        self._last_log_ts = {}
        self._last_logged_value = {}
        self._inbound_queue = queue.SimpleQueue()
        self._shutdown_requested = False

        # ROS 2 publisher
        self.ros_pub = self.create_publisher(String, self.topic_vr, 10)
        self.ros_pick_pub = self.create_publisher(String, self.topic_pick, 10)
        self.ros_plan_pub = self.create_publisher(String, self.topic_plan, 10)
        self.ros_debug_pub = self.create_publisher(Bool, self.topic_debug_toggle, 10)
        self.create_subscription(String, self.topic_camera_ros, self._camera_cb, 20)
        self.create_subscription(String, self.topic_plan_status_ros, self._plan_status_cb, 20)
        self.create_subscription(String, self.topic_plan_message_ros, self._plan_message_cb, 20)
        self.create_subscription(String, self.topic_huskylens_frame_ros, self._huskylens_frame_cb, 20)
        self.create_subscription(Bool, self.topic_huskylens_valid_ros, self._huskylens_valid_cb, 20)

        if mqtt is None:
            raise RuntimeError("paho-mqtt is not installed")

        # MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        self._mqtt_connected = threading.Event()

        self.client.connect(self.broker_address, self.broker_port, 60)

        # MQTT chạy ở thread riêng
        self._stop_event = Event()
        self._mqtt_thread = threading.Thread(target=self.client.loop_forever, daemon=True)
        self._mqtt_thread.start()

        # Log bridge
        self._log_bridge = LogBridge(config, self._mqtt_publish, self.log)
        if self._log_bridge.enabled:
            self.create_subscription(Log, self._log_bridge.ros_topic, self._log_bridge.rosout_cb, 100)

        # Keyboard
        self._keyboard = KeyboardInput(
            config, plan_keys, self.room_plan_map,
            self._inbound_queue, self.log, self._stop_event,
        )
        self._keyboard.start()
        self._bridge_timer = self.create_timer(0.02, self._drain_inbound_queue)

        self.log.info("MQTT <-> ROS2 bridge started", event="BOOT")

    def _mqtt_publish(self, text):
        self.client.publish(self.topic_log_out, text)

    # MQTT callbacks
    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._mqtt_connected.set()
            self.log.info("Connected to MQTT broker", event="MQTT")
            client.subscribe(self.topic_vr)
            client.subscribe(self.topic_pick)
            client.subscribe(self.topic_plan)
        else:
            self.log.error(f"MQTT connect failed: {rc}", event="MQTT")

    def _on_disconnect(self, client, userdata, rc):
        self._mqtt_connected.clear()
        self.log.warning(f"Disconnected from MQTT broker: rc={rc}", event="MQTT")

    def _on_message(self, client, userdata, msg):
        data = msg.payload.decode(errors="replace")
        if self._is_recent_local_echo(msg.topic, data):
            return
        self._inbound_queue.put((msg.topic, data))

    def destroy_node(self):
        self._stop_event.set()
        try:
            self.client.disconnect()
        except Exception:
            pass
        for thread in (getattr(self, "_mqtt_thread", None),):
            if thread and thread.is_alive():
                thread.join(timeout=0.5)
        super().destroy_node()

    def _drain_inbound_queue(self):
        while True:
            try:
                topic, data = self._inbound_queue.get_nowait()
            except Exception:
                break
            self._handle_inbound_event(topic, data)

        if self._shutdown_requested and rclpy.ok():
            rclpy.shutdown()

    def _handle_inbound_event(self, topic, data):
        if topic == "__keyboard_vr__":
            self.ros_pub.publish(String(data=data))
            self.client.publish(self.topic_vr, data)
            self._mark_local_publish(self.topic_vr, data)
            self._log_throttled(
                key="bridge_out:vr",
                period=self._log_control["bridge_traffic_period"],
                enabled=self._log_control["bridge_out_enabled"],
                message=f"ROS2 -> MQTT vr: {data}",
                event="BRIDGE_OUT",
            )
            return

        if topic == "__keyboard_pick__":
            self.ros_pick_pub.publish(String(data=data))
            self.client.publish(self.topic_pick, data)
            self._mark_local_publish(self.topic_pick, data)
            self._log_throttled(
                key="bridge_out:pick",
                period=self._log_control["bridge_traffic_period"],
                enabled=self._log_control["bridge_out_enabled"],
                message=f"ROS2 -> MQTT pick: {data}",
                event="BRIDGE_OUT",
            )
            return

        if topic == "__keyboard_debug__":
            self._debug_logs_enabled = not self._debug_logs_enabled
            self.ros_debug_pub.publish(Bool(data=self._debug_logs_enabled))
            state = "ON" if self._debug_logs_enabled else "OFF"
            self.log.info(
                f"Debug logs toggled: {state}", event="DEBUG_TOGGLE",
            )
            return

        if topic == "__keyboard_plan__":
            plan_name, mqtt_plan_cmd = data.split("|", 1)
            if self._mqtt_connected.is_set():
                self.ros_plan_pub.publish(String(data=plan_name))
                self.client.publish(self.topic_plan, mqtt_plan_cmd)
                self._mark_local_publish(self.topic_plan, mqtt_plan_cmd)
                self._log_throttled(
                    key="bridge_out:plan",
                    period=self._log_control["bridge_traffic_period"],
                    enabled=self._log_control["bridge_out_enabled"],
                    message=f"ROS2 -> MQTT plan: {mqtt_plan_cmd}",
                    event="BRIDGE_OUT",
                )
            else:
                self.ros_plan_pub.publish(String(data=plan_name))
                self.log.warning(
                    f"MQTT unavailable, published plan locally: {plan_name}",
                    event="BRIDGE_FALLBACK",
                )
            return

        if self._is_recent_local_echo(topic, data):
            return
        self._log_throttled(
            key=f"bridge_in:{topic}",
            period=self._log_control["bridge_traffic_period"],
            enabled=self._log_control["bridge_in_enabled"],
            message=f"MQTT -> ROS2 [{topic}]: {data}",
            event="BRIDGE_IN",
        )

        ros_msg = String()
        if topic == self.topic_vr:
            ros_msg.data = data
            self.ros_pub.publish(ros_msg)
        elif topic == self.topic_pick:
            ros_msg.data = data
            self.ros_pick_pub.publish(ros_msg)
        elif topic == self.topic_plan:
            resolved = self._resolve_plan_command(data)
            if not resolved:
                self.log.warning(f"Ignored invalid plan command: {data}", event="PLAN_MQTT")
                return
            ros_msg.data = resolved
            self.ros_plan_pub.publish(ros_msg)
            if resolved != data:
                self.log.info(f"Normalized plan command: {data} -> {resolved}", event="PLAN_MQTT")

    def _plan_status_cb(self, msg: String):
        if not self._mqtt_connected.is_set():
            return
        payload = str(msg.data or "").strip()
        if not payload:
            return
        self._update_plan_runtime_state(payload)
        self.client.publish(self.topic_plan_status_mqtt, payload)
        self._log_throttled(
            key="plan_status",
            period=self._log_control["plan_status_period"],
            enabled=self._log_control["plan_status_enabled"],
            message=f"ROS2 -> MQTT plan status: {payload}",
            event="PLAN_STATUS_MQTT",
        )

    def _plan_message_cb(self, msg: String):
        if not self._mqtt_connected.is_set():
            return
        payload = str(msg.data or "").strip()
        if not payload:
            return
        self.client.publish(self.topic_plan_message_mqtt, payload)
        self._log_throttled(
            key="plan_message",
            period=self._log_control["plan_message_period"],
            enabled=self._log_control["plan_message_enabled"],
            message=f"ROS2 -> MQTT plan message: {payload}",
            event="PLAN_MESSAGE_MQTT",
        )

    def _camera_cb(self, msg: String):
        if not self._mqtt_connected.is_set():
            return
        payload = str(msg.data or "")
        if payload == "":
            return
        if self._camera_face_forward_enabled:
            self.client.publish(self.topic_camera_mqtt, payload)
        self._publish_camera_plan_message(payload)

    def _huskylens_frame_cb(self, msg: String):
        if not self._mqtt_connected.is_set():
            return
        payload = str(msg.data or "").strip()
        if not payload:
            return
        self.client.publish(self.topic_huskylens_frame_mqtt, payload)
        self._log_throttled(
            key="huskylens_frame",
            period=self._log_control["huskylens_frame_period"],
            enabled=self._log_control["huskylens_frame_enabled"],
            message=f"ROS2 -> MQTT huskylens frame: {payload}",
            event="HUSKYLENS_MQTT",
        )

    def _huskylens_valid_cb(self, msg: Bool):
        if not self._mqtt_connected.is_set():
            return
        payload = "1" if bool(msg.data) else "0"
        self.client.publish(self.topic_huskylens_valid_mqtt, payload)
        if self._log_control["huskylens_valid_log_on_change"]:
            self._log_on_change(
                key="huskylens_valid",
                value=payload,
                enabled=self._log_control["huskylens_valid_enabled"],
                message=f"ROS2 -> MQTT huskylens valid: {payload}",
                event="HUSKYLENS_MQTT",
            )
        else:
            self._log_throttled(
                key="huskylens_valid",
                period=self._log_control["huskylens_frame_period"],
                enabled=self._log_control["huskylens_valid_enabled"],
                message=f"ROS2 -> MQTT huskylens valid: {payload}",
                event="HUSKYLENS_MQTT",
            )

    def _publish_camera_plan_message(self, payload: str):
        if not self._camera_face_plan_enabled:
            return
        if self._camera_face_require_plan_autoline:
            if (not self._plan_autoline_active) or (not self._active_plan_name):
                return
        face_ids = self._extract_face_ids(payload)
        if not face_ids:
            return
        for face_id in face_ids:
            message = self._camera_face_plan_messages.get(str(face_id))
            if not message:
                continue
            self.client.publish(self.topic_plan_message_mqtt, message)
            self.log.info(
                f"Camera FACE id={face_id} -> MQTT {self.topic_plan_message_mqtt}: {message}",
                event="CAMERA_PLAN_MSG",
            )

    def _update_plan_runtime_state(self, payload: str):
        try:
            data = json.loads(payload)
        except Exception:
            return
        if not isinstance(data, dict):
            return

        event_name = str(data.get("event") or "").strip().lower()
        plan_name = data.get("plan")
        autoline = data.get("autoline")

        if plan_name:
            self._active_plan_name = str(plan_name).strip()

        if autoline is not None:
            self._plan_autoline_active = bool(autoline)

        if event_name in {"cleared", "completed_reset"}:
            self._active_plan_name = None
            self._plan_autoline_active = False

    def _extract_face_ids(self, payload: str):
        text = (payload or "").strip().upper()
        if "FACE" not in text:
            return []
        m = re.search(r"FACE\s*,\s*([0-9,;|/ ]+)", text)
        if not m:
            return []
        ids = []
        for token in re.findall(r"\d+", m.group(1)):
            try:
                face_id = int(token)
            except ValueError:
                continue
            ids.append(face_id)
        return ids

    def _resolve_plan_command(self, payload: str):
        text = (payload or "").strip()
        if not text:
            return None

        lower = text.lower()
        if lower in {"0", "clear", "room:0", "room/0"}:
            return "clear"

        if text in self.plan_key_map:
            return self.plan_key_map[text]
        if lower in self.room_plan_map:
            return self.room_plan_map[lower]

        m = re.match(r"^(room|phong|plan)\s*[:/_-]?\s*([a-z0-9_-]+)$", lower)
        if m:
            prefix = m.group(1).strip()
            room_id = m.group(2).strip()
            if room_id in {"0", "clear"}:
                return "clear"
            if room_id in self.room_plan_map:
                return self.room_plan_map[room_id]
            if prefix == "plan" and room_id.startswith("plan_"):
                return room_id
            self.log.warning(f"Room id has no mapped plan: {room_id}", event="PLAN_MQTT")
            return None

        if lower.startswith("plan:") or lower.startswith("plan/"):
            plan_name = text.split(":", 1)[1].strip() if ":" in text else text.split("/", 1)[1].strip()
            return plan_name or None

        if text in self._known_plans:
            return text

        return text

    def _mark_local_publish(self, topic: str, payload: str):
        self._recent_local_pub[(topic, payload)] = time.time()

    def _is_recent_local_echo(self, topic: str, payload: str):
        now = time.time()
        key = (topic, payload)
        ts = self._recent_local_pub.get(key)
        if ts is None:
            return False
        if (now - ts) <= self._echo_suppress_window_sec:
            return True
        del self._recent_local_pub[key]
        return False

    def _log_throttled(self, key: str, period: float, enabled: bool, message: str, event: str):
        if not enabled:
            return
        now = time.time()
        last = self._last_log_ts.get(key, 0.0)
        if (now - last) < max(0.0, float(period)):
            return
        self._last_log_ts[key] = now
        self.log.info(message, event=event)

    def _log_on_change(self, key: str, value: str, enabled: bool, message: str, event: str):
        if not enabled:
            return
        previous = self._last_logged_value.get(key)
        if previous == value:
            return
        self._last_logged_value[key] = value
        self.log.info(message, event=event)


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