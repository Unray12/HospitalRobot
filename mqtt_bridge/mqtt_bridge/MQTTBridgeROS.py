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
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import Log
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter

from .keyboard_input import KeyboardInput
from .log_bridge import LogBridge
from .log_throttler import LogThrottler
from .plan_resolver import PlanCommandResolver, extract_face_ids


class MQTTBridgeNode(Node):
    """Bridge MQTT commands, ROS topics, keyboard input, and UI-facing status flows."""

    # ── Timing & capacity constants ────────────────────────────────────────────
    # QoS depths
    _QOS_PUB_DEPTH = 10          # Standard publisher queue depth
    _QOS_SUB_DEPTH = 20          # Standard subscriber queue depth
    _QOS_ROSOUT_DEPTH = 100      # /rosout subscriber — keep buffered logs

    # MQTT reconnect backoff (seconds)
    _MQTT_RECONNECT_MIN_DELAY = 1
    _MQTT_RECONNECT_MAX_DELAY = 10
    _MQTT_KEEPALIVE = 60

    # Bridge timer interval (seconds): drain inbound queue every 50 ms
    _BRIDGE_TIMER_INTERVAL = 0.05

    # Echo-suppress dict prune threshold: prune when dict exceeds this size.
    # Well above commands-per-window; rarely hit in the hot path.
    _ECHO_SUPPRESS_PRUNE_THRESHOLD = 64

    # Default log-control fallback values
    _DEFAULT_ECHO_SUPPRESS_WINDOW = 0.35
    _DEFAULT_BRIDGE_TRAFFIC_PERIOD = 1.0
    _DEFAULT_PLAN_STATUS_PERIOD = 0.5
    _DEFAULT_PLAN_MESSAGE_PERIOD = 0.5
    _DEFAULT_HUSKYLENS_FRAME_PERIOD = 2.0

    def __init__(self):
        super().__init__('mqtt_bridge_ros2')
        self.log = LogAdapter(self.get_logger(), "mqtt_bridge")

        config = ConfigManager("mqtt_bridge", logger=self.log).load()
        plan_mapping = ConfigManager("robot_common", "plan_mapping.yaml", logger=self.log).load()

        self._load_config(config, plan_mapping)
        self._setup_ros_pubsub()
        self._setup_mqtt_client(config)
        self._setup_log_bridge(config)
        self._setup_keyboard(config)

        self.log.info("MQTT <-> ROS2 bridge started", event="BOOT")

    # ── Init helpers ───────────────────────────────────────────────────────────

    def _load_config(self, config: dict, plan_mapping: dict) -> None:
        """Extract all config values from YAML into instance attributes."""
        broker_cfg = config.get("broker", {})
        topics_cfg = config.get("topics", {})
        plan_keys = plan_mapping.get("plan_keys", config.get("plan_keys", {}))
        room_plans = plan_mapping.get("room_plans", config.get("room_plans", {}))
        log_control_cfg = config.get("log_control", {})
        camera_face_forward_cfg = config.get("camera_face_forward", {})
        camera_plan_cfg = config.get("camera_face_plan_message", {})

        # Broker
        self.broker_address = broker_cfg.get("address", "127.0.0.1")
        self.broker_port = broker_cfg.get("port", 1883)

        # Topic names
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

        # Plan maps & resolver
        self.plan_key_map = plan_keys
        self.room_plan_map = {str(k).strip().lower(): str(v).strip() for k, v in room_plans.items()}
        if not self.room_plan_map:
            self.room_plan_map = {
                str(k).strip().lower(): str(v).strip()
                for k, v in self.plan_key_map.items()
                if str(k).strip().isdigit()
            }
        self._known_plans = set(self.room_plan_map.values()) | set(self.plan_key_map.values())
        self._plan_resolver = PlanCommandResolver(
            plan_key_map=self.plan_key_map,
            room_plan_map=self.room_plan_map,
            known_plans=self._known_plans,
        )

        # Runtime state
        self._debug_logs_enabled = False
        self._echo_suppress_window_sec = float(
            config.get("echo_suppress_window_sec", self._DEFAULT_ECHO_SUPPRESS_WINDOW)
        )
        self._recent_local_pub = {}
        self._plan_autoline_active = False
        self._active_plan_name = None
        self._inbound_queue = queue.SimpleQueue()
        self._shutdown_requested = False

        # Camera face config
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

        # Log control
        self._log_control = {
            "bridge_in_enabled": bool(log_control_cfg.get("bridge_in_enabled", False)),
            "bridge_out_enabled": bool(log_control_cfg.get("bridge_out_enabled", False)),
            "bridge_traffic_period": float(max(
                0.0, log_control_cfg.get("bridge_traffic_period", self._DEFAULT_BRIDGE_TRAFFIC_PERIOD)
            )),
            "plan_status_enabled": bool(log_control_cfg.get("plan_status_enabled", True)),
            "plan_status_period": float(max(
                0.0, log_control_cfg.get("plan_status_period", self._DEFAULT_PLAN_STATUS_PERIOD)
            )),
            "plan_message_enabled": bool(log_control_cfg.get("plan_message_enabled", True)),
            "plan_message_period": float(max(
                0.0, log_control_cfg.get("plan_message_period", self._DEFAULT_PLAN_MESSAGE_PERIOD)
            )),
            "huskylens_frame_enabled": bool(log_control_cfg.get("huskylens_frame_enabled", False)),
            "huskylens_frame_period": float(max(
                0.0, log_control_cfg.get("huskylens_frame_period", self._DEFAULT_HUSKYLENS_FRAME_PERIOD)
            )),
            "huskylens_valid_enabled": bool(log_control_cfg.get("huskylens_valid_enabled", True)),
            "huskylens_valid_log_on_change": bool(
                log_control_cfg.get("huskylens_valid_log_on_change", True)
            ),
        }
        self._last_log_ts = {}
        self._last_logged_value = {}
        self._log_throttler = LogThrottler()

    def _setup_ros_pubsub(self) -> None:
        """Create all ROS 2 publishers and subscriptions."""
        self.ros_pub = self.create_publisher(String, self.topic_vr, self._QOS_PUB_DEPTH)
        self.ros_pick_pub = self.create_publisher(String, self.topic_pick, self._QOS_PUB_DEPTH)
        self.ros_plan_pub = self.create_publisher(String, self.topic_plan, self._QOS_PUB_DEPTH)
        self.ros_debug_pub = self.create_publisher(Bool, self.topic_debug_toggle, self._QOS_PUB_DEPTH)
        self.create_subscription(String, self.topic_camera_ros, self._camera_cb, self._QOS_SUB_DEPTH)
        self.create_subscription(
            String, self.topic_plan_status_ros, self._plan_status_cb, self._QOS_SUB_DEPTH
        )
        self.create_subscription(
            String, self.topic_plan_message_ros, self._plan_message_cb, self._QOS_SUB_DEPTH
        )
        self.create_subscription(
            String, self.topic_huskylens_frame_ros, self._huskylens_frame_cb, self._QOS_SUB_DEPTH
        )
        self.create_subscription(
            Bool, self.topic_huskylens_valid_ros, self._huskylens_valid_cb, self._QOS_SUB_DEPTH
        )

    def _setup_mqtt_client(self, config: dict) -> None:
        """Initialise paho MQTT client and start async connection + I/O loop."""
        if mqtt is None:
            raise RuntimeError("paho-mqtt is not installed")

        self.client = mqtt.Client()
        self.client.on_connect = self._on_connect
        self.client.on_disconnect = self._on_disconnect
        self.client.on_message = self._on_message
        # Exponential backoff: paho retries 1s → 2s → ... → 10s cap. We pay
        # one wasted reconnect at most every 10s if the broker stays down,
        # acceptable for hospital robot recovery times.
        self.client.reconnect_delay_set(
            min_delay=self._MQTT_RECONNECT_MIN_DELAY,
            max_delay=self._MQTT_RECONNECT_MAX_DELAY,
        )
        self._mqtt_connected = threading.Event()
        self._stop_event = Event()

        try:
            # connect_async returns immediately; paho's loop thread does the
            # actual TCP connect. This means the node boots even when mosquitto
            # is not yet listening (common race: Pi boots both services in
            # parallel and mqtt_bridge wins).
            self.client.connect_async(
                self.broker_address, self.broker_port, self._MQTT_KEEPALIVE
            )
        except Exception as exc:
            self.log.warning(
                f"MQTT initial connect_async failed, will keep retrying: {exc}",
                event="MQTT",
            )

        # paho's loop_start spawns its own daemon thread for I/O. We don't
        # manage it directly; loop_stop in destroy_node tears it down cleanly.
        self.client.loop_start()

    def _setup_log_bridge(self, config: dict) -> None:
        """Set up optional LogBridge that forwards /rosout entries to MQTT."""
        # Match rcl_logging publisher QoS so late-joining subscribers still
        # receive buffered log messages (TRANSIENT_LOCAL).
        self._log_bridge = LogBridge(config, self._mqtt_publish, self.log)
        if self._log_bridge.enabled:
            rosout_qos = QoSProfile(
                depth=self._QOS_ROSOUT_DEPTH,
                history=HistoryPolicy.KEEP_LAST,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.create_subscription(
                Log, self._log_bridge.ros_topic, self._log_bridge.rosout_cb, rosout_qos,
            )

    def _setup_keyboard(self, config: dict) -> None:
        """Start keyboard input thread and the ROS timer that drains the inbound queue."""
        self._keyboard = KeyboardInput(
            config, self.plan_key_map, self.room_plan_map,
            self._inbound_queue, self.log, self._stop_event,
        )
        self._keyboard.start()
        self._bridge_timer = self.create_timer(self._BRIDGE_TIMER_INTERVAL, self._drain_inbound_queue)

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
        # Filter out MQTT echoes of our own publishes BEFORE queueing — the
        # bridge publishes to topics it also subscribes to (VR_control etc.),
        # so without this filter every keyboard press would loop back.
        if self._is_recent_local_echo(msg.topic, data):
            return
        # Hand off to ROS executor thread via a queue. We never touch ROS
        # publishers from paho's thread directly because rclpy publishers are
        # not safe to call cross-thread under all DDS implementations.
        self._inbound_queue.put((msg.topic, data))

    def destroy_node(self):
        self._stop_event.set()
        try:
            self.client.disconnect()
        except Exception as exc:
            self.log.debug(f"MQTT disconnect error (ignored): {exc}", event="MQTT")
        try:
            self.client.loop_stop()
        except Exception as exc:
            self.log.debug(f"MQTT loop_stop error (ignored): {exc}", event="MQTT")
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
            # KeyboardInput formats payloads as "<plan_name>|<mqtt_plan_cmd>".
            # Inbound from MQTT is untrusted — guard against missing separator
            # rather than letting ValueError crash the drain loop.
            if "|" not in data:
                self.log.warning(
                    f"Malformed plan payload (missing '|'): {data!r}",
                    event="BRIDGE_BAD_PAYLOAD",
                )
                return
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
        return extract_face_ids(payload)

    def _resolve_plan_command(self, payload: str):
        resolved = self._plan_resolver.resolve(payload)
        if resolved is None and (payload or "").strip().lower().startswith(("room", "phong", "plan")):
            # Log the same warning as before for room:X with unmapped id.
            text = (payload or "").strip().lower()
            m = re.match(r"^(room|phong|plan)\s*[:/_-]?\s*([a-z0-9_-]+)$", text)
            if m:
                self.log.warning(f"Room id has no mapped plan: {m.group(2).strip()}", event="PLAN_MQTT")
        return resolved

    def _mark_local_publish(self, topic: str, payload: str):
        self._recent_local_pub[(topic, payload)] = time.time()
        # Prune opportunistically. Entries are only deleted on lookup when
        # found stale; commands that never echo back (broker drop, topic not
        # subscribed) would otherwise live forever. 64 is well above the
        # number of commands sent in one echo_suppress_window so we rarely
        # prune in the hot path.
        if len(self._recent_local_pub) > self._ECHO_SUPPRESS_PRUNE_THRESHOLD:
            self._prune_local_pub()

    def _prune_local_pub(self):
        cutoff = time.time() - self._echo_suppress_window_sec
        self._recent_local_pub = {
            k: v for k, v in self._recent_local_pub.items() if v >= cutoff
        }

    def _is_recent_local_echo(self, topic: str, payload: str):
        # Echo identity is (topic, payload). Two distinct identical commands
        # within the window collapse — acceptable for idempotent commands
        # (Forward, Stop) which is the only thing operators spam quickly.
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
        if self._log_throttler.should_emit_throttled(key, period, enabled):
            self.log.info(message, event=event)

    def _log_on_change(self, key: str, value: str, enabled: bool, message: str, event: str):
        if self._log_throttler.should_emit_on_change(key, value, enabled):
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