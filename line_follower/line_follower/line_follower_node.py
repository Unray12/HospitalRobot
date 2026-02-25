import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Bool, String
from std_srvs.srv import SetBool

from .line_follower import LineFollowerFSM
from robot_common.command_protocol import format_command
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower")
        self.log = LogAdapter(self.get_logger(), "line_follower")

        config = ConfigManager("line_follower", logger=self.log).load()
        topics_cfg = config.get("topics", {})
        service_cfg = config.get("service", {})
        self.plan_alias = config.get("plan_alias", {})
        self.auto_on_plan_select_default = bool(config.get("auto_on_plan_select", True))

        self.base_speed = int(config.get("base_speed", 6))
        self.plan_select_debounce_sec = float(config.get("plan_select_debounce_sec", 0.35))
        self.autoMode = False
        self._last_frame = None
        self._last_plan_name = None
        self._last_plan_ts = 0.0
        self._active_plan_name = None
        self._last_plan_status_text = None
        self._last_plan_status_log_ts = 0.0
        self.plan_status_log_period = float(config.get("plan_status_log_period", 0.8))
        self._active_plan_autoline = None
        self._plan_completion_reported = False

        cfg_mgr = ConfigManager("line_follower", logger=self.log)
        plan_name = config.get("cross_plan_name")
        plan_data = cfg_mgr.load_plan(plan_name, force=True) if plan_name else None
        plan_steps = plan_data.get("steps", []) if plan_data else config.get("cross_plan", [])
        plan_end_state = plan_data.get("end_state", config.get("plan_end_state", "stop")) if plan_data else config.get("plan_end_state", "stop")

        self.follower = LineFollowerFSM(
            base_speed=self.base_speed,
            turn_speed_left=config.get("turn_speed_left", self.base_speed),
            turn_speed_right=config.get("turn_speed_right", self.base_speed),
            crossing_duration=config.get("crossing_duration", 2.0),
            cross_plan=plan_steps,
            plan_end_state=plan_end_state,
            cross_pre_forward_speed=config.get("cross_pre_forward_speed", 8),
            cross_pre_forward_duration=config.get("cross_pre_forward_duration", 2.0),
            cross_pre_stop_duration=config.get("cross_pre_stop_duration", 1.0),
            rotate_min_duration=config.get("rotate_min_duration", 0.5),
            rotate_line_mid_min_count=config.get("rotate_line_mid_min_count", 1),
            rotate_line_side_max_count=config.get("rotate_line_side_max_count", 2),
            rotate_early_stop_on_side=config.get("rotate_early_stop_on_side", True),
            rotate_line_side_min_count=config.get("rotate_line_side_min_count", 1),
            plan_lost_line_hold_sec=config.get("plan_lost_line_hold_sec", 0.8),
            plan_lost_line_warn_period=config.get("plan_lost_line_warn_period", 0.5),
            logger=self.log,
        )
        if plan_name:
            self._active_plan_name = plan_name

        cmd_topic = topics_cfg.get("cmd_vel", "/cmd_vel")
        frame_topic = topics_cfg.get("line_frame", "/line_sensors/frame")
        auto_topic = topics_cfg.get("auto_mode", "/auto_mode")
        plan_topic = topics_cfg.get("plan_select", "/plan_select")
        pick_topic = topics_cfg.get("pick_robot", "/pick_robot")
        plan_status_topic = topics_cfg.get("plan_status", "/plan_status")
        plan_callback_topic = topics_cfg.get("plan_callback", "/plan_callback")
        auto_service = service_cfg.get("set_auto_mode", "/set_auto_mode")
        self.plan_status_topic = plan_status_topic
        self._string_publishers = {}

        self.cmd_pub = self.create_publisher(String, cmd_topic, 10)
        self.pick_pub = self.create_publisher(String, pick_topic, 10)
        self.plan_status_pub = self.create_publisher(String, plan_status_topic, 10)
        self.plan_callback_pub = self.create_publisher(String, plan_callback_topic, 10)
        self.create_subscription(Int16MultiArray, frame_topic, self._frame_cb, 10)
        self.create_subscription(Bool, auto_topic, self._auto_cb, 10)
        self.create_subscription(String, plan_topic, self._plan_cb, 10)
        self.create_service(SetBool, auto_service, self._auto_srv_cb)

        self.timer = self.create_timer(0.01, self._timer_cb)

    def _frame_cb(self, msg: Int16MultiArray):
        if len(msg.data) < 6:
            self.log.warning("Line frame invalid length", event="FRAME")
            return

        self._last_frame = {
            "left_count": msg.data[0],
            "mid_count": msg.data[1],
            "right_count": msg.data[2],
            "left_full": bool(msg.data[3]),
            "mid_full": bool(msg.data[4]),
            "right_full": bool(msg.data[5]),
        }

    def _auto_cb(self, msg: Bool):
        self._set_auto_mode(msg.data)

    def _auto_srv_cb(self, request, response):
        self._set_auto_mode(request.data)
        response.success = True
        response.message = "Auto mode updated"
        return response

    def _plan_cb(self, msg: String):
        name = msg.data.strip()
        if not name:
            return
        if name == "0" or name.lower() == "clear":
            self.follower.clear_plan()
            self.follower.set_autoline_mode(False)
            self.pick_pub.publish(String(data="0"))
            self._set_auto_mode(False)
            cleared_plan = self._active_plan_name
            self._active_plan_name = None
            self._active_plan_autoline = None
            self._plan_completion_reported = False
            self._publish_plan_status_event("cleared", plan_name=cleared_plan, status=self.follower.get_plan_status())
            self.log.info("Plan cleared", event="PLAN")
            return
        if name in self.plan_alias:
            name = self.plan_alias[name]

        now = time.time()
        if (
            self._last_plan_name == name
            and (now - self._last_plan_ts) < self.plan_select_debounce_sec
        ):
            self.log.info(f"Plan duplicate ignored: {name}", event="PLAN")
            return

        cfg_mgr = ConfigManager("line_follower", logger=self.log)
        plan_data = cfg_mgr.load_plan(name, force=True)
        if not plan_data:
            self.log.warning(f"Plan not found: {name}", event="PLAN")
            return

        steps = plan_data.get("steps", [])
        end_state = plan_data.get("end_state", "stop")
        auto_flag_raw = (
            plan_data.get("autoline")
            if "autoline" in plan_data
            else plan_data.get("auto_on_select")
        )
        auto_on_select = self._to_bool(auto_flag_raw, self.auto_on_plan_select_default)
        start_without_cross = self._to_bool(
            plan_data.get("start_without_cross", plan_data.get("start_immediately")),
            False,
        )
        self.follower.set_plan(steps, end_state)
        self.follower.set_autoline_mode(auto_on_select)
        self._active_plan_name = name
        self._active_plan_autoline = auto_on_select
        self._plan_completion_reported = False
        self._last_plan_name = name
        self._last_plan_ts = now
        self._publish_plan_status_event("selected", status=self.follower.get_plan_status())
        if auto_on_select:
            self.pick_pub.publish(String(data="1"))
            self._set_auto_mode(True)
            self._publish_plan_status_event("autoline_enabled", status=self.follower.get_plan_status())
            self.log.info(f"Auto enabled by plan select: {name}", event="PLAN")
        elif self.autoMode:
            self.follower.reset()
        if start_without_cross:
            if self.follower.request_plan_start():
                self._publish_plan_status_event("triggered_without_cross", status=self.follower.get_plan_status())
                self.log.info(f"Plan triggered without cross: {name}", event="PLAN")
        self.log.info(f"Plan selected: {name}", event="PLAN")

    def _set_auto_mode(self, enabled: bool):
        if enabled and not self.autoMode:
            self.autoMode = True
            self.follower.reset()
            self.log.info("Auto Mode Enabled", event="MODE")
        elif not enabled and self.autoMode:
            self.autoMode = False
            self.follower.stop()
            self._publish_stop()
            self.log.info("Auto Mode Disabled", event="MODE")

    def _timer_cb(self):
        if not self.autoMode:
            self._check_and_publish_plan_completed()
            return

        now = time.time()
        self._consume_autoline_action()
        self._consume_step_messages()
        result = self.follower.update(self._last_frame, now)
        self._consume_autoline_action()
        self._consume_step_messages()
        if result is None:
            self._log_plan_status(now)
            self._check_and_publish_plan_completed()
            return

        direction, speed = result
        self.cmd_pub.publish(self._command_to_msg(direction, speed))
        self._log_plan_status(now)
        self._check_and_publish_plan_completed()

    def _consume_autoline_action(self):
        requested = self.follower.consume_requested_autoline()
        if requested is None:
            return
        self._active_plan_autoline = bool(requested)
        self.follower.set_autoline_mode(bool(requested))
        self._on_autoline_step_callback(bool(requested))
        self._publish_plan_status_event(
            "autoline_step",
            status=self.follower.get_plan_status(),
        )
        if requested:
            self.pick_pub.publish(String(data="1"))
            self._set_auto_mode(True)
        else:
            self.pick_pub.publish(String(data="0"))
            self._set_auto_mode(False)

    def _consume_step_messages(self):
        entries = self.follower.consume_requested_step_messages()
        if not entries:
            return
        status = self.follower.get_plan_status()
        for item in entries:
            if not isinstance(item, dict):
                continue
            topic = str(item.get("topic") or "").strip()
            if not topic:
                continue
            message = item.get("message")
            if message is None:
                message = item.get("payload", "")
            if isinstance(message, (dict, list)):
                payload = json.dumps(message, ensure_ascii=False, separators=(",", ":"))
            else:
                payload = str(message)
            pub = self._get_string_publisher(topic)
            pub.publish(String(data=payload))
            self.log.info(
                f"[PLAN_MSG] topic={topic} payload={payload}",
                event="PLAN_MESSAGE",
            )
            if topic == self.plan_status_topic:
                self.log.info(
                    f"ROS2 -> MQTT plan status: {payload}",
                    event="PLAN_STATUS_MQTT",
                )
        self._publish_plan_status_event("step_message_sent", status=status)

    def _get_string_publisher(self, topic: str):
        pub = self._string_publishers.get(topic)
        if pub is None:
            pub = self.create_publisher(String, topic, 10)
            self._string_publishers[topic] = pub
        return pub

    def _on_autoline_step_callback(self, enabled: bool):
        """
        Callback hook when plan step AutoLine is applied.
        Add your custom function call here (service call, extra publish, etc.).
        """
        state = "ON" if enabled else "OFF"
        self.log.info(
            f"[CALLBACK] AutoLine step applied -> {state} (plan={self._active_plan_name})",
            event="AUTOLINE_CALLBACK",
        )
        self._publish_plan_status_event(
            "autoline_callback",
            status=self.follower.get_plan_status(),
        )
        self._my_function_after_rotate_to_autoline(enabled)

    def _my_function_after_rotate_to_autoline(self, enabled: bool):
        """
        Custom hook after Rotate...until-line transitions to AutoLine step.
        Publishes structured callback payload to /plan_callback for MQTT bridge/UI/backend.
        """
        status = self.follower.get_plan_status()
        payload = {
            "event": "rotate_to_autoline",
            "plan": self._active_plan_name,
            "autoline_enabled": bool(enabled),
            "state": status.get("state"),
            "step": status.get("next_step"),
            "total_steps": status.get("total_steps"),
            "action": status.get("current_action"),
            "end_state": status.get("end_state"),
            "ts": round(time.time(), 3),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.plan_callback_pub.publish(msg)
        self.log.info(msg.data, event="PLAN_CALLBACK")

    def _command_to_msg(self, direction, speed):
        command = format_command(direction, speed)
        if command is None:
            command = "Stop:0"
        msg = String()
        msg.data = command
        return msg

    def _publish_stop(self):
        self.cmd_pub.publish(self._command_to_msg("Stop", 0))

    def _log_plan_status(self, now):
        status = self.follower.get_plan_status()
        if not status.get("has_plan"):
            return
        plan_name = self._active_plan_name or "unknown"
        text = (
            f"{plan_name} | state={status['state']} | "
            f"step={status['next_step']}/{status['total_steps']} | "
            f"action={status['current_action']} | end_state={status['end_state']}"
        )
        period_ok = (now - self._last_plan_status_log_ts) >= self.plan_status_log_period
        changed = text != self._last_plan_status_text
        if changed or period_ok:
            self.log.info(text, event="PLAN_STATUS")
            self._last_plan_status_text = text
            self._last_plan_status_log_ts = now

    def _check_and_publish_plan_completed(self):
        if not self._active_plan_name or self._plan_completion_reported:
            return

        status = self.follower.get_plan_status()
        done = bool(status.get("plan_done", False))
        if not done:
            return

        done_plan = self._active_plan_name
        self._plan_completion_reported = True
        self._publish_plan_status_event("completed", status=status)

        # Return to normal idle behavior after completed stop-plan.
        end_state = str(status.get("end_state", "") or "").strip().lower()
        if end_state == "follow":
            return

        self._set_auto_mode(False)
        self.follower.clear_plan()
        self.follower.set_autoline_mode(False)
        self.pick_pub.publish(String(data="0"))
        self._active_plan_autoline = False
        self._active_plan_name = None
        self._publish_plan_status_event(
            "completed_reset",
            status=self.follower.get_plan_status(),
            plan_name=done_plan,
        )
        self.log.info(f"Plan completed and reset: {done_plan}", event="PLAN")

    def _publish_plan_status_event(self, event_name, status=None, plan_name=None):
        payload = {
            "event": str(event_name),
            "plan": (plan_name if plan_name is not None else self._active_plan_name),
            "autoline": bool(self._active_plan_autoline) if self._active_plan_autoline is not None else None,
        }
        if status:
            payload["state"] = status.get("state")
            payload["step"] = status.get("next_step")
            payload["total_steps"] = status.get("total_steps")
            payload["action"] = status.get("current_action")
            payload["end_state"] = status.get("end_state")

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
        self.plan_status_pub.publish(msg)
        self.log.info(msg.data, event="PLAN_EVENT")

    def _to_bool(self, value, default):
        if value is None:
            return default
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            text = value.strip().lower()
            if text in {"1", "true", "yes", "on"}:
                return True
            if text in {"0", "false", "no", "off"}:
                return False
        return bool(value)



def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
