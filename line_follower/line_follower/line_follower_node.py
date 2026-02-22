import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Bool, String
from std_srvs.srv import SetBool

from .line_follower import LineFollowerFSM
from robot_common.command_protocol import format_command
from robot_common.config_manager import ConfigManager


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower")

        config = ConfigManager("line_follower", logger=self.get_logger()).load()
        topics_cfg = config.get("topics", {})
        service_cfg = config.get("service", {})
        self.plan_alias = config.get("plan_alias", {})

        self.base_speed = int(config.get("base_speed", 6))
        self.plan_select_debounce_sec = float(config.get("plan_select_debounce_sec", 0.35))
        self.autoMode = False
        self._last_frame = None
        self._last_plan_name = None
        self._last_plan_ts = 0.0

        cfg_mgr = ConfigManager("line_follower", logger=self.get_logger())
        plan_name = config.get("cross_plan_name")
        plan_data = cfg_mgr.load_plan(plan_name) if plan_name else None
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
            logger=self.get_logger(),
        )

        cmd_topic = topics_cfg.get("cmd_vel", "/cmd_vel")
        frame_topic = topics_cfg.get("line_frame", "/line_sensors/frame")
        auto_topic = topics_cfg.get("auto_mode", "/auto_mode")
        plan_topic = topics_cfg.get("plan_select", "/plan_select")
        auto_service = service_cfg.get("set_auto_mode", "/set_auto_mode")

        self.cmd_pub = self.create_publisher(String, cmd_topic, 10)
        self.create_subscription(Int16MultiArray, frame_topic, self._frame_cb, 10)
        self.create_subscription(Bool, auto_topic, self._auto_cb, 10)
        self.create_subscription(String, plan_topic, self._plan_cb, 10)
        self.create_service(SetBool, auto_service, self._auto_srv_cb)

        self.timer = self.create_timer(0.01, self._timer_cb)

    def _frame_cb(self, msg: Int16MultiArray):
        if len(msg.data) < 6:
            self.get_logger().warning("Line frame invalid length")
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
            self.get_logger().info("Plan cleared")
            return
        if name in self.plan_alias:
            name = self.plan_alias[name]

        now = time.time()
        if (
            self._last_plan_name == name
            and (now - self._last_plan_ts) < self.plan_select_debounce_sec
        ):
            self.get_logger().info(f"Plan duplicate ignored: {name}")
            return

        cfg_mgr = ConfigManager("line_follower", logger=self.get_logger())
        plan_data = cfg_mgr.load_plan(name)
        if not plan_data:
            self.get_logger().warning(f"Plan not found: {name}")
            return

        steps = plan_data.get("steps", [])
        end_state = plan_data.get("end_state", "stop")
        self.follower.set_plan(steps, end_state)
        self._last_plan_name = name
        self._last_plan_ts = now
        if self.autoMode:
            self.follower.reset()
        self.get_logger().info(f"Plan selected: {name}")

    def _set_auto_mode(self, enabled: bool):
        if enabled and not self.autoMode:
            self.autoMode = True
            self.follower.reset()
            self.get_logger().info("Auto Mode Enabled")
        elif not enabled and self.autoMode:
            self.autoMode = False
            self.follower.stop()
            self._publish_stop()
            self.get_logger().info("Auto Mode Disabled")

    def _timer_cb(self):
        if not self.autoMode:
            return

        result = self.follower.update(self._last_frame, time.time())
        if result is None:
            return

        direction, speed = result
        self.cmd_pub.publish(self._command_to_msg(direction, speed))

    def _command_to_msg(self, direction, speed):
        command = format_command(direction, speed)
        if command is None:
            command = "Stop:0"
        msg = String()
        msg.data = command
        return msg

    def _publish_stop(self):
        self.cmd_pub.publish(self._command_to_msg("Stop", 0))



def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
