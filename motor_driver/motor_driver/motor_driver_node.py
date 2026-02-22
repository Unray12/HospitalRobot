import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool

from .motor_controller import MotorController
from robot_common.command_protocol import parse_command
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver")
        self.log = LogAdapter(self.get_logger(), "motor_driver")

        config = ConfigManager("motor_driver", logger=self.log).load()
        serial_cfg = config.get("serial", {})
        sub_cfg = config.get("subscribe", {})
        topics_cfg = config.get("topics", {})

        port = serial_cfg.get("port", "/dev/ttyUSB0")
        baudrate = serial_cfg.get("baudrate", 115200)
        timeout = serial_cfg.get("timeout", 0.1)
        cmd_topic = sub_cfg.get("cmd_vel", "/cmd_vel")
        debug_toggle_topic = topics_cfg.get("debug_toggle", "/debug_logs_toggle")
        self.debug_log_period = float(config.get("debug_log_period", 0.2))
        self.debug_enabled = bool(config.get("debug_enabled_default", False))
        self._last_debug_log_ts = 0.0

        self.motor = MotorController(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            logger=self.log,
        )

        self.create_subscription(String, cmd_topic, self._cmd_cb, 10)
        self.create_subscription(Bool, debug_toggle_topic, self._debug_toggle_cb, 10)

    def _debug_toggle_cb(self, msg: Bool):
        self.debug_enabled = bool(msg.data)
        state = "ON" if self.debug_enabled else "OFF"
        self.log.info(f"Motor debug log: {state}", event="DEBUG_TOGGLE")

    def _cmd_cb(self, msg: String):
        direction, speed = parse_command(msg.data)
        if not direction:
            return
        wheel_speeds = self.motor.move(direction, speed)
        self._log_motor_debug(direction, speed, wheel_speeds)

    def _log_motor_debug(self, direction, speed, wheel_speeds):
        if not self.debug_enabled or wheel_speeds is None:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        if (now - self._last_debug_log_ts) < self.debug_log_period:
            return
        self._last_debug_log_ts = now
        self.log.info(
            "Wheel speeds",
            event="MOTOR",
            direction=direction,
            speed=speed,
            fl=wheel_speeds[0],
            fr=wheel_speeds[1],
            rl=wheel_speeds[2],
            rr=wheel_speeds[3],
        )

    def destroy_node(self):
        self.motor.close()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
