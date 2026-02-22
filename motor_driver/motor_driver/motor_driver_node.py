import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .motor_controller import MotorController
from robot_common.command_protocol import parse_command
from robot_common.config_manager import ConfigManager


class MotorDriverNode(Node):
    def __init__(self):
        super().__init__("motor_driver")

        config = ConfigManager("motor_driver", logger=self.get_logger()).load()
        serial_cfg = config.get("serial", {})
        sub_cfg = config.get("subscribe", {})

        port = serial_cfg.get("port", "/dev/ttyUSB0")
        baudrate = serial_cfg.get("baudrate", 115200)
        timeout = serial_cfg.get("timeout", 0.1)
        cmd_topic = sub_cfg.get("cmd_vel", "/cmd_vel")

        self.motor = MotorController(
            port=port,
            baudrate=baudrate,
            timeout=timeout,
            logger=self.get_logger(),
        )

        self.create_subscription(String, cmd_topic, self._cmd_cb, 10)

    def _cmd_cb(self, msg: String):
        direction, speed = parse_command(msg.data)
        if not direction:
            return
        self.motor.move(direction, speed)

    def destroy_node(self):
        self.motor.close()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
