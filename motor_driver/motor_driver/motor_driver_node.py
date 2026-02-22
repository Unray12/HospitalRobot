import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .motor_controller import MotorController
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
        direction, speed = self._parse_command(msg.data)
        if not direction:
            return
        self.motor.move(direction, speed)

    def _parse_command(self, data: str):
        text = (data or "").strip()
        if not text:
            return None, None

        direction = text
        speed = 0

        for sep in (":", ",", " "):
            if sep in text:
                parts = [p for p in text.split(sep) if p]
                if parts:
                    direction = parts[0].strip()
                if len(parts) > 1:
                    try:
                        speed = int(parts[1])
                    except ValueError:
                        speed = 0
                break

        if direction == "Stop":
            return "Stop", 0

        if direction in ("Forward", "Backward", "Left", "Right", "RotateLeft", "RotateRight"):
            if speed < 0:
                speed = 0
            return direction, int(speed)

        return None, None

    def destroy_node(self):
        self.motor.close()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = MotorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
