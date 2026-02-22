import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool

from robot_common.config_manager import ConfigManager

class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control")

        config = ConfigManager("manual_control", logger=self.get_logger()).load()
        topics_cfg = config.get("topics", {})
        service_cfg = config.get("service", {})

        self.base_speed = int(config.get("base_speed", 6))
        self.autoMode = False

        cmd_topic = topics_cfg.get("cmd_vel", "/cmd_vel")
        auto_topic = topics_cfg.get("auto_mode", "/auto_mode")
        vr_topic = topics_cfg.get("vr_control", "/VR_control")
        pick_topic = topics_cfg.get("pick_robot", "/pick_robot")
        auto_service = service_cfg.get("set_auto_mode", "/set_auto_mode")

        self.cmd_pub = self.create_publisher(String, cmd_topic, 10)
        self.auto_pub = self.create_publisher(Bool, auto_topic, 10)
        self.auto_client = self.create_client(SetBool, auto_service)

        self.create_subscription(String, vr_topic, self._manual_cb, 10)
        self.create_subscription(String, pick_topic, self._pick_cb, 10)

    def _manual_cb(self, msg: String):
        if self.autoMode:
            return
        cmd = msg.data.strip()
        msg = self._command_to_msg(cmd, self.base_speed)
        if msg:
            self.cmd_pub.publish(msg)

    def _pick_cb(self, msg: String):
        data = msg.data.strip()
        self.autoMode = data == "1"
        self.auto_pub.publish(Bool(data=self.autoMode))

        if self.auto_client.service_is_ready():
            req = SetBool.Request()
            req.data = self.autoMode
            self.auto_client.call_async(req)

    def _command_to_msg(self, command, speed):
        command = command.strip()
        if not command:
            return None

        if command == "Stop":
            return String(data="Stop:0")

        speed = int(round(speed))
        if speed < 0:
            speed = 0

        if command in ("Forward", "Backward", "Left", "Right", "RotateLeft", "RotateRight"):
            return String(data=f"{command}:{speed}")
        return None



def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
