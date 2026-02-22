import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist


class ManualControlNode(Node):
    def __init__(self):
        super().__init__("manual_control")

        self.base_speed = 6
        self.autoMode = False

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.auto_pub = self.create_publisher(Bool, "/auto_mode", 10)
        self.auto_client = self.create_client(SetBool, "/set_auto_mode")

        self.create_subscription(String, "/VR_control", self._manual_cb, 10)
        self.create_subscription(String, "/pick_robot", self._pick_cb, 10)

    def _manual_cb(self, msg: String):
        if self.autoMode:
            return
        cmd = msg.data.strip()
        twist = self._command_to_twist(cmd, self.base_speed)
        self.cmd_pub.publish(twist)

    def _pick_cb(self, msg: String):
        data = msg.data.strip()
        self.autoMode = data == "1"
        self.auto_pub.publish(Bool(data=self.autoMode))

        if self.auto_client.service_is_ready():
            req = SetBool.Request()
            req.data = self.autoMode
            self.auto_client.call_async(req)

    def _command_to_twist(self, command, speed):
        msg = Twist()
        if command == "Forward":
            msg.linear.x = speed
        elif command == "Backward":
            msg.linear.x = -speed
        elif command == "Left":
            msg.linear.y = speed
        elif command == "Right":
            msg.linear.y = -speed
        elif command == "RotateLeft":
            msg.angular.z = speed
        elif command == "RotateRight":
            msg.angular.z = -speed
        elif command == "Stop":
            pass
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ManualControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
