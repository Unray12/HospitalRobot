import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, Bool
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist

from .line_follower import LineFollowerFSM


class LineFollowerNode(Node):
    def __init__(self):
        super().__init__("line_follower")

        self.base_speed = 6
        self.autoMode = False
        self._last_frame = None

        self.follower = LineFollowerFSM(
            base_speed=self.base_speed,
            crossing_duration=2.0,
            pid_kp=6.0,
            pid_ki=0.0,
            pid_kd=0.0,
            pid_deadband=0.15,
            min_turn_speed=2.0,
            max_turn_speed=self.base_speed,
            logger=self.get_logger(),
        )

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.create_subscription(Int16MultiArray, "/line_sensors/frame", self._frame_cb, 10)
        self.create_subscription(Bool, "/auto_mode", self._auto_cb, 10)
        self.create_service(SetBool, "/set_auto_mode", self._auto_srv_cb)

        self.timer = self.create_timer(0.01, self._timer_cb)

    def _frame_cb(self, msg: Int16MultiArray):
        if len(msg.data) < 6:
            self.get_logger().warn("Line frame invalid length")
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
        self.cmd_pub.publish(self._command_to_twist(direction, speed))

    def _command_to_twist(self, direction, speed):
        msg = Twist()
        if direction == "Forward":
            msg.linear.x = speed
        elif direction == "Backward":
            msg.linear.x = -speed
        elif direction == "Left":
            msg.linear.y = speed
        elif direction == "Right":
            msg.linear.y = -speed
        elif direction == "RotateLeft":
            msg.angular.z = speed
        elif direction == "RotateRight":
            msg.angular.z = -speed
        return msg

    def _publish_stop(self):
        self.cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
