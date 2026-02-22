import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

from .line_sensor_reader import LineSensorReader


class LineSensorDriverNode(Node):
    def __init__(self):
        super().__init__("line_sensor_driver")

        self.reader = LineSensorReader(
            port="/dev/ttyACM0",
            baudrate=115200,
            timeout=0.1,
            logger=self.get_logger(),
        )

        self.pub = self.create_publisher(Int16MultiArray, "/line_sensors/frame", 10)
        self.timer = self.create_timer(0.01, self._timer_cb)

    def _timer_cb(self):
        frame = self.reader.read_frame()
        if frame is None:
            return

        msg = Int16MultiArray()
        msg.data = [
            int(frame["left_count"]),
            int(frame["mid_count"]),
            int(frame["right_count"]),
            int(frame["left_full"]),
            int(frame["mid_full"]),
            int(frame["right_full"]),
        ]
        self.pub.publish(msg)

    def destroy_node(self):
        self.reader.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineSensorDriverNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
