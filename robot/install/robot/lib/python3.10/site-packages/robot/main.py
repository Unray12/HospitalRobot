#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import serial
import time


class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("twist_subscriber")

        # ROS2 subscriber
        self.pose_subscriber_ = self.create_subscription(
            Twist,
            "/VR_control",
            self.Driver_callback,
            10
        )

        # Serial init
        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',   # Pi: /dev/serial0 | USB: /dev/ttyUSB0
                baudrate=115200,
                timeout=0.1
            )
            time.sleep(2)
            self.get_logger().info("Serial connected")
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser = None

    def Driver_callback(self, msg: Twist):
        linear_x = msg.linear.x
        linear_y = msg.linear.y
        angular_z = msg.angular.z
        
        # print(msg)
        # Log (debug)
        self.get_logger().info(
            f"linear.x = {linear_x:.3f},linear.y = {linear_y:.3f}, angular.z = {angular_z:.3f}"
        )

        # Send to robot
        if self.ser and self.ser.is_open:
            # Format: <lin,ang>
            data = f"({linear_x:.3f},{linear_y:.3f},{angular_z:.3f})\n"
            self.ser.write(data.encode())

    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()