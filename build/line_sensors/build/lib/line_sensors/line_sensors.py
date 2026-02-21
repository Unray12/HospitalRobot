#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import time
import json


class LineSensors(Node):
    
    def __init__(self):
        super().__init__("line_sensors")
        self.lineSensorBuffer = ""

        try:
            self.serLineSensors = serial.Serial(
                port='/dev/ttyUSB1',   # Pi: /dev/serial0 | USB: /dev/ttyUSB1
                baudrate=115200,
                timeout=0.1
            )
            time.sleep(2)
            self.get_logger().info("Serial Line Sensors connected")
            
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            self.serLineSensors = None

def main(args=None):
    rclpy.init(args=args)
    node = LineSensors()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()