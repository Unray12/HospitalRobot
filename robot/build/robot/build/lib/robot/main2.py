#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import time


class PoseSubscriberNode(Node):
    def __init__(self):
        super().__init__("string_subscriber")

        self.pose_subscriber_ = self.create_subscription(
            String,
            "/VR_control",
            self.Driver_callback,
            10
        )

        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                timeout=0.1
            )
            time.sleep(2)
            self.get_logger().info("Serial connected")
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser = None

    def Driver_callback(self, msg: String):
        raw = msg.data.strip()
        self.get_logger().info(f"RX: {raw}")

        try:
            raw = raw.strip("()")
            parts = raw.split(",")

            data = {}
            for part in parts:
                k, v = part.split(":")
                data[k.lower()] = float(v)

            s1 = data.get("s1", 0.0)
            s2 = data.get("s2", 0.0)
            s3 = data.get("s3", 0.0)
            s4 = data.get("s4", 0.0)

            if self.ser and self.ser.is_open:
                out = f"({s1:.2f},{s2:.2f},{s3:.2f},{s4:.2f})\n"
                self.ser.write(out.encode())

        except Exception as e:
            self.get_logger().error(f"Parse error: {e}")

    def read_serial(self):
        if self.ser and self.ser.is_open:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode(errors='ignore').strip()
                    if line:
                        self.get_logger().info(f"STM32 → ROS2: {line}")

                        if line == "receiveok":
                            self.get_logger().info("✅ Receive OK confirmed")
            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")


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

