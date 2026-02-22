#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

import paho.mqtt.client as mqtt
import sys
import termios
import tty
import threading

# MQTT settings
BROKER_ADDRESS = "127.0.0.1"
MQTT_PORT = 1883
TOPICS = ['VR_control', 'pick_robot']


# Hàm đọc phím 1 ký tự
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


class MQTTBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_ros2')

        # ROS 2 publisher
        self.ros_pub = self.create_publisher(String, 'VR_control', 10)
        self.ros_pick_pub = self.create_publisher(String, 'pick_robot', 10)

        # MQTT client
        self.client = mqtt.Client()
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message

        self.client.connect(BROKER_ADDRESS, MQTT_PORT, 60)

        # MQTT chạy ở thread riêng
        mqtt_thread = threading.Thread(target=self.client.loop_forever, daemon=True)
        mqtt_thread.start()

        self.get_logger().info("MQTT ↔ ROS 2 bridge started")
        self.keyboard_loop()

    # MQTT callbacks
    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info("Connected to MQTT broker")
            for topic in TOPICS:
                client.subscribe(topic)
        else:
            self.get_logger().error(f"MQTT connect failed: {rc}")

    def on_message(self, client, userdata, msg):
        data = msg.payload.decode()
        self.get_logger().info(f"MQTT → ROS2: {data}")

        ros_msg = String()
        ros_msg.data = data
        if msg.topic == TOPICS[0]:
            self.ros_pub.publish(ros_msg)
        elif msg.topic == TOPICS[1]:
            self.ros_pick_pub.publish(ros_msg)

    # Keyboard → MQTT
    def keyboard_loop(self):
        self.get_logger().info("Điều khiển WASD | q để thoát")
        autoMode = False
        while rclpy.ok():
            key = get_key().lower()
            cmd = ""

            if key == "w":
                cmd = "Forward"
            elif key == "s":
                cmd = "Backward"
            elif key == "a":
                cmd = "Left"
            elif key == "d":
                cmd = "Right"
            elif key == " ":
                cmd = "Stop"
            elif key == "j":
                cmd = "RotateLeft"
            elif key == "p":
                cmd = "RotateRight"
            elif key == "k":
                autoMode = not autoMode
                mode_msg = "1" if autoMode else "0"
                self.client.publish(TOPICS[1], mode_msg)
                self.get_logger().info(f"ROS2 → MQTT: {mode_msg}")
            elif key == "q":
                self.get_logger().info("Thoát chương trình")
                break

            if cmd:
                self.client.publish(TOPICS[0], cmd)
                self.get_logger().info(f"ROS2 → MQTT: {cmd}")

        self.client.disconnect()


def main(args=None):
    rclpy.init(args=args)
    node = MQTTBridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
