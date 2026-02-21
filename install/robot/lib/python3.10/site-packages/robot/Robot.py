import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import serial
import time
import json

# Direction map for mecanum wheels
# Order: [FL, FR, RL, RR]
DIR = {
    'Forward': (+1, +1, +1, +1),   # Forward
    'Backward': (-1, -1, -1, -1),   # Backward
    'Right': (+1, -1, +1, -1),   # Right
    'Left': (-1, +1, -1, +1),   # Left
    'RotateRight': (+1, -1, -1, +1),   # Forward Right
    'RotateLeft': ( -1, +1, +1, -1),   # Forward Left
    'Stop': (0, 0, 0, 0)    # Stop
}

class Robot(Node):
    
    self.serLineSensors = None
    self.ser = None
    self.autoMode = False
    
    def robotMove(self, direction: String, speed: float):
        direction = direction.data
        if direction in DIR:
            wheel_speeds = [s * speed for s in DIR[direction]]
            cmd = ",".join(f"{v}" for v in wheel_speeds) + "\n"
            if self.ser and self.ser.is_open:
                self.ser.write(f"({cmd})".encode())
            print(f"Moving {direction} with speeds: ({cmd})")
        else:
            print("Invalid direction command")
    
    def __init__(self):
        super().__init__("twist_subscriber")

        try:
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',   # Pi: /dev/serial0 | USB: /dev/ttyUSB0
                baudrate=115200,
                timeout=0.1
            )
            time.sleep(2)
            self.get_logger().info("Serial CAN connected")
            
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            self.ser = None
        
        try:
            self.serLineSensors = serial.Serial(
                port='/dev/ttyACM0',   # Pi: /dev/serial0 | USB: /dev/ttyUSB0
                baudrate=115200,
                timeout=0.1
            )
            time.sleep(2)
            self.get_logger().info("Serial Line Sensors connected")
            
        except Exception as e:
            self.get_logger().error(f"Serial error: {e}")
            self.serLineSensors = None

        # ROS2 subscriber
        self.RobotManualSubscriber_ = self.create_subscription(
            String,
            "/VR_control",
            self.robotManual_callback,
            10
        )
        
        self.PickRobotSubscriber_ = self.create_subscription(
            String,
            "/pick_robot",
            self.PickRobot_callback,
            10
        )


    def getLineSensors(self):
        if self.serLineSensors and self.serLineSensors.is_open:
            try:
                if self.serLineSensors.in_waiting > 0:
                    lineSensorsJson = self.serLineSensors.readline().decode(errors='ignore').strip()
                    if lineSensorsJson:
                        self.get_logger().info(f"Line Sensors: {lineSensorsJson}")
                        return lineSensorsJson
            except Exception as e:
                self.get_logger().error(f"Line Sensors read error: {e}")
        return None
    
    def processLineSensors(self):
        lineSensorsData = json.loads(self.getLineSensors())["LineSensor"]
        if lineSensorsData:
            all_one = all(
            value == 0
            for device in lineSensorsData.values()
            for value in device.values()
        )
            if all_one:
                self.get_logger().info("Line Detected: Stopping Robot")
                self.robotMove(String(data="Stop"), speed=0)
            else:
                self.get_logger().info("Line Not Detected: Continuing")
                self.robotMove(String(data="Forward"), speed=4)
                
        
    def robotManual_callback(self, msg: String):
        print("robotManual_callback called")
        if self.autoMode:
            pass
        else:
            self.robotMove(msg, speed=4)

    def PickRobot_callback(self, msg: String):
        if msg.data == "1":
            self.autoMode = True
            self.processLineSensors()
            self.get_logger().info("Auto Mode Enabled")
        else:
            self.autoMode = False
            self.get_logger().info("Auto Mode Disabled")
        
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()
        