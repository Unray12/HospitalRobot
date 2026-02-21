#!/usr/bin/env python3
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
    'Forward': (-1, -1, -1, -1),   # Forward
    'Backward': (+1, +1, +1, +1),   # Backward
    'Right': (-1, +1, -1, +1),   # Right
    'Left': (+1, -1, +1, -1),   # Left
    'RotateRight': (-1, -1, +1, +1),   # Forward Right
    'RotateLeft': ( +1, +1, -1, -1),   # Forward Left
    'Stop': (0, 0, 0, 0)    # Stop
}


class Robot(Node):
    
    # States for line following
    STATE_FOLLOWING = 0      # Bám line bình thường
    STATE_CROSSING = 1       # Vừa gặp line ngang, tiến lên
    STATE_TURNING = 2        # Đang xoay tìm line phải
    
    def robotMove(self, direction, speed: float = 0):
        # direction = direction.data
        if direction in DIR:
            wheel_speeds = [s * speed for s in DIR[direction]]
            cmd = ",".join(f"{v}" for v in wheel_speeds) + "\n"
            if self.ser and self.ser.is_open:
                self.ser.write(f"({cmd})".encode())
            print(f"Moving {direction} with speeds: ({cmd}) Automode: {self.autoMode}")
        else:
            print("Invalid direction command")
    
    def __init__(self):
        super().__init__("twist_subscriber")
        self.lineSensorBuffer = ""
        self.baseSpeed = 4
        self.autoMode = False
        self.ser = None
        self.serLineSensors = None
        
        # State machine variables
        self.lineState = self.STATE_FOLLOWING
        self.crossingStartTime = None
        self.crossingDuration = 2.0  # 2 seconds
        self.lastDirection = "Stop"
        
        self.timer = self.create_timer(0.01 , self.processGoLine)



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
        if not (self.serLineSensors and self.serLineSensors.is_open):
            return None

        try:
            # Đọc tất cả dữ liệu đang có trong buffer
            data = self.serLineSensors.read(self.serLineSensors.in_waiting or 1)
            if not data:
                return None

            # Ghép vào buffer tổng
            self.lineSensorBuffer += data.decode(errors="ignore")

            # Nếu có newline -> tách 1 frame hoàn chỉnh
            if "\n" in self.lineSensorBuffer:
                line, self.lineSensorBuffer = self.lineSensorBuffer.split("\n", 1)
                line = line.strip()

                if line:
                    self.get_logger().info(f"Line Sensors: {line}")
                    return line

        except Exception as e:
            self.get_logger().error(f"Line Sensors read error: {e}")

        return None

       
    def count_active(self, sensor_dict):
        count = sum(sensor_dict.values())
        return count

    def is_full_black(self, sensor_dict):
        return all(v == 1 for v in sensor_dict.values())

    def processGoLine(self):
        """Process line following with state machine.
        Called every 10ms by timer - no blocking loops!
        """
        if not self.autoMode:
            return

        raw_data = self.getLineSensors()
        if not raw_data:
            return

        # Parse JSON
        try:
            data = json.loads(raw_data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Corrupted JSON skipped: {raw_data}")
            return

        if "LineSensor" not in data:
            return

        lineSensorsData = data["LineSensor"]
        if not lineSensorsData:
            return

        # Get sensor data
        left = lineSensorsData.get("0x25", {})
        middle = lineSensorsData.get("0x24", {})
        right = lineSensorsData.get("0x23", {})

        left_count = self.count_active(left)
        mid_count = self.count_active(middle)
        right_count = self.count_active(right)
        total_black = left_count + mid_count + right_count

        direction = None
        speed = self.baseSpeed

        # ================================================
        # STATE MACHINE FOR LINE FOLLOWING
        # ================================================

        # STATE 1: DETECTING HORIZONTAL LINE
        if self.is_full_black(left) and \
           self.is_full_black(middle) and \
           self.is_full_black(right):
            
            # Start crossing state
            self.lineState = self.STATE_CROSSING
            self.crossingStartTime = time.time()
            self.get_logger().info("===> Horizontal line detected! Starting crossing sequence...")
            direction = "Forward"
            speed = self.baseSpeed  # Tiếp tục tiến 2s

        # STATE 2: CROSSING HORIZONTAL LINE (advance for 2 seconds)
        elif self.lineState == self.STATE_CROSSING:
            elapsed = time.time() - self.crossingStartTime
            
            if elapsed < self.crossingDuration:
                # Keep moving forward
                direction = "Forward"
                speed = self.baseSpeed
                self.get_logger().info(f"Crossing... {elapsed:.2f}s / {self.crossingDuration}s")
            else:
                # 2 seconds done - now turn right to find right line
                self.lineState = self.STATE_TURNING
                self.get_logger().info("Crossing complete! Starting right turn...")
                direction = "RotateRight"
                speed = self.baseSpeed

        # STATE 3: TURNING TO FIND RIGHT LINE
        elif self.lineState == self.STATE_TURNING:
            # Keep turning right until we detect right sensor line
            if right_count >= 2:  # Found right line
                self.lineState = self.STATE_FOLLOWING
                self.get_logger().info("Right line found! Returning to normal following...")
                direction = "Forward"
                speed = self.baseSpeed
            else:
                # Still looking for right line
                direction = "RotateRight"
                speed = self.baseSpeed
                self.get_logger().debug(f"Turning right... right_count={right_count}")

        # STATE 4: NORMAL LINE FOLLOWING
        elif self.lineState == self.STATE_FOLLOWING:
            # ===============================
            # 1️⃣ Đi thẳng - middle line clear
            # ===============================
            if self.is_full_black(middle) and left_count <= 1 and right_count <= 1:
                direction = "Forward"

            # ===============================
            # 2️⃣ Lệch trái
            # ===============================
            elif left_count > right_count:
                direction = "RotateLeft"

            # ===============================
            # 3️⃣ Lệch phải
            # ===============================
            elif right_count > left_count:
                direction = "RotateRight"

            # ===============================
            # 4️⃣ Mất line giữa -> xoay tìm
            # ===============================
            elif mid_count == 0:
                if left_count > 0:
                    direction = "RotateLeft"
                elif right_count > 0:
                    direction = "RotateRight"
                else:
                    direction = "Stop"
                    self.get_logger().info("===> LOST LINE COMPLETELY")

            # ===============================
            # 5️⃣ Default: di thẳng
            # ===============================
            else:
                direction = "Forward"

        # ===============================
        # SEND COMMAND
        # ===============================
        if direction:
            self.robotMove(direction, speed)
            self.lastDirection = direction
    

    def robotManual_callback(self, msg: String):
        print("robotManual_callback called")
        if self.autoMode:
            pass
        else:
            self.robotMove(msg.data, speed=4)

    def PickRobot_callback(self, msg: String):
        if msg.data == "1":
            self.autoMode = True
            # self.processGoLine()
            self.get_logger().info("Auto Mode Enabled")
        else:
            self.autoMode = False
            self.get_logger().info("Auto Mode Disabled")
        
    def destroy_node(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
        super().destroy_node()



def main(args=None):
    rclpy.init(args=args)
    node = Robot()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()