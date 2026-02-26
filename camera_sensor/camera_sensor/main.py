#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import threading
import time
from robot_common.config_manager import ConfigManager
from robot_common.logging_utils import LogAdapter


class SerialReader(Node):
    def __init__(self):
        super().__init__('camera_sensor')
        self.log = LogAdapter(self.get_logger(), "camera_sensor")

        config = ConfigManager("camera_sensor", logger=self.log).load()
        serial_cfg = config.get("serial", {})
        pub_cfg = config.get("publish", {})

        self.port = str(serial_cfg.get("port", "/dev/ttyACM0"))
        self.baud = int(serial_cfg.get("baudrate", 115200))
        self.topic = str(pub_cfg.get("topic", "/face/camera"))
        self.read_timeout = float(serial_cfg.get("timeout", 0.2))

        self.pub = self.create_publisher(String, self.topic, 10)

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._read_loop, daemon=True)

        self.ser = None
        self._open_serial()

        self._thread.start()
        self.log.info(
            f"Reading serial from {self.port} @ {self.baud} -> topic '{self.topic}'",
            event="BOOT",
        )

    def _open_serial(self):
        try:
            # exclusive=True giúp tránh 2 process mở cùng lúc (pyserial>=3.5)
            self.ser = serial.Serial(
                port=self.port,
                baudrate=int(self.baud),
                timeout=self.read_timeout,
                exclusive=True
            )
            # Optional: flush
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            # self._log_info(f"Serial Line Sensors connected: {self.port}")
        except Exception as e:
            self.log.error(f"Cannot open serial {self.port}: {e}", event="SERIAL")
            raise

    def _read_loop(self):
        while rclpy.ok() and not self._stop.is_set():
            try:
                if self.ser is None or not self.ser.is_open:
                    time.sleep(0.2)
                    continue

                line = self.ser.readline()  # bytes, ends with \n if present
                if not line:
                    continue

                text = line.decode('utf-8', errors='replace').strip()
                if text == '':
                    continue

                msg = String()
                msg.data = text
                # self._log_info(text)
                self.pub.publish(msg)

            except serial.SerialException as e:
                self.log.error(f"SerialException: {e}", event="SERIAL")
                time.sleep(0.5)
            except Exception as e:
                self.log.error(f"Read error: {e}", event="SERIAL")
                time.sleep(0.2)

    def destroy_node(self):
        self._stop.set()
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = SerialReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
