import serial
import time



ser = serial.Serial(
    port='/dev/ttyACM0',   # Pi: /dev/serial0 | USB: /dev/ttyUSB0
    baudrate=115200,
    timeout=0.1
)
time.sleep(2)
    


while True:
    if ser and ser.is_open and ser.in_waiting > 0:
        # Format: <lin,ang>
        data = ser.readline().decode().strip()
        print(f"Received: {data}")
