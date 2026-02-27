#import "@preview/boxed-sheet:0.1.2": *

= Package `camera_sensor`
#concept-block[
  #inline("Role")
  - Thu thập dữ liệu nhận diện từ cổng serial camera/MCU.
  - Chuẩn hoá payload lỗi/biến thể về format thống nhất cho hệ thống downstream.
  - Publish dữ liệu đã chuẩn hóa để bridge sang MQTT/UI.

  #inline("Runtime node")
  - Executable: `camera_sensor`
  - Run:
    `ros2 run camera_sensor camera_sensor`
  - Main file:
    `camera_sensor/camera_sensor/main.py`

  #inline("I/O contract")
  - Publish:
    `/face/camera` (`std_msgs/String`)
  - Output format chuẩn:
    `<DEV1,FACE,0|1>` hoặc `<DEV1,NO_OBJECT>`
  - Input format chấp nhận:
    chuỗi biến thể có/không có dấu `< >`, có ký tự nhiễu, có lỗi chính tả nhẹ.

  #inline("Configuration (current defaults)")
  - Section:
    `robot_common/robot_common/config.json -> camera_sensor`
  - Serial:
    `port=/dev/ttyACM0`, `baudrate=115200`, `timeout=0.2`
  - Reconnect:
    `reconnect_period_sec=2.0`
  - Topic:
    `/face/camera`

  #inline("Runtime behavior")
  - Node mở serial với `exclusive=True` (nếu pyserial hỗ trợ) để tránh nhiều process mở chung cổng.
  - Timer callback đọc `readline()` theo chu kỳ `publish.rate_hz`, rồi normalize từng frame.
  - Frame malformed bị drop, có đếm số lượng drop để log định kỳ.
  - Khi lỗi serial:
    đóng cổng và reconnect bằng timer định kỳ.

  #inline("Normalization behavior")
  - Làm sạch text:
    uppercase, bỏ khoảng trắng/ký tự lạ, xử lý frame có dấu `<...>`.
  - Chuẩn hoá device token:
    `DEV`, `DV1`, `EV1`, `D1`, ... -> `DEV1`.
  - Chuẩn hoá state token:
    `FACE` hoặc biến thể lỗi chính tả -> `FACE`,
    `NO_OBJECT` và biến thể -> `NO_OBJECT`.
  - Nếu payload lỗi định dạng:
    drop frame và log warning định kỳ.

  #inline("Validation commands")
  ```bash
  ros2 topic echo /face/camera
  ros2 topic hz /face/camera
  ```

  #inline("Troubleshooting")
  - Không có dữ liệu:
    kiểm tra serial port đang bị process khác giữ.
  - Nhiều frame bị drop:
    kiểm tra format firmware gửi lên có theo `DEV,STATE[,score]`.
  - MQTT không thấy `face/camera`:
    kiểm tra `mqtt_bridge` đang subscribe ROS `/face/camera`.
]
