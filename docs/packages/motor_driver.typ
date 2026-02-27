#import "@preview/boxed-sheet:0.1.2": *

= Package `motor_driver`
#concept-block[
  #inline("Role")
  - Tầng actuation bridge giữa ROS2 command và controller phần cứng.
  - Nhận command text mức cao, chuyển sang 4 bánh mecanum và đẩy xuống serial.

  #inline("Runtime node")
  - Executable: `motor_driver`
  - Run:
    `ros2 run motor_driver motor_driver`
  - Main files:
    `motor_driver/motor_driver_node.py`,
    `motor_driver/motor_controller.py`

  #inline("I/O contract")
  - Subscribe:
    `/motor_cmd` (`std_msgs/String`), `/debug_logs_toggle` (`std_msgs/Bool`)
  - Command input format:
    `Direction:Speed`
  - Serial output format:
    `(fl,fr,rl,rr)\n`
  - Value policy:
    wheel speed = `direction_sign * speed`.

  #inline("Direction mapping")
  - `Forward`: `(-1,-1,-1,-1)`
  - `Backward`: `(1,1,1,1)`
  - `Right`: `(-1,1,-1,1)`
  - `Left`: `(1,-1,1,-1)`
  - `RotateRight`: `(-1,-1,1,1)`
  - `RotateLeft`: `(1,1,-1,-1)`
  - `Stop`: `(0,0,0,0)`

  #inline("Configuration (current defaults)")
  - Section:
    `robot_common/robot_common/config.json -> motor_driver`
  - Serial:
    `port=/dev/ttyUSB1`, `baudrate=115200`, `timeout=0.1`
  - Reconnect:
    `reconnect_period_sec=2.0`,
    fallback `[/dev/ttyUSB1, /dev/ttyUSB0]`,
    `scan_prefixes=[/dev/ttyUSB, /dev/ttyACM, COM]`
  - Command topic:
    `/motor_cmd`
  - Debug toggle topic:
    `/debug_logs_toggle`

  #inline("Runtime behavior")
  - Parse command bằng `robot_common.command_protocol.parse_command`.
  - Command invalid bị bỏ qua, không phát serial.
  - Khi serial write lỗi:
    đóng port và chờ reconnect timer tự phục hồi.
  - Reconnect strategy:
    thử port chính -> fallback ports -> scan ports theo prefix.

  #inline("Observability")
  - `DEBUG_TOGGLE`: trạng thái debug log ON/OFF.
  - `MOTOR`: log wheel speed `fl/fr/rl/rr` khi debug bật.
  - `SERIAL`: thông báo reconnect thành công.

  #inline("Validation commands")
  ```bash
  ros2 topic pub --once /motor_cmd std_msgs/String "{data: 'Forward:8'}"
  ros2 topic pub --once /debug_logs_toggle std_msgs/Bool "{data: true}"
  ros2 topic pub --once /motor_cmd std_msgs/String "{data: 'RotateLeft:6'}"
  ```

  #inline("Troubleshooting")
  - Có `/motor_cmd` nhưng motor không chạy:
    kiểm tra serial port, quyền truy cập cổng, và nguồn board.
  - Command bị bỏ qua:
    kiểm tra direction có nằm trong whitelist protocol.
  - Lệnh đúng nhưng robot đi sai hướng:
    kiểm tra mapping bánh thực tế và dấu tốc độ theo chassis.
]
