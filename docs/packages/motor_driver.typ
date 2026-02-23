#import "@preview/boxed-sheet:0.1.2": *

= Package `motor_driver`
#concept-block[
  #inline("Role")
  - Chuyển command hướng thành tốc độ 4 bánh mecanum.
  - Gửi packet serial xuống board điều khiển motor.

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

  #inline("Direction mapping")
  - `Forward`: `(-1,-1,-1,-1)`
  - `Backward`: `(1,1,1,1)`
  - `Right`: `(-1,1,-1,1)`
  - `Left`: `(1,-1,1,-1)`
  - `RotateRight`: `(-1,-1,1,1)`
  - `RotateLeft`: `(1,1,-1,-1)`
  - `Stop`: `(0,0,0,0)`

  #inline("Config defaults (current)")
  - Section:
    `robot_common/robot_common/config.json -> motor_driver`
  - Serial:
    `port=/dev/ttyUSB0`, `baudrate=115200`, `timeout=0.1`
  - Command topic:
    `/motor_cmd`
  - Debug toggle topic:
    `/debug_logs_toggle`

  #inline("Operational logs")
  - `DEBUG_TOGGLE`: trạng thái debug log ON/OFF.
  - `MOTOR`: log wheel speed `fl/fr/rl/rr` khi debug bật.

  #inline("Troubleshooting checklist")
  - Có `/motor_cmd` nhưng motor không chạy:
    kiểm tra serial port, quyền truy cập cổng, và nguồn board.
  - Command bị bỏ qua:
    kiểm tra direction có nằm trong whitelist protocol.
  - Lệnh đúng nhưng robot đi sai hướng:
    kiểm tra mapping bánh thực tế và dấu tốc độ theo chassis.
]
