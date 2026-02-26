#import "@preview/boxed-sheet:0.1.2": *

= Package `robot`
#concept-block[
  #inline("Role")
  - Entry package cho vận hành hệ thống bằng một lệnh duy nhất.
  - Đóng vai trò orchestrator khởi tạo các node runtime theo cấu hình.

  #inline("Runtime entry")
  - Executable: `robot`
  - Run:
    `ros2 run robot robot`
  - Main file:
    `robot/robot/main.py`

  #inline("Startup sequence")
  - Load config section `robot` bằng `ConfigManager("robot")`.
  - Đọc `bringup.nodes`.
  - Map tên node -> `(package, executable)`.
  - Tạo `launch_ros.actions.Node` cho từng node hợp lệ.
  - Chạy `LaunchService`.

  #inline("Configuration (current defaults)")
  - Section:
    `robot_common/robot_common/config.json -> robot`
  - `bringup.nodes` mặc định:
    `line_sensor_driver`, `line_follower`, `motor_driver`, `manual_control`, `camera_sensor`

  #inline("Default bringup nodes")
  - `line_sensor_driver`
  - `line_follower`
  - `motor_driver`
  - `manual_control`
  - `camera_sensor`

  #inline("Customization")
  - Sửa danh sách node tại:
    `robot_common/robot_common/config.json`
  - Path config:
    `robot.bringup.nodes`

  #inline("Behavior on invalid config")
  - Node name lạ (không nằm trong map) sẽ bị skip.
  - Hệ thống in warning:
    `Unknown node '<name>' - skipped`.

  #inline("Validation commands")
  ```bash
  ros2 run robot robot
  ros2 node list
  ros2 topic list
  ```

  #inline("Troubleshooting")
  - Bringup thiếu node:
    kiểm tra `bringup.nodes` và tên executable thực tế.
  - Node chết ngay sau launch:
    chạy từng package riêng để khoanh vùng lỗi cục bộ.
]
