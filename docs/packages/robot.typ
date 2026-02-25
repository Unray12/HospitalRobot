#import "@preview/boxed-sheet:0.1.2": *

= Package `robot`
#concept-block[
  #inline("Role")
  - Gói bringup để launch nhóm node runtime bằng một lệnh.
  - Dùng cho chế độ vận hành tiêu chuẩn của robot.

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

  #inline("Default bringup nodes")
  - `line_sensor_driver`
  - `line_follower`
  - `motor_driver`
  - `manual_control`

  #inline("Customization")
  - Sửa danh sách node tại:
    `robot_common/robot_common/config.json`
  - Path config:
    `robot.bringup.nodes`

  #inline("Behavior on invalid config")
  - Node name lạ (không nằm trong map) sẽ bị skip.
  - Hệ thống in warning:
    `Unknown node '<name>' - skipped`.

  #inline("Troubleshooting checklist")
  - Bringup thiếu node:
    kiểm tra `bringup.nodes` và tên executable thực tế.
  - Node chết ngay sau launch:
    chạy từng package riêng để khoanh vùng lỗi cục bộ.
]
