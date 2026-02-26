#import "@preview/boxed-sheet:0.1.2": *

= Package `robot_common`
#concept-block[
  #inline("Role")
  - Nền tảng dùng chung cho toàn hệ thống ROS2 trong project.
  - Quản lý config tập trung, chuẩn command text, chuẩn logging, và plan JSON.

  #inline("Core files")
  - `robot_common/robot_common/config.json`
  - `robot_common/robot_common/config_manager.py`
  - `robot_common/robot_common/command_protocol.py`
  - `robot_common/robot_common/logging_utils.py`
  - `robot_common/robot_common/plans/*.json`

  #inline("ConfigManager API")
  - `load(defaults=None, force=False)`:
    đọc `config.json`, trả section theo package và merge default nếu có.
  - `load_plan(plan_name)`:
    đọc `plans/<plan_name>.json`, cache kết quả theo tên plan.
  - Fallback:
    nếu lỗi đọc file thì warning và trả config rỗng để node vẫn khởi động được.

  #inline("Command protocol")
  - Parse text `Direction:Speed` thành tuple `(direction, speed)`.
  - Hỗ trợ separator `:`, `,`, hoặc khoảng trắng.
  - Direction hợp lệ:
    `Forward`, `Backward`, `Left`, `Right`, `RotateLeft`, `RotateRight`, `Stop`.
  - Speed âm sẽ bị clamp về `0`.

  #inline("Logging contract")
  - Format chuẩn:
    `[component] [event] message | key=value`.
  - Màu theo level:
    `INFO` (cyan), `WARNING` (yellow), `ERROR` (red).
  - Tắt màu:
    `NO_COLOR=1` hoặc `ROBOT_LOG_COLOR=0`.

  #inline("Plans currently packaged")
  - `a20`
  - `a21`
  - `a22`
  - `a23`
  - `a24`
  - `a25`
  - Tham khảo ví dụ tổng hợp: `docs/plan_examples.json`

  #inline("Troubleshooting")
  - `Plan not found`:
    kiểm tra tên gửi vào `/plan_select` có khớp tên file plan không.
  - Config không load:
    kiểm tra `package_data` trong `robot_common/setup.py`.
  - Alias plan sai:
    kiểm tra đồng bộ giữa `line_follower.plan_alias` và `mqtt_bridge.plan_keys`.
]
