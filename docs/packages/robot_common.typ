#import "@preview/boxed-sheet:0.1.2": *

= Package `robot_common`
#concept-block[
  #inline("Role")
  - Cung cấp nền tảng chia sẻ cho toàn bộ workspace.
  - Đảm nhận 4 trách nhiệm chính:
    quản lý cấu hình runtime, chuẩn giao thức command text,
    chuẩn hoá logging có metadata, và loader cho file plan.

  #inline("Core files")
  - `robot_common/robot_common/config.json`
  - `robot_common/robot_common/config_manager.py`
  - `robot_common/robot_common/command_protocol.py`
  - `robot_common/robot_common/logging_utils.py`
  - `robot_common/robot_common/plans/*.json`

  #inline("Public API - ConfigManager")
  - `load(defaults=None, force=False)`:
    đọc `config.json`, lấy đúng section theo tên package,
    hỗ trợ deep-merge `defaults` vào config đã load.
  - `load_plan(plan_name, force=False)`:
    đọc `plans/<plan_name>.json` theo tên plan, cache theo từng plan.
  - Cache policy:
    cache global cho config file và plan file trong process hiện tại.
  - Error policy:
    lỗi IO/JSON không làm crash node, chỉ warning và fallback rỗng.

  #inline("Public API - Command protocol")
  - `parse_command(text)`:
    parse `Direction:Speed` (chấp nhận `:`, `,`, hoặc khoảng trắng).
  - `format_command(direction, speed=0)`:
    chuẩn hoá command string để publish.
  - Direction whitelist:
    `Forward`, `Backward`, `Left`, `Right`, `RotateLeft`, `RotateRight`, `Stop`.
  - Validation:
    direction sai trả `(None, None)` hoặc `None`.
  - Safety:
    speed âm bị clamp về `0`; `Stop` luôn được chuẩn hoá thành `Stop:0`.

  #inline("Public API - Logging contract")
  - Adapter:
    `LogAdapter(logger, component, use_color=None)`
  - Message format:
    `[component] [event] message | key=value`.
  - Level color mapping:
    `INFO` (cyan), `WARNING` (yellow), `ERROR` (red).
  - Environment flags:
    `NO_COLOR=1` hoặc `ROBOT_LOG_COLOR=0`.
  - Mục tiêu:
    log có thể đọc bởi người vận hành và đồng thời parse được bởi tool.

  #inline("Plan repository hiện có")
  - `a15`, `a16`, `a17`, `a18`, `a19`, `a20`
  - `plan_demo_autoline_callback`
  - `plan_sample_huong_dan`
  - Legacy plan templates:
  - `plan_straight`
  - `plan_turn_right`
  - `plan_u_turn`
  - `plan_stop`

  #inline("Validation commands")
  ```bash
  ros2 pkg prefix robot_common
  python -c "from robot_common.config_manager import ConfigManager as C; print(C('line_follower').load().get('base_speed'))"
  python -c "from robot_common.command_protocol import parse_command; print(parse_command('RotateLeft:6'))"
  ```

  #inline("Troubleshooting")
  - `Plan not found`:
    kiểm tra tên gửi vào `/plan_select` có khớp tên file plan không.
  - Config không load:
    kiểm tra `package_data` trong `robot_common/setup.py`.
  - Alias plan sai:
    kiểm tra đồng bộ giữa `line_follower.plan_alias`, `mqtt_bridge.plan_keys`, `mqtt_bridge.room_plans`.
]
