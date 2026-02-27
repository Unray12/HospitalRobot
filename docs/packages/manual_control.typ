#import "@preview/boxed-sheet:0.1.2": *

= Package `manual_control`
#concept-block[
  #inline("Role")
  - Quản lý kênh điều khiển tay từ operator.
  - Đồng bộ trạng thái auto mode theo cả topic và service để tránh lệch trạng thái.

  #inline("Runtime node")
  - Executable: `manual_control`
  - Run:
    `ros2 run manual_control manual_control`
  - Main files:
    `manual_control/manual_control_node.py`,
    `manual_control/auto_mode_sync.py`

  #inline("I/O contract")
  - Subscribe:
    `/VR_control` (`std_msgs/String`), `/pick_robot` (`std_msgs/String`)
  - Publish:
    `/motor_cmd` (`std_msgs/String`), `/auto_mode` (`std_msgs/Bool`)
  - Service client:
    `/set_auto_mode` (`std_srvs/SetBool`)

  #inline("Configuration (current defaults)")
  - Section:
    `robot_common/robot_common/config.json -> manual_control`
  - `base_speed=10`
  - `manual_override_on_input=true`
  - Topics:
    `vr_control=/VR_control`, `pick_robot=/pick_robot`,
    `cmd_vel=/motor_cmd`, `auto_mode=/auto_mode`
  - Service:
    `set_auto_mode=/set_auto_mode`
  - Retry sync:
    `auto_mode_service_retry_period=0.2`,
    `auto_mode_service_max_attempts=30`

  #inline("Runtime behavior")
  - `autoMode=true` + có lệnh manual:
    nếu `manual_override_on_input=true` thì tự tắt auto mode rồi mới phát lệnh tay.
  - `autoMode=true` + `manual_override_on_input=false` -> bỏ qua lệnh tay.
  - `/pick_robot` nhận `"1"/"0"` để bật/tắt auto mode.
  - Khi mode đổi:
    publish `/auto_mode` và queue sync service với retry logic.

  #inline("AutoModeSync states")
  - `idle`: không có pending request.
  - `wait`: chờ service ready.
  - `send`: gửi request.
  - `give_up`: bỏ sync sau quá số lần thử.

  #inline("Validation commands")
  ```bash
  ros2 topic pub --once /VR_control std_msgs/String "{data: 'Forward'}"
  ros2 topic pub --once /pick_robot std_msgs/String "{data: '1'}"
  ros2 topic echo /auto_mode
  ros2 topic echo /motor_cmd
  ```

  #inline("Troubleshooting")
  - Toggle auto không phản ánh ở follower:
    xem log `AUTO_SYNC` và tình trạng service `/set_auto_mode`.
  - Manual command mất tác dụng:
    kiểm tra `autoMode` có đang bật hay không.
  - Sync bị drop:
    tăng `auto_mode_service_max_attempts` hoặc giảm retry period.
]
