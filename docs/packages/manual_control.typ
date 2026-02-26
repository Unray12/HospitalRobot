#import "@preview/boxed-sheet:0.1.2": *

= Package `manual_control`
#concept-block[
  #inline("Role")
  - Nhận lệnh điều khiển tay từ operator.
  - Đồng bộ auto mode giữa topic và service.

  #inline("Runtime node")
  - Executable: `manual_control`
  - Run:
    `ros2 run manual_control manual_control`
  - Main files:
    `manual_control/manual_control_node.py`,
    `manual_control/auto_mode_sync.py`

  #inline("I/O contract")
  - Subscribe: `/VR_control` (`std_msgs/String`)
  - Subscribe: `/pick_robot` (`std_msgs/String`)
  - Publish: `/motor_cmd` (`std_msgs/String`)
  - Publish: `/auto_mode` (`std_msgs/Bool`)
  - Service client: `/set_auto_mode` (`std_srvs/SetBool`)

  #inline("Runtime behavior")
  - `autoMode=true` -> bỏ qua command manual `/VR_control`.
  - `/pick_robot` nhận `"1"/"0"` để bật/tắt auto mode.
  - Khi mode đổi:
    publish `/auto_mode` và queue sync service với retry logic.

  #inline("Config defaults (current)")
  - Section: `robot_common/robot_common/config.json -> manual_control`
  - Speed: `base_speed=10`
  - Topic: `vr_control=/VR_control`
  - Topic: `pick_robot=/pick_robot`
  - Topic: `cmd_vel=/motor_cmd`
  - Topic: `auto_mode=/auto_mode`
  - Service: `set_auto_mode=/set_auto_mode`
  - Optional retry: `auto_mode_service_retry_period`
  - Optional retry: `auto_mode_service_max_attempts`

  #inline("AutoModeSync states")
  - `idle`: không có pending request.
  - `wait`: chờ service ready.
  - `send`: gửi request.
  - `give_up`: bỏ sync sau quá số lần thử.

  #inline("Troubleshooting checklist")
  - Toggle auto không phản ánh ở follower:
    xem log `AUTO_SYNC` và tình trạng service `/set_auto_mode`.
  - Manual command mất tác dụng:
    kiểm tra `autoMode` có đang bật hay không.
  - Sync bị drop:
    tăng `auto_mode_service_max_attempts` hoặc giảm retry period.
]
