#import "@preview/boxed-sheet:0.1.2": *

= Package `line_follower`
#concept-block[
  #inline("Role")
  - Bộ điều khiển chính cho line tracking.
  - Chạy FSM bám line và plan engine khi gặp giao cắt.
  - Nhận frame từ `line_sensors`, phát command sang `motor_driver`.

  #inline("Runtime node")
  - Executable: `line_follower`
  `ros2 run line_follower line_follower`
  - Main files:
    `line_follower/line_follower_node.py`,
    `line_follower/line_follower.py`

  #inline("I/O contract")
  - Subscribe topics: `/line_sensors/frame`
  - Subscribe topics: `/auto_mode`
  - Subscribe topics: `/plan_select`
  - Publish topics: `/motor_cmd` (`std_msgs/String`, format `Direction:Speed`)
  - Publish topics: `/pick_robot` (`std_msgs/String`)
  - Publish topics: `/plan_status` (`std_msgs/String`)
  - Publish topics: `/plan_callback` (`std_msgs/String`)
  - Service server: `/set_auto_mode` (`std_srvs/SetBool`)

  #inline("FSM states")
  - `FOLLOWING`, `TURN_LEFT`, `TURN_RIGHT`
  - `CROSS_PRE`: chạy forward/stop trước khi vào plan step.
  - `PLAN`: chạy action theo step hiện tại.
  - `CROSSING`: chế độ crossing đơn giản khi không có plan.
  - `STOPPED`: dừng hẳn khi mất line hoặc end_state=stop.

  #inline("Plan actions")
  - Move:
    `Forward`, `Backward`, `Left`, `Right` (`speed`, `duration`).
  - Rotate:
    `RotateLeft`, `RotateRight` (theo `duration` hoặc `until: "line"` + `timeout`).
    Hỗ trợ `min_duration`, `strict_line`.
  - Control:
    `Wait`, `Stop`, `Follow`, `Auto`, `AutoLine`.
  - Logic:
    `Goto`, `label`.
  - Step extras:
    `messages`, `continue_immediately` (alias `no_wait_cross`).

  #inline("Config defaults (current)")
  - Section: `robot_common/robot_common/config.json -> line_follower`
  - Speed: `base_speed=8`
  - Speed: `turn_speed_left=6`
  - Speed: `turn_speed_right=6`
  - Crossing: `crossing_duration=3.0`
  - Pre-cross: `cross_pre_forward_speed=8`
  - Pre-cross: `cross_pre_forward_duration=2.7`
  - Pre-cross: `cross_pre_stop_duration=1.0`
  - Rotate: `rotate_min_duration=5`
  - Rotate: `rotate_line_mid_min_count=1`
  - Rotate: `rotate_line_side_max_count=3`
  - Rotate: `rotate_early_stop_on_side=true`
  - Rotate: `rotate_line_side_min_count=1`
  - Plan: `cross_plan_name=a20`
  - Plan: `plan_end_state=stop`
  - Plan: `auto_on_plan_select=true`
  - Plan: `plan_lost_line_hold_sec=1.2`
  - Plan: `plan_lost_line_warn_period=0.6`
  - Plan alias: `1->a20`
  - Plan alias: `2->a21`
  - Plan alias: `3->a22`
  - Plan alias: `4->a23`
  - Plan alias: `5->a24`
  - Plan alias: `6->a25`

  #inline("Operational logs")
  - `MODE`: bật/tắt auto mode.
  - `PLAN`: selected, cleared, duplicate ignored, not found.
  - `PLAN_STATUS`: state/step/action/end_state (đã throttle).
  - `PLAN_EVENT`: payload JSON publish `/plan_status`.
  - `PLAN_CALLBACK`: callback JSON publish `/plan_callback`.
  - `FRAME`: frame cảm biến sai định dạng.

  #inline("Troubleshooting checklist")
  - Robot quay mãi:
    thêm `timeout` cho rotate step kiểu `until: "line"`.
  - Plan không load:
    kiểm tra tên plan từ `/plan_select` và file plan thực tế.
  - Auto ON nhưng không chạy:
    kiểm tra `/line_sensors/frame` có dữ liệu hợp lệ.
  - Manual/auto lệch trạng thái:
    kiểm tra đồng bộ `/auto_mode` và service `/set_auto_mode`.
]
