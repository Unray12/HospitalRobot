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
  - Subscribe topics:
    `/line_sensors/frame`, `/auto_mode`, `/plan_select`
  - Publish topic:
    `/motor_cmd` (`std_msgs/String`, format `Direction:Speed`)
  - Service server:
    `/set_auto_mode` (`std_srvs/SetBool`)

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
  - Control:
    `Wait`, `Stop`, `Follow`, `Goto`, `Auto`.
  - Metadata:
    hỗ trợ `label` + `Goto` target theo index hoặc label.

  #inline("Config defaults (current)")
  - Section:
    `robot_common/robot_common/config.json -> line_follower`
  - Speed:
    `base_speed=8`, `turn_speed_left=6`, `turn_speed_right=6`
  - Crossing/pre-cross:
    `crossing_duration=3.0`,
    `cross_pre_forward_speed=8`,
    `cross_pre_forward_duration=3.0`,
    `cross_pre_stop_duration=1.0`
  - Rotate reacquire:
    `rotate_min_duration=2`,
    `rotate_line_mid_min_count=1`,
    `rotate_line_side_max_count=2`,
    `rotate_early_stop_on_side=true`,
    `rotate_line_side_min_count=1`
  - Plan:
    `cross_plan_name=plan_ntp`,
    `plan_end_state=stop`,
    alias `1/2/3/4` tương ứng các plan chính.

  #inline("Operational logs")
  - `MODE`: bật/tắt auto mode.
  - `PLAN`: selected, cleared, duplicate ignored, not found.
  - `PLAN_STATUS`: state/step/action/end_state (đã throttle).
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
