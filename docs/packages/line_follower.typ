#import "@preview/boxed-sheet:0.1.2": *

= Package `line_follower`
#concept-block[
  #inline("Role")
  - Bộ điều khiển quyết định chuyển động chính của robot.
  - Kết hợp 2 năng lực:
    line following theo cảm biến và plan execution theo kịch bản JSON.
  - Là nguồn phát lệnh vận tốc thấp mức giao thức text sang `motor_driver`.

  #inline("Runtime node")
  - Executable: `line_follower`
  - Run:
    `ros2 run line_follower line_follower`
  - Main files:
    `line_follower/line_follower_node.py`,
    `line_follower/line_follower.py`

  #inline("I/O contract")
  - Subscribe topics:
    `/line_sensors/frame`, `/auto_mode`, `/plan_select`
  - Publish topics:
    `/motor_cmd` (`std_msgs/String`, format `Direction:Speed`)
    `/pick_robot` (`std_msgs/String`), `/plan_status` (`std_msgs/String`),
    `/plan_callback` (`std_msgs/String`)
  - Dynamic publish:
    tạo publisher `std_msgs/String` theo `step.messages[*].topic` (ví dụ `/plan_message`)
  - Service server:
    `/set_auto_mode` (`std_srvs/SetBool`)

  #inline("FSM states")
  - `FOLLOWING`, `TURN_LEFT`, `TURN_RIGHT`
  - `CROSS_PRE`: chạy forward/stop trước khi vào plan step.
  - `PLAN`: chạy action theo step hiện tại.
  - `CROSSING`: chế độ crossing đơn giản khi không có plan.
  - `STOPPED`: dừng hẳn khi mất line hoặc end_state=stop.

  #inline("Plan actions và semantics")
  - Move:
    `Forward`, `Backward`, `Left`, `Right` (`speed`, `duration`).
  - Rotate:
    `RotateLeft`, `RotateRight` (theo `duration` hoặc `until: "line"` + `timeout`).
  - Control:
    `Wait`, `Stop`, `Follow`, `Goto`, `Auto`, `AutoLine`.
  - Metadata:
    hỗ trợ `label`, `Goto(target)`, `start_without_cross`,
    `continue_immediately`, `strict_line`, `min_duration`,
    `messages[]`, `end_state=true` (force complete).

  #inline("Configuration (current defaults)")
  - Section:
    `robot_common/robot_common/config.json -> line_follower`
  - Speed:
    `base_speed=8`, `turn_speed_left=6`, `turn_speed_right=6`
  - Crossing/pre-cross:
    `crossing_duration=3.0`,
    `cross_pre_forward_speed=8`,
    `cross_pre_forward_duration=2.7`,
    `cross_pre_stop_duration=1.0`
  - Rotate reacquire:
    `rotate_min_duration=5`,
    `rotate_line_mid_min_count=1`,
    `rotate_line_side_max_count=3`,
    `rotate_early_stop_on_side=true`,
    `rotate_line_side_min_count=1`
  - Plan:
    `cross_plan_name=a20`,
    `plan_end_state=stop`,
    alias `1..6` tương ứng `a20..a15`.

  #inline("Runtime sequence (auto mode)")
  - Nhận frame từ `/line_sensors/frame` và cập nhật `self._last_frame`.
  - Timer 10ms gọi FSM `update(frame, now)`.
  - Nếu có result `(direction, speed)`:
    format thành `Direction:Speed` và publish `/motor_cmd`.
  - Song song:
    xử lý `AutoLine` request, step messages, plan status events.
  - Khi plan hoàn tất:
    tùy `end_state`, hoặc quay về follow, hoặc reset+stop an toàn.

  #inline("Plan select behavior")
  - Nhận `String` từ `/plan_select`, hỗ trợ alias map.
  - `clear` hoặc `0`:
    clear plan, tắt autoline, publish `/pick_robot=0`, tắt auto mode.
  - Chống rung chọn plan:
    cùng plan trong cửa sổ `plan_select_debounce_sec` sẽ bị bỏ qua.
  - Khi plan có `autoline=true`:
    tự bật auto mode bằng publish `/pick_robot=1` và `_set_auto_mode(True)`.

  #inline("Observability")
  - `MODE`: bật/tắt auto mode.
  - `PLAN`: selected, cleared, duplicate ignored, not found.
  - `PLAN_STATUS`: state/step/action/end_state (đã throttle).
  - `PLAN_EVENT`: JSON status event publish ra `/plan_status`.
  - `PLAN_CALLBACK`: JSON callback publish ra `/plan_callback`.
  - `PLAN_MESSAGE`: step messages publish theo topic khai báo trong plan.
  - `FRAME`: frame cảm biến sai định dạng.

  #inline("Validation commands")
  ```bash
  ros2 topic echo /plan_status
  ros2 topic echo /plan_callback
  ros2 topic pub --once /plan_select std_msgs/String "{data: 'a20'}"
  ros2 service call /set_auto_mode std_srvs/srv/SetBool "{data: true}"
  ros2 topic echo /motor_cmd
  ```

  #inline("Troubleshooting")
  - Robot quay mãi:
    thêm `timeout` cho rotate step kiểu `until: "line"`.
  - Plan không load:
    kiểm tra tên plan từ `/plan_select` và file plan thực tế.
  - Auto ON nhưng không chạy:
    kiểm tra `/line_sensors/frame` có dữ liệu hợp lệ.
  - Manual/auto lệch trạng thái:
    kiểm tra đồng bộ `/auto_mode` và service `/set_auto_mode`.
]
