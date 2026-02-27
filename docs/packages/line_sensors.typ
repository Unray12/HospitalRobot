#import "@preview/boxed-sheet:0.1.2": *

= Package `line_sensors`
#concept-block[
  #inline("Role")
  - Node gateway cho line sensor qua serial.
  - Chuyển payload JSON từ firmware thành frame ROS2 ổn định cho `line_follower`.
  - Có cơ chế chống nhiễu frame zero và tự reconnect serial.

  #inline("Runtime node")
  - Executable: `line_sensor_driver`
  - Run:
    `ros2 run line_sensors line_sensor_driver`
  - Main files:
    `line_sensors/line_sensor_driver_node.py`,
    `line_sensors/line_sensor_reader.py`

  #inline("I/O contract")
  - Publish:
    `/line_sensors/frame` (`std_msgs/Int16MultiArray`)
  - Frame layout:
    `[left_count, mid_count, right_count, left_full, mid_full, right_full]`
  - Subscribe:
    `/debug_logs_toggle` (`std_msgs/Bool`)
  - Internal source payload kỳ vọng:
    JSON có key `LineSensor`, với 3 cụm địa chỉ `0x25`, `0x24`, `0x23`.

  #inline("Configuration (current defaults)")
  - Section:
    `robot_common/robot_common/config.json -> line_sensors`
  - Serial:
    `port=/dev/ttyACM1`, `baudrate=115200`, `timeout=0.1`
  - Reconnect:
    `reconnect_period_sec=2.0`,
    fallback `[/dev/ttyACM1, /dev/ttyACM0]`
  - Publish:
    `topic=/line_sensors/frame`, `rate_hz=100`
  - Debug toggle topic:
    `/debug_logs_toggle`
  - Filter zero-frame:
    `zero_hold_sec=0.15`, `zero_min_streak=3`
  - Optional:
    `debug_log_period`, `debug_enabled_default`, `scan_prefixes`

  #inline("Runtime behavior")
  - Timer publish chạy theo `rate_hz`, đọc non-blocking từ serial buffer.
  - Nếu parse thành công:
    publish frame theo layout cố định cho downstream node.
  - Nếu frame toàn zero:
    giữ frame non-zero gần nhất trong cửa sổ `zero_hold_sec`.
  - Nếu mất serial:
    chạy reconnect timer và thử fallback ports + scan prefixes.

  #inline("Parser guarantees")
  - JSON lỗi -> bỏ frame (không crash node).
  - Thiếu key `LineSensor` -> warning + bỏ frame.
  - Dict sensor rỗng -> không đánh dấu full-black sai.
  - Reader decode có `errors=\"ignore\"` để chịu lỗi byte từ serial.
  - Khi mất serial, node tự quét và reconnect định kỳ theo prefix cấu hình.

  #inline("Observability")
  - Event `SENSOR`: log frame định kỳ khi debug bật.
  - Event `DEBUG_TOGGLE`: báo trạng thái ON/OFF log.
  - Event `SERIAL`: báo reconnect thành công.
  - Warning phổ biến:
    `Corrupted JSON skipped`, `LineSensor key missing`.

  #inline("Validation commands")
  ```bash
  ros2 topic echo /line_sensors/frame
  ros2 topic hz /line_sensors/frame
  ros2 topic pub --once /debug_logs_toggle std_msgs/Bool "{data: true}"
  ```

  #inline("Troubleshooting")
  - Không có frame:
    kiểm tra serial port/baudrate và cổng thiết bị thực tế.
  - Frame toàn 0 kéo dài:
    kiểm tra dây cảm biến, nguồn module, payload key mapping (`0x25/0x24/0x23`).
  - Log quá nhiều:
    tăng `debug_log_period` hoặc tắt `/debug_logs_toggle`.
]
