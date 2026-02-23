#import "@preview/boxed-sheet:0.1.2": *

= Package `line_sensors`
#concept-block[
  #inline("Role")
  - Driver đọc cảm biến line từ serial.
  - Parse JSON thành frame chuẩn để `line_follower` dùng ngay.

  #inline("Runtime node")
  - Executable: `line_sensor_driver`
  `ros2 run line_sensors line_sensor_driver`
  - Main files:
    `line_sensors/line_sensor_driver_node.py`,
    `line_sensors/line_sensor_reader.py`

  #inline("I/O contract")
  - Publish topic:
    `/line_sensors/frame` (`std_msgs/Int16MultiArray`)
  - Payload layout:
    `[left_count, mid_count, right_count, left_full, mid_full, right_full]`
  - Subscribe: `/debug_logs_toggle` (`std_msgs/Bool`)

  #inline("Config defaults (current)")
  - Section:
    `robot_common/robot_common/config.json -> line_sensors`
  - Serial:
    `port=/dev/ttyACM1`, `baudrate=115200`, `timeout=0.1`
  - Publish:
    `topic=/line_sensors/frame`, `rate_hz=100`
  - Debug toggle topic:
    `/debug_logs_toggle`
  - Optional:
    `debug_log_period`, `debug_enabled_default`

  #inline("Parser guarantees")
  - JSON lỗi -> bỏ frame (không crash node).
  - Thiếu key `LineSensor` -> warning + bỏ frame.
  - Dict sensor rỗng -> không đánh dấu full-black sai.
  - Reader decode có `errors=\"ignore\"` để chịu lỗi byte từ serial.

  #inline("Operational logs")
  - Event `SENSOR`: log frame định kỳ khi debug bật.
  - Event `DEBUG_TOGGLE`: báo trạng thái ON/OFF log.
  - Warning phổ biến:
    `Corrupted JSON skipped`, `LineSensor key missing`.

  #inline("Troubleshooting checklist")
  - Không có frame:
    kiểm tra serial port/baudrate và cổng thiết bị thực tế.
  - Frame toàn 0 kéo dài:
    kiểm tra dây cảm biến, nguồn module, payload key mapping (`0x25/0x24/0x23`).
  - Log quá nhiều:
    tăng `debug_log_period` hoặc tắt `/debug_logs_toggle`.
]
