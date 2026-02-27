#import "@preview/boxed-sheet:0.1.2": *
#import "@preview/chronos:0.3.0"

#set text(font: (
  "Times New Roman",
  "SimSun",
))
#show raw: set text(font: "Consolas")

#let homepage = link("https://github.com/")[HospitalRobot]
#let author = "ACLAB Team"
#let title = "HospitalRobot ROS2 Boxed Sheet"

#let my-colors = (
  rgb(190, 149, 196),
  rgb("#f39f71"),
  rgb(102, 155, 188),
  rgb(229, 152, 155),
  rgb("6a4c93"),
  rgb("E0A500"),
  rgb("#934c84"),
  rgb("#934c5a"),
)

#show: boxedsheet-scaling.with(
  title: title,
  homepage: homepage,
  authors: author,
  write-title: false,
  page-w: 350.28pt,
  page-h: 241.89pt,
  title-align: left,
  title-number: true,
  title-delta: 2pt,
  scaling-size: false,
  font-size: 5.8pt,
  line-skip: 5.9pt,
  x-margin: 10pt,
  y-margin: 30pt,
  num-columns: 1,
  column-gutter: 3pt,
  numbered-units: false,
  color-box: my-colors,
)

#align(center)[
  #v(4.5cm)
  #text(18pt, weight: "bold")[HospitalRobot]
  #v(0.7cm)
  #text(12pt, weight: "semibold")[ROS2 Technical Report]
  #v(0.6cm)
  #text(10pt)[Version: 2026-02-26]
  #linebreak()
  #text(10pt)[Team: ACLAB]
  #v(0.8cm)
  #text(9pt)[Repository: #link("https://github.com/Unray12/HospitalRobot")[github.com/Unray12/HospitalRobot]]
]

#colbreak()

= System Overview
#concept-block[
  #inline("Mục tiêu tài liệu")
  Tài liệu này là đặc tả kỹ thuật đầy đủ cho runtime ROS2 của workspace `HospitalRobot`.
  Mục tiêu là giúp developer, operator và QA có cùng một chuẩn hiểu về:
  interface contract, luồng điều khiển, cấu hình, hành vi lỗi và cách kiểm chứng.

  #inline("Phạm vi hệ thống")
  - `robot_common`: lớp chia sẻ config, command protocol, logging và plan JSON
  - `line_sensors`: lấy frame line sensor qua serial
  - `line_follower`: FSM bám line + plan executor
  - `motor_driver`: chuyển command thành tốc độ 4 bánh và gửi serial
  - `manual_control`: đồng bộ mode manual/auto và service sync
  - `camera_sensor`: đọc serial camera, chuẩn hoá payload nhận diện
  - `mqtt_bridge`: cầu nối MQTT/ROS2 + keyboard teleop
  - `robot`: gói bringup runtime

  #inline("Nguyên tắc kiến trúc")
  - Điều khiển chuyển động tách theo pipeline `sense -> decide -> act`
  - Auto mode và manual mode dùng chung đường xuất lệnh `/motor_cmd`
  - Mọi tích hợp ngoài ROS2 (MQTT, serial) phải có lớp chuẩn hoá dữ liệu
  - Runtime ưu tiên fail-safe: mất line hoặc dữ liệu lỗi thì dừng robot
]

#colbreak()

= Interface Matrix
#concept-block[
  #inline("Topics - data plane")
  - `/line_sensors/frame` (`std_msgs/Int16MultiArray`)
    layout: `[left_count, mid_count, right_count, left_full, mid_full, right_full]`
  - `/motor_cmd` (`std_msgs/String`)
    format chuẩn: `Direction:Speed`
  - `/auto_mode` (`std_msgs/Bool`)
    cờ mode auto dùng cho đồng bộ control path
  - `/VR_control` (`std_msgs/String`)
    command manual từ operator/MQTT/keyboard
  - `/pick_robot` (`std_msgs/String`)
    trigger mode auto bằng `"1"` hoặc `"0"`
  - `/plan_select` (`std_msgs/String`)
    chọn plan runtime hoặc lệnh `clear`
  - `/face/camera` (`std_msgs/String`)
    payload chuẩn hoá camera `<DEV1,FACE,score>` hoặc `<DEV1,NO_OBJECT>`

  #inline("Topics - observability plane")
  - `/plan_status` (`std_msgs/String`)
    JSON event/status theo tiến trình plan
  - `/plan_callback` (`std_msgs/String`)
    callback JSON tại mốc rotate -> autoline
  - `/plan_message` (`std_msgs/String`)
    payload step-level do plan phát ra
  - `/debug_logs_toggle` (`std_msgs/Bool`)
    bật/tắt debug log runtime ở node hỗ trợ

  #inline("Services")
  - `/set_auto_mode` (`std_srvs/SetBool`)
    service đồng bộ mode với `line_follower`
]

#colbreak()

= Runtime Procedure
#concept-block[
  #inline("Bước 1 - Build và source workspace")
  ```bash
  colcon build --symlink-install
  source install/setup.bash
  ```

  #inline("Bước 2 - Bringup runtime chính")
  ```bash
  ros2 run robot robot
  ```

  #inline("Bước 3 - Khởi động MQTT bridge (khuyến nghị)")
  ```bash
  ros2 run mqtt_bridge mqtt_bridge
  ```

  #inline("Bước 4 - Kiểm tra interface sau khi lên hệ thống")
  ```bash
  ros2 topic list
  ros2 topic echo /line_sensors/frame
  ros2 topic echo /motor_cmd
  ros2 topic echo /plan_status
  ros2 service list | grep set_auto_mode
  ```

  #inline("Bước 5 - Thử tuyến điều khiển end-to-end")
  ```bash
  ros2 topic pub --once /pick_robot std_msgs/String "{data: '1'}"
  ros2 topic pub --once /plan_select std_msgs/String "{data: 'a20'}"
  ros2 topic echo /motor_cmd
  ```
]

#colbreak()

= System Flow Diagram
#{
  set text(8pt)
  chronos.diagram({
    import chronos: *
    _par("line_sensors")
    _par("line_follower")
    _par("motor_driver")
    _seq("line_sensors", "line_follower", comment: "/line_sensors/frame")
    _seq("line_follower", "motor_driver", comment: "/motor_cmd")
  })
}

#colbreak()

= Package Sheets
#include "packages/robot_common.typ"
#colbreak()
#include "packages/line_sensors.typ"
#colbreak()
#include "packages/line_follower.typ"
#colbreak()
#include "packages/motor_driver.typ"
#colbreak()
#include "packages/manual_control.typ"
#colbreak()
#include "packages/camera_sensor.typ"
#colbreak()
#include "packages/mqtt_bridge.typ"
#colbreak()
#include "packages/robot.typ"

#colbreak()

= Documentation Style
#concept-block[
  #inline("Style chuẩn áp dụng toàn bộ docs")
  - Mỗi package phải có các mục theo đúng thứ tự:
    `Role -> Runtime node -> I/O contract -> Configuration -> Runtime behavior -> Observability -> Validation -> Troubleshooting`
  - Mọi giá trị default phải bám trực tiếp `robot_common/robot_common/config.json`
  - Mọi command format phải chỉ rõ ví dụ payload hợp lệ
  - Mọi topic/service phải ghi rõ kiểu message
  - Mọi hành vi fallback/recovery phải mô tả điều kiện kích hoạt và kết quả
]

#colbreak()

= Build Docs
#concept-block[
  #inline("Compile index only")
  ```bash
  typst compile docs/index.typ docs/build/index.pdf
  ```

  #inline("Compile all Typst docs in workspace")
  ```powershell
  powershell -ExecutionPolicy Bypass -File scripts/build_typst.ps1
  ```

  #inline("Output artifacts")
  - Main report: `docs/build/index.pdf`
  - Per-package sheets: `docs/build/packages/*.pdf`
]
