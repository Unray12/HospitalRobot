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
  #text(10pt)[Version: 2026-02-23]
  #linebreak()
  #text(10pt)[Team: ACLAB]
  #v(0.8cm)
  #text(9pt)[Repository: #link("https://github.com/Unray12/HospitalRobot")[github.com/Unray12/HospitalRobot]]
]

#colbreak()

= System Overview
#concept-block[
  #inline("Purpose")
  Tài liệu này mô tả chi tiết kiến trúc kỹ thuật của workspace `HospitalRobot`,
  bao gồm interfaces, cấu hình runtime, luồng dữ liệu và checklist debug.

  #inline("In-scope packages")
  - `robot_common`: config/plan/command/logging shared layer
  - `line_sensors`: serial sensor driver
  - `line_follower`: FSM + plan executor
  - `motor_driver`: actuation bridge to motor controller
  - `manual_control`: manual path + auto mode sync
  - `mqtt_bridge`: MQTT/keyboard integration
  - `robot`: bringup entrypoint

  #inline("Primary control path")
  `line_sensors -> /line_sensors/frame -> line_follower -> /motor_cmd -> motor_driver`

  #inline("Secondary control paths")
  - Manual command path:
    `/VR_control -> manual_control -> /motor_cmd`
  - Auto mode path:
    `/pick_robot -> manual_control -> /auto_mode -> line_follower`
  - Plan select path:
    `/plan_select -> line_follower -> plan files`

  #inline("Core service interface")
  `/set_auto_mode` (`std_srvs/SetBool`) for mode synchronization.
]

#colbreak()

= Interface Matrix
#concept-block[
  #inline("Topics")
  - `/line_sensors/frame` (`std_msgs/Int16MultiArray`): sensor frame
  - `/motor_cmd` (`std_msgs/String`): low-level motion command
  - `/auto_mode` (`std_msgs/Bool`): global auto state
  - `/VR_control` (`std_msgs/String`): manual command input
  - `/pick_robot` (`std_msgs/String`): auto mode trigger
  - `/plan_select` (`std_msgs/String`): plan runtime selection
  - `/debug_logs_toggle` (`std_msgs/Bool`): runtime debug toggle

  #inline("Services")
  - `/set_auto_mode` (`std_srvs/SetBool`)
]

#colbreak()

= Runtime Procedure
#concept-block[
  #inline("Step 1 - Build and source")
  ```bash
  colcon build --symlink-install
  source install/setup.bash
  ```

  #inline("Step 2 - Bringup core runtime")
  ```bash
  ros2 run robot robot
  ```

  #inline("Step 3 - Start MQTT bridge (optional but recommended)")
  ```bash
  ros2 run mqtt_bridge mqtt_bridge
  ```

  #inline("Step 4 - Verify runtime interfaces")
  ```bash
  ros2 topic list
  ros2 topic echo /line_sensors/frame
  ros2 topic echo /motor_cmd
  ros2 service list | grep set_auto_mode
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
#include "packages/mqtt_bridge.typ"
#colbreak()
#include "packages/robot.typ"

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
