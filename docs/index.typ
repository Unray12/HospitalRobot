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
  write-title: true,
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

= System Overview
#concept-block[
  #inline("Workspace packages")
  - `robot_common`
  - `line_sensors`
  - `line_follower`
  - `motor_driver`
  - `manual_control`
  - `mqtt_bridge`
  - `robot`

  #inline("Main flow")
  - `line_sensors -> /line_sensors/frame -> line_follower -> /motor_cmd -> motor_driver`
  - `manual_control` và `mqtt_bridge` đẩy command/mode/plan vào cùng luồng.

  #inline("Core control service")
  - Service đồng bộ auto mode: `/set_auto_mode` (`std_srvs/SetBool`)

  #inline("Plan runtime")
  - Topic chọn plan: `/plan_select`
  - Plan files: `robot_common/robot_common/plans/*.json`
]

= Runtime Commands
#concept-block[
  #inline("Build & source")
  ```bash
  colcon build --symlink-install
  source install/setup.bash
  ```

  #inline("Bringup runtime")
  ```bash
  ros2 run robot robot
  ```

  #inline("Start MQTT bridge")
  ```bash
  ros2 run mqtt_bridge mqtt_bridge
  ```

  #inline("Debug quick checks")
  ```bash
  ros2 topic list
  ros2 topic echo /line_sensors/frame
  ros2 topic echo /motor_cmd
  ros2 service list | grep set_auto_mode
  ```
]

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

= Package Sheets
#include "packages/robot_common.typ"
#include "packages/line_sensors.typ"
#include "packages/line_follower.typ"
#include "packages/motor_driver.typ"
#include "packages/manual_control.typ"
#include "packages/mqtt_bridge.typ"
#include "packages/robot.typ"

= Build Docs
#concept-block[
  #inline("Compile single file")
  ```bash
  typst compile docs/index.typ docs/build/index.pdf
  ```

  #inline("Build all docs")
  ```powershell
  powershell -ExecutionPolicy Bypass -File scripts/build_typst.ps1
  ```
]
