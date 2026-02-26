#import "@preview/boxed-sheet:0.1.2": *

= Package `mqtt_bridge`
#concept-block[
  #inline("Role")
  - Cầu nối MQTT <-> ROS2.
  - Hỗ trợ keyboard teleop để điều khiển nhanh khi vận hành.

  #inline("Runtime node")
  - Executable: `mqtt_bridge`
  - Run:
    `ros2 run mqtt_bridge mqtt_bridge`
  - Main file:
    `mqtt_bridge/mqtt_bridge/MQTTBridgeROS.py`

  #inline("I/O contract")
  - MQTT subscribe: `vr_control`
  - MQTT subscribe: `pick_robot`
  - MQTT subscribe: `plan_select`
  - MQTT publish: `plan_status`
  - MQTT publish: `plan_message`
  - MQTT publish: `robot_logs`
  - ROS publish: `/VR_control`
  - ROS publish: `/pick_robot`
  - ROS publish: `/plan_select`
  - ROS publish: `/debug_logs_toggle`
  - ROS subscribe: `/plan_status`
  - ROS subscribe: `/plan_message`
  - ROS subscribe: `/rosout` (filtered)
  - Plan command format (recommended): `room:a20..a25`
  - Plan command format (recommended): `room:0` hoặc `clear` để clear plan.
  - Backward compatible plan command: `1..6`
  - Backward compatible plan command: `room:1..6`
  - Backward compatible plan command: `phong:1..6`
  - Backward compatible plan command: `plan:1..6`
  - Backward compatible plan command: direct plan name

  #inline("Keyboard mapping")
  - Move: `w`
  - Move: `s`
  - Move: `a`
  - Move: `d`
  - Move: `space` (stop)
  - Rotate: `j` (left)
  - Rotate: `p` (right)
  - Mode/debug: `k` toggle auto
  - Mode/debug: `e` toggle debug logs
  - Plan hotkeys: `1..6` select room-plan
  - Plan hotkeys: `0` clear
  - Quit: `q`

  #inline("Config defaults (current)")
  - Section: `robot_common/robot_common/config.json -> mqtt_bridge`
  - Broker: `address=127.0.0.1`
  - Broker: `port=1883`
  - Topic names: `VR_control`
  - Topic names: `pick_robot`
  - Topic names: `plan_select`
  - Topic names: `plan_status`
  - Topic names: `plan_message`
  - Topic names: `robot_logs`
  - Debug topic: `/debug_logs_toggle`

  #inline("Runtime architecture")
  - MQTT network loop chạy thread riêng.
  - Keyboard loop chạy worker thread riêng.
  - Khi MQTT mất kết nối, plan key có fallback publish local ROS topic.
  - Log bridge lọc `/rosout` theo keyword (mặc định `PLAN`/`PLAN_STATUS`).

  #inline("Troubleshooting checklist")
  - Bridge không lên:
    kiểm tra `paho-mqtt` và broker connectivity.
  - Nhấn phím mà robot không đổi mode:
    kiểm tra `/pick_robot` và `/auto_mode`.
  - Plan key không có tác dụng:
    kiểm tra `plan_keys` mapping và tên file plan.
]
