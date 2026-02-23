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
  - MQTT subscribe:
    `vr_control`, `pick_robot`, `plan_select`
  - ROS publish:
    `/VR_control`, `/pick_robot`, `/plan_select`, `/debug_logs_toggle`

  #inline("Keyboard mapping")
  - Move:
    `w/s/a/d`, stop `space`
  - Rotate:
    `j` (left), `p` (right)
  - Mode/debug:
    `k` toggle auto, `e` toggle debug logs
  - Plan hotkeys:
    `1..4` select plan, `0` clear
  - Quit:
    `q`

  #inline("Config defaults (current)")
  - Section:
    `robot_common/robot_common/config.json -> mqtt_bridge`
  - Broker:
    `address=127.0.0.1`, `port=1883`
  - Topic names:
    `VR_control`, `pick_robot`, `plan_select`
  - Debug topic:
    `/debug_logs_toggle`

  #inline("Runtime architecture")
  - MQTT network loop chạy thread riêng.
  - Keyboard loop chạy worker thread riêng.
  - Khi MQTT mất kết nối, plan key có fallback publish local ROS topic.

  #inline("Troubleshooting checklist")
  - Bridge không lên:
    kiểm tra `paho-mqtt` và broker connectivity.
  - Nhấn phím mà robot không đổi mode:
    kiểm tra `/pick_robot` và `/auto_mode`.
  - Plan key không có tác dụng:
    kiểm tra `plan_keys` mapping và tên file plan.
]
