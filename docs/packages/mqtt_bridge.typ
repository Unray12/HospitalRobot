#import "@preview/boxed-sheet:0.1.2": *

= Package `mqtt_bridge`
#concept-block[
  #inline("Role")
  - Là gateway hai chiều giữa ROS2 và broker MQTT.
  - Cung cấp keyboard teleop để vận hành trực tiếp khi cần fallback HMI.
  - Chuẩn hoá command plan từ MQTT về tên plan runtime.

  #inline("Runtime node")
  - Executable: `mqtt_bridge`
  - Run:
    `ros2 run mqtt_bridge mqtt_bridge`
  - Main file:
    `mqtt_bridge/mqtt_bridge/MQTTBridgeROS.py`

  #inline("I/O contract")
  - MQTT subscribe:
    `VR_control`, `pick_robot`, `plan_select`
  - ROS publish:
    `/VR_control`, `/pick_robot`, `/plan_select`, `/debug_logs_toggle`
  - ROS subscribe -> MQTT publish:
    `/face/camera -> face/camera`,
    `/plan_status -> plan_status`,
    `/plan_message -> plan_message`
  - Plan command format (recommended):
    `room:a20..a15` hoặc `room:1..6`, `room:0` hoặc `clear` để clear plan.
  - Backward compatible plan command:
    `1..6`, `phong:*`, `plan:*`, direct plan name.

  #inline("Configuration (current defaults)")
  - Section:
    `robot_common/robot_common/config.json -> mqtt_bridge`
  - Broker:
    `address=127.0.0.1`, `port=1883`
  - Topics:
    `VR_control`, `pick_robot`, `plan_select`,
    `plan_status`, `plan_message`, `face/camera`
  - Debug topic:
    `/debug_logs_toggle`
  - Echo suppress:
    `echo_suppress_window_sec=0.35`
  - Log bridge:
    bridge log lọc từ `/rosout` theo `source_nodes` và `keywords`.

  #inline("Keyboard mapping")
  - Move:
    `w/s/a/d`, stop `space`
  - Rotate:
    `j` (left), `p` (right)
  - Mode/debug:
    `k` toggle auto, `e` toggle debug logs
  - Plan hotkeys:
    `1..6` select room-plan, `0` clear
  - Quit:
    `q`

  #inline("Runtime architecture")
  - MQTT network loop chạy thread riêng.
  - Keyboard loop chạy worker thread riêng.
  - Khi MQTT mất kết nối, plan key có fallback publish local ROS topic.
  - Cơ chế chống loopback:
    local publish được đánh dấu timestamp để suppress echo trong cửa sổ cấu hình.

  #inline("Validation commands")
  ```bash
  mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:1"
  mosquitto_pub -h 127.0.0.1 -p 1883 -t pick_robot -m "1"
  mosquitto_sub -h 127.0.0.1 -p 1883 -t plan_status -v
  ros2 topic echo /plan_select
  ```

  #inline("Troubleshooting")
  - Bridge không lên:
    kiểm tra `paho-mqtt` và broker connectivity.
  - Nhấn phím mà robot không đổi mode:
    kiểm tra `/pick_robot` và `/auto_mode`.
  - Plan key không có tác dụng:
    kiểm tra `plan_keys` mapping và tên file plan.
]
