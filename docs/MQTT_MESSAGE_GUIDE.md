# Hướng Dẫn Gửi MQTT Message (HospitalRobot)

Tài liệu này hướng dẫn gửi MQTT message để điều khiển robot.

## 1. Thông tin MQTT mặc định

- Broker: `127.0.0.1`
- Port: `1883`
- Topics:
  - `VR_control`: lệnh di chuyển tay
  - `pick_robot`: bật/tắt auto mode
  - `plan_select`: chọn plan theo phòng

Nguồn cấu hình: `robot_common/robot_common/config.json` -> `mqtt_bridge`.

## 2. Chuẩn payload khuyến nghị

### 2.1 `plan_select`

- Khuyến nghị:
  - `room:1` -> `plan_ntp`
  - `room:2` -> `plan_straight`
  - `room:3` -> `plan_turn_right`
  - `room:4` -> `plan_stop`
  - `room:0` hoặc `clear` -> clear plan

- Tương thích ngược (vẫn hỗ trợ):
  - `1`, `2`, `3`, `4`
  - `phong:1..4`
  - `plan:1..4`
  - tên plan trực tiếp, ví dụ: `plan_turn_right`

Lưu ý:
- Khi chọn plan, hệ thống sẽ tự bật auto mode (`pick_robot=1`) theo cấu hình mặc định hiện tại.
- Có thể ghi đè theo từng plan bằng key `autoline` trong file JSON plan.
  - Tương thích key cũ: `auto_on_select`.

### 2.2 `pick_robot` (auto mode)

- `1`: bật auto mode
- `0`: tắt auto mode

### 2.3 `VR_control` (manual command)

Payload hợp lệ:
- `Forward`
- `Backward`
- `Left`
- `Right`
- `Stop`
- `RotateLeft`
- `RotateRight`

## 3. Gửi message bằng `mosquitto_pub`

### 3.1 Chọn phòng (plan)

```bash
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:1"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:2"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:3"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:4"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "clear"
```

### 3.2 Bật/tắt auto

```bash
mosquitto_pub -h 127.0.0.1 -p 1883 -t pick_robot -m "1"
mosquitto_pub -h 127.0.0.1 -p 1883 -t pick_robot -m "0"
```

### 3.3 Điều khiển tay

```bash
mosquitto_pub -h 127.0.0.1 -p 1883 -t VR_control -m "Forward"
mosquitto_pub -h 127.0.0.1 -p 1883 -t VR_control -m "Left"
mosquitto_pub -h 127.0.0.1 -p 1883 -t VR_control -m "Stop"
```

## 4. Theo dõi message để debug

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -t plan_select -v
mosquitto_sub -h 127.0.0.1 -p 1883 -t pick_robot -v
mosquitto_sub -h 127.0.0.1 -p 1883 -t VR_control -v
```

Nếu cần theo dõi tất cả:

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -t "#" -v
```

## 5. Kiểm tra từ phía ROS2

Trong lúc publish MQTT, mở terminal khác để check ROS2 topics:

```bash
ros2 topic echo /plan_select
ros2 topic echo /pick_robot
ros2 topic echo /VR_control
```

Kỳ vọng:
- Khi gửi `plan_select = room:2`, ROS `/plan_select` nhận `plan_straight`.
- Khi gửi `pick_robot = 1`, node auto mode chuyển sang bật.
- Khi gửi `VR_control = Forward`, manual_control publish lệnh sang `/motor_cmd`.

## 6. Python nhanh (paho-mqtt)

```python
import paho.mqtt.client as mqtt

client = mqtt.Client()
client.connect("127.0.0.1", 1883, 60)

client.publish("pick_robot", "1")      # bật auto
client.publish("plan_select", "room:3")# chọn phòng 3
client.publish("VR_control", "Stop")   # lệnh tay

client.disconnect()
```

## 7. Lỗi thường gặp

- Gửi `room:5` nhưng không chạy:
  - Hiện tại config chỉ map phòng `1..4`.
- Gửi plan nhưng line_follower báo `Plan not found`:
  - Kiểm tra tên file plan trong `robot_common/robot_common/plans/`.
- Gửi MQTT có message nhưng ROS không nhận:
  - Kiểm tra `mqtt_bridge` đang chạy và kết nối đúng broker.
