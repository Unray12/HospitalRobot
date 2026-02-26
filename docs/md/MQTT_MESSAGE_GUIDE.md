# Hướng Dẫn Gửi MQTT Message (HospitalRobot)

Tài liệu này hướng dẫn gửi MQTT message để điều khiển robot.

## 1. Thông tin MQTT mặc định

- Broker: `127.0.0.1`
- Port: `1883`
- Topics điều khiển:
  - `VR_control`: lệnh di chuyển tay
  - `pick_robot`: bật/tắt auto mode
  - `plan_select`: chọn plan theo phòng
- Topics phản hồi (bridge từ ROS2):
  - `plan_status`: trạng thái plan dạng JSON
  - `plan_message`: message tùy biến từ plan step
  - `robot_logs`: log filtered từ `/rosout`

Nguồn cấu hình: `robot_common/robot_common/config.json` -> `mqtt_bridge`.

## 2. Chuẩn payload khuyến nghị

### 2.1 `plan_select`

- Khuyến nghị:
  - `room:a20` -> `a20`
  - `room:a21` -> `a21`
  - `room:a22` -> `a22`
  - `room:a23` -> `a23`
  - `room:a24` -> `a24`
  - `room:a25` -> `a25`
  - `room:0` hoặc `clear` -> clear plan

- Tương thích ngược (vẫn hỗ trợ):
  - `1`, `2`, `3`, `4`, `5`, `6`
  - `room:1..6`
  - `phong:1..6`
  - `plan:1..6`
  - tên plan trực tiếp, ví dụ: `a20`, `a25`

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
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:a20"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:a21"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:a22"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:a23"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:a24"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:a25"
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

Theo dõi phản hồi từ ROS2:

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -t plan_status -v
mosquitto_sub -h 127.0.0.1 -p 1883 -t plan_message -v
mosquitto_sub -h 127.0.0.1 -p 1883 -t robot_logs -v
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
ros2 topic echo /plan_status
ros2 topic echo /plan_message
```

Kỳ vọng:
- Khi gửi `plan_select = room:a20`, ROS `/plan_select` nhận `a20`.
- Khi gửi `plan_select = room:a25`, ROS `/plan_select` nhận `a25`.
- Khi gửi `pick_robot = 1`, node auto mode chuyển sang bật.
- Khi gửi `VR_control = Forward`, manual_control publish lệnh sang `/motor_cmd`.

## 6. Python nhanh (paho-mqtt)

```python
import paho.mqtt.client as mqtt

client = mqtt.Client()
client.connect("127.0.0.1", 1883, 60)

client.publish("pick_robot", "1")        # bật auto
client.publish("plan_select", "room:a20") # chọn phòng a20
client.publish("VR_control", "Stop")     # lệnh tay

client.disconnect()
```

## 7. Lỗi thường gặp

- Gửi `room:a26` nhưng không chạy:
  - Hiện tại config chỉ map `a20..a25`.
- Gửi plan nhưng line_follower báo `Plan not found`:
  - Kiểm tra tên file plan trong `robot_common/robot_common/plans/`.
- Gửi MQTT có message nhưng ROS không nhận:
  - Kiểm tra `mqtt_bridge` đang chạy và kết nối đúng broker.
