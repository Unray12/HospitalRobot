# MQTT Message Guide (HospitalRobot)

Tài liệu này mô tả đầy đủ cách điều khiển robot bằng MQTT, bám theo code hiện tại trong:
- `mqtt_bridge/mqtt_bridge/MQTTBridgeROS.py`
- `robot_common/robot_common/config.json`

## 1. Mục đích

- Chuẩn hóa payload MQTT để điều khiển robot ổn định.
- Giảm lỗi tích hợp giữa app điều khiển, MQTT broker và ROS2 runtime.
- Cung cấp checklist test nhanh cho vận hành và QA.

## 2. Thông số broker và topic

Thông số mặc định:
- Broker host: `127.0.0.1`
- Broker port: `1883`

Topic điều khiển (MQTT -> ROS2):
- `VR_control`
- `pick_robot`
- `plan_select`

Topic trạng thái (ROS2 -> MQTT):
- `plan_status`
- `plan_message`
- `face/camera`
- `robot_logs` (lọc từ `/rosout` theo cấu hình log bridge)

## 3. Contract chi tiết từng topic điều khiển

### 3.1 Topic `VR_control`

Ý nghĩa: gửi lệnh điều khiển tay.

Payload hợp lệ:
- `Forward`
- `Backward`
- `Left`
- `Right`
- `Stop`
- `RotateLeft`
- `RotateRight`

ROS2 tương ứng:
- Bridge publish sang `/VR_control` (`std_msgs/String`)
- `manual_control` chuyển thành `/motor_cmd` theo `base_speed`

### 3.2 Topic `pick_robot`

Ý nghĩa: bật/tắt auto mode toàn cục.

Payload hợp lệ:
- `1`: bật auto mode
- `0`: tắt auto mode

ROS2 tương ứng:
- Bridge publish sang `/pick_robot` (`std_msgs/String`)
- `manual_control` đồng bộ sang `/auto_mode` (`std_msgs/Bool`)
- `manual_control` gọi service `/set_auto_mode`

### 3.3 Topic `plan_select`

Ý nghĩa: chọn plan cho `line_follower`.

Payload khuyến nghị:
- `room:a20`, `room:a19`, `room:a18`, `room:a17`, `room:a16`, `room:a15`
- `room:1` đến `room:6`
- `clear` hoặc `room:0` để clear plan

Payload tương thích:
- `1..6`
- `phong:1..6`
- `plan:1..6`
- tên plan trực tiếp như `a20`, `plan_turn_right`

Map mặc định:
- `1 -> a20`
- `2 -> a19`
- `3 -> a18`
- `4 -> a17`
- `5 -> a16`
- `6 -> a15`

Lưu ý:
- Bridge normalize payload trước khi publish ROS topic `/plan_select`.
- Nếu room id không map được sẽ bị bỏ qua và warning log.

## 4. Ví dụ publish chuẩn

### 4.1 Chọn plan

```bash
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:1"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:a20"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "clear"
```

### 4.2 Bật/tắt auto mode

```bash
mosquitto_pub -h 127.0.0.1 -p 1883 -t pick_robot -m "1"
mosquitto_pub -h 127.0.0.1 -p 1883 -t pick_robot -m "0"
```

### 4.3 Điều khiển tay

```bash
mosquitto_pub -h 127.0.0.1 -p 1883 -t VR_control -m "Forward"
mosquitto_pub -h 127.0.0.1 -p 1883 -t VR_control -m "RotateLeft"
mosquitto_pub -h 127.0.0.1 -p 1883 -t VR_control -m "Stop"
```

## 5. Subscribe để theo dõi realtime

```bash
mosquitto_sub -h 127.0.0.1 -p 1883 -t VR_control -v
mosquitto_sub -h 127.0.0.1 -p 1883 -t pick_robot -v
mosquitto_sub -h 127.0.0.1 -p 1883 -t plan_select -v
mosquitto_sub -h 127.0.0.1 -p 1883 -t plan_status -v
mosquitto_sub -h 127.0.0.1 -p 1883 -t plan_message -v
mosquitto_sub -h 127.0.0.1 -p 1883 -t face/camera -v
```

## 6. Kiểm tra ROS2 song song

```bash
ros2 topic echo /VR_control
ros2 topic echo /pick_robot
ros2 topic echo /plan_select
ros2 topic echo /auto_mode
ros2 topic echo /plan_status
ros2 topic echo /plan_message
ros2 topic echo /face/camera
ros2 topic echo /motor_cmd
```

Kỳ vọng:
- `plan_select=room:1` -> `/plan_select` nhận `a20`
- `pick_robot=1` -> `/auto_mode` chuyển `True`
- `VR_control=Forward` -> `/motor_cmd` xuất `Forward:<speed>`

## 7. Keyboard teleop tích hợp trong mqtt_bridge

Mapping mặc định:
- `w/s/a/d`: `Forward/Backward/Left/Right`
- `space`: `Stop`
- `j/p`: `RotateLeft/RotateRight`
- `k`: toggle auto mode
- `e`: toggle debug logs
- `1..6`: chọn plan
- `0`: clear plan
- `q`: quit node

## 8. Echo suppression (tránh loop MQTT)

Bridge có cơ chế đánh dấu local publish và suppress message echo trong:
- `echo_suppress_window_sec = 0.35`

Mục đích:
- tránh lặp command khi bridge vừa publish vừa subscribe cùng topic.

## 9. Python client mẫu

```python
import paho.mqtt.client as mqtt

client = mqtt.Client()
client.connect("127.0.0.1", 1883, 60)

client.publish("pick_robot", "1")
client.publish("plan_select", "room:1")
client.publish("VR_control", "Stop")

client.disconnect()
```

## 10. Lỗi thường gặp và cách xử lý

- `plan_select` gửi rồi nhưng robot không chạy:
  - kiểm tra `line_follower` có load được plan file không
  - kiểm tra payload sau normalize có đúng tên plan
- MQTT có message nhưng ROS2 không thấy:
  - kiểm tra `mqtt_bridge` đang chạy
  - kiểm tra đúng topic name phân biệt hoa/thường (`VR_control` là chữ hoa)
- Auto mode không đổi:
  - kiểm tra `/pick_robot` và service `/set_auto_mode`
  - xem log `AUTO_SYNC` từ `manual_control`
- Message bị lặp:
  - kiểm tra client ngoài có publish trùng
  - kiểm tra không có nhiều bridge chạy song song
