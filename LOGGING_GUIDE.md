# Logging Guide - HospitalRobot

Tài liệu này định nghĩa chuẩn log thống nhất cho toàn bộ node trong workspace.

## 1. Mục tiêu

- Log phải dễ đọc cho vận hành và đủ dữ liệu cho debug kỹ thuật.
- Cùng một format giữa các package để grep/lọc nhanh.
- Tối ưu signal-to-noise: tránh spam nhưng vẫn giữ mốc quan trọng.

## 2. Format chuẩn bắt buộc

Mẫu log:

```text
[component] [event] message | key=value key2=value2
```

Ví dụ:

```text
[line_follower] [MODE] Auto Mode Enabled
[line_follower] [PLAN] Plan selected: a20
[line_follower] [PLAN_STATUS] a20 | state=PLAN | step=2/7 | action=RotateRight | end_state=stop
[mqtt_bridge] [MQTT] Connected to MQTT broker
[manual_control] [AUTO_SYNC] set_auto_mode rejected: service unavailable
```

## 3. Thành phần API dùng chung

File dùng chung:
- `robot_common/robot_common/logging_utils.py`

Khởi tạo trong node:

```python
from robot_common.logging_utils import LogAdapter

self.log = LogAdapter(self.get_logger(), "line_follower")
self.log.info("Auto Mode Enabled", event="MODE")
self.log.warning("Plan not found: a99", event="PLAN")
self.log.error("Serial write error", event="SERIAL")
```

## 4. Màu log và env control

Color mapping:
- `INFO`: cyan
- `WARNING`: yellow
- `ERROR`: red

Tắt màu:
- `NO_COLOR=1`
- hoặc `ROBOT_LOG_COLOR=0`

Ví dụ:

```bash
export NO_COLOR=1
```

## 5. Event taxonomy theo package

### 5.1 line_follower

- `MODE`: bật/tắt auto mode
- `PLAN`: chọn/clear/not-found/duplicate
- `PLAN_STATUS`: state machine status (throttle)
- `PLAN_EVENT`: JSON event publish `/plan_status`
- `PLAN_CALLBACK`: callback publish `/plan_callback`
- `PLAN_MESSAGE`: step message publish theo topic
- `PLAN_STATUS_MQTT`: trạng thái plan chuyển sang MQTT
- `FRAME`: dữ liệu frame đầu vào không hợp lệ

### 5.2 line_sensors

- `SERIAL`: connect/reconnect trạng thái serial
- `SENSOR`: frame cảm biến (khi debug ON)
- `DEBUG_TOGGLE`: bật/tắt debug log

### 5.3 motor_driver

- `SERIAL`: connect/reconnect và lỗi serial
- `MOTOR`: tốc độ 4 bánh (khi debug ON)
- `DEBUG_TOGGLE`: bật/tắt debug log

### 5.4 manual_control

- `MODE`: manual override và đổi mode
- `AUTO_SYNC`: retry/send/give-up khi sync service `/set_auto_mode`

### 5.5 mqtt_bridge

- `BOOT`: node startup
- `MQTT`: connect/disconnect broker
- `BRIDGE_IN`: MQTT -> ROS2
- `BRIDGE_OUT`: ROS2 -> MQTT
- `BRIDGE_FALLBACK`: fallback local khi MQTT down
- `PLAN_MQTT`: normalize/validate payload plan
- `PLAN_STATUS_MQTT`: publish plan status ra MQTT
- `PLAN_MESSAGE_MQTT`: publish plan message ra MQTT
- `KEYBOARD`: sự kiện vận hành từ bàn phím
- `DEBUG_TOGGLE`: đổi trạng thái debug logs runtime

### 5.6 camera_sensor

- `BOOT`: startup serial reader
- `SERIAL`: open/read/reconnect error
- `FRAME`: drop frame malformed

## 6. Quy tắc nội dung message

- Message ngắn, rõ và có context ngay trong dòng log.
- Ưu tiên bổ sung trường chẩn đoán trong `key=value`:
  - `plan`
  - `step`, `total_steps`
  - `state`
  - `action`
  - `end_state`
  - `topic`, `payload`
- Không log payload nhạy cảm hoặc quá dài liên tục.

## 7. Chống spam log

Cơ chế hiện có:
- `line_follower` throttle `PLAN_STATUS` theo `plan_status_log_period`
- `line_sensors` throttle debug theo `debug_log_period`
- `motor_driver` throttle debug theo `debug_log_period`
- `camera_sensor` chỉ log drop malformed theo chu kỳ đếm

Khuyến nghị:
- Dùng event code cố định trước khi tăng tần suất log.
- Khi cần trace sâu, bật debug tạm thời qua `/debug_logs_toggle`.

## 8. Cầu nối log ROS2 -> MQTT

`mqtt_bridge` có chế độ log bridge:
- subscribe `/rosout`
- lọc theo `source_nodes` và `keywords`
- publish sang MQTT topic `robot_logs`

Mục tiêu:
- dashboard/UI có thể nhận sự kiện plan quan trọng không cần đọc trực tiếp ROS logs.

## 9. Checklist review log khi thêm tính năng mới

1. Có `component` rõ ràng theo package.
2. Có `event` nhất quán, không đặt tên tự do trùng nghĩa.
3. Có đủ key-value để debug từ xa.
4. Không spam ở vòng lặp nhanh.
5. Có ít nhất một log cho state transition quan trọng.

## 10. Quy trình debug nhanh theo tình huống

- Robot không đi:
  - xem `line_follower [MODE]` và `/auto_mode`
  - xem `line_follower [PLAN]` và `plan not found`
  - xem `motor_driver [SERIAL]`
- Plan không chạy đúng bước:
  - xem `PLAN_STATUS`, `PLAN_EVENT`, `PLAN_MESSAGE`
- MQTT điều khiển không vào ROS:
  - xem `mqtt_bridge [MQTT]` và `BRIDGE_IN`
- Manual override không hoạt động:
  - xem `manual_control [MODE]` và `AUTO_SYNC`
