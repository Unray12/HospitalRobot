# Logging Standard - HospitalRobot

Tài liệu này định nghĩa chuẩn log đồng bộ cho toàn hệ thống ROS2.

## 1. Mục tiêu

- Log phải đọc nhanh, lọc nhanh, và truy vết được luồng xử lý.
- Cùng một format cho tất cả node.
- Có màu theo mức log để vận hành dễ theo dõi.

## 2. Format chuẩn

Mỗi log dùng format:

```text
[component] [event] message | key=value key2=value2
```

Ví dụ:

```text
[line_follower] [MODE] Auto Mode Enabled
[line_follower] [PLAN] Plan selected: a20
[line_follower] [PLAN_STATUS] a20 | state=PLAN | step=2/6 | action=RotateRight | label=scan_right | end_state=stop
[mqtt_bridge] [MQTT] Connected to MQTT broker
[manual_control] [AUTO_SYNC] set_auto_mode rejected: ...
```

## 3. Màu theo level

- `INFO`: Cyan
- `WARNING`: Yellow
- `ERROR`: Red

## 4. Bật/tắt màu

- Mặc định: bật màu.
- Tắt màu bằng một trong các cách:
  - set env `NO_COLOR=1`
  - hoặc set env `ROBOT_LOG_COLOR=0`

Ví dụ:

```bash
export NO_COLOR=1
```

## 5. Event code chuẩn nên dùng

### line_follower

- `MODE`: trạng thái auto/manual.
- `PLAN`: chọn/xóa/duplicate plan.
- `PLAN_STATUS`: trạng thái runtime của plan (state/step/action/label).
- `PLAN_EVENT`: event JSON publish ra `/plan_status`.
- `PLAN_MESSAGE`: step messages publish ra topic.
- `PLAN_STATUS_MQTT`: log khi publish step message vào `/plan_status`.
- `PLAN_CALLBACK`: callback payload publish ra `/plan_callback`.
- `AUTOLINE_CALLBACK`: callback AutoLine hook.
- `FRAME`: frame cảm biến không hợp lệ.

### mqtt_bridge

- `BOOT`: khởi động node.
- `MQTT`: kết nối/mất kết nối broker.
- `BRIDGE_IN`: MQTT -> ROS.
- `BRIDGE_OUT`: ROS -> MQTT.
- `BRIDGE_FALLBACK`: fallback local khi MQTT lỗi.
- `PLAN_MQTT`: normalize/invalid plan command.
- `PLAN_STATUS_MQTT`: ROS plan status -> MQTT.
- `PLAN_MESSAGE_MQTT`: ROS plan message -> MQTT.
- `KEYBOARD`: thao tác keyboard.
- `DEBUG_TOGGLE`: bật/tắt debug log bằng phím `e`.

### manual_control

- `AUTO_SYNC`: đồng bộ service `/set_auto_mode`.

### line_sensors

- `SERIAL`: kết nối/mất kết nối serial.
- `DEBUG_TOGGLE`: bật/tắt debug log.
- `SENSOR`: log frame cảm biến (khi debug bật).

### motor_driver

- `SERIAL`: kết nối/mất kết nối serial.
- `DEBUG_TOGGLE`: bật/tắt debug log.
- `MOTOR`: log tốc độ 4 bánh (khi debug bật).

## 6. Quy tắc nội dung message

- Viết ngắn, đúng thông tin cần debug.
- Ưu tiên có dữ liệu chẩn đoán:
  - tên plan
  - step hiện tại/tổng step
  - action hiện tại
  - label hiện tại (nếu có)
  - end_state
  - mã lỗi hoặc exception

## 7. Log quá nhiều (anti-spam)

- Trạng thái plan đã có throttle (`plan_status_log_period`) để tránh spam.
- Chỉ log khi:
  - trạng thái đổi
  - hoặc hết chu kỳ log định kỳ.

## 8. API dùng chung

Dùng `LogAdapter` trong:

- `robot_common/robot_common/logging_utils.py`

Node khởi tạo:

```python
self.log = LogAdapter(self.get_logger(), "line_follower")
self.log.info("Plan selected: a20", event="PLAN")
self.log.warning("Plan not found", event="PLAN")
self.log.error("Serial open failed", event="SERIAL")
```

## 9. Khuyến nghị vận hành

- Khi debug plan: lọc theo `PLAN`, `PLAN_STATUS`, `PLAN_EVENT`.
- Khi debug bridge: lọc theo `MQTT`, `BRIDGE_IN`, `BRIDGE_OUT`, `PLAN_MQTT`.
- Khi debug auto mode: lọc theo `MODE`, `AUTO_SYNC`.
- Bật/tắt debug sensor + motor:
  - Nhấn phím `e` trong `mqtt_bridge`.
  - Topic toggle: `/debug_logs_toggle` (`std_msgs/Bool`).
