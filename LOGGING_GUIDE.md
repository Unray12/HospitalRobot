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
[line_follower] [PLAN] Plan selected: plan_ntp
[line_follower] [PLAN_STATUS] plan_ntp | state=PLAN | step=2/5 | action=RotateRight | end_state=follow
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
- `PLAN_STATUS`: trạng thái runtime của plan (state/step/action).
- `FRAME`: frame cảm biến không hợp lệ.

### mqtt_bridge

- `BOOT`: khởi động node.
- `MQTT`: kết nối/mất kết nối broker.
- `BRIDGE_IN`: MQTT -> ROS.
- `BRIDGE_OUT`: ROS -> MQTT.
- `BRIDGE_FALLBACK`: fallback local khi MQTT lỗi.
- `KEYBOARD`: thao tác keyboard.
- `DEBUG_TOGGLE`: bật/tắt debug log bằng phím `e`.

### manual_control

- `AUTO_SYNC`: đồng bộ service `/set_auto_mode`.

### line_sensors / motor_driver

- `INFO/WARN/ERR`: kết nối serial, lỗi parse/thiết bị.
- `SENSOR`: log giá trị cảm biến line (khi debug bật).
- `MOTOR`: log tốc độ 4 bánh (khi debug bật).

## 6. Quy tắc nội dung message

- Viết ngắn, đúng thông tin cần debug.
- Ưu tiên có dữ liệu chẩn đoán:
  - tên plan
  - step hiện tại/tổng step
  - action hiện tại
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
self.log.info("Plan selected: plan_ntp", event="PLAN")
self.log.warning("Plan not found", event="PLAN")
self.log.error("Serial open failed", event="SERIAL")
```

## 9. Khuyến nghị vận hành

- Khi debug plan: lọc theo `PLAN` và `PLAN_STATUS`.
- Khi debug bridge: lọc theo `MQTT`, `BRIDGE_IN`, `BRIDGE_OUT`.
- Khi debug auto mode: lọc theo `MODE`, `AUTO_SYNC`.
- Bật/tắt debug sensor + motor:
  - Nhấn phím `e` trong `mqtt_bridge`.
  - Topic toggle: `/debug_logs_toggle` (`std_msgs/Bool`).
