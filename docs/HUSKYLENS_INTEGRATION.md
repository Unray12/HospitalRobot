# HuskyLens Integration Guide

Tài liệu mô tả tích hợp thiết bị HuskyLens serial vào hệ thống HospitalRobot.

## 1. Kiến trúc luồng dữ liệu

- Thiết bị HuskyLens (serial `/dev/ttyACM2`) xuất JSON line-by-line.
- Node mới `huskylens_sensor` đọc serial, normalize frame và publish:
  - `/huskylens/frame` (`std_msgs/String`)
  - `/huskylens/valid` (`std_msgs/Bool`)
- `line_follower` subscribe `/huskylens/frame` theo cấu hình strategy (`line_sensor`, `huskylens`, `hybrid`) và dùng `error` bias quay trái/phải.
- Nếu HuskyLens invalid hoặc mất tín hiệu:
  - Với `fallback_on_invalid=true` sẽ quay về logic line sensor cũ.
  - Với `fallback_on_invalid=false` sẽ STOP để fail-safe.
- `mqtt_bridge` forward observability ROS -> MQTT:
  - `/huskylens/frame` -> `huskylens/frame`
  - `/huskylens/valid` -> `huskylens/valid`

## 2. Cấu hình

Trong `robot_common/robot_common/config.json`:

### `huskylens_sensor`

- `serial.port`: mặc định `/dev/ttyACM2`
- `serial.baudrate`: mặc định `9600`
- `serial.timeout`: mặc định `0.2`
- `publish.topic_frame`: mặc định `/huskylens/frame`
- `publish.topic_valid`: mặc định `/huskylens/valid`
- `publish.rate_hz`: mặc định `20`
- `reconnect_period_sec`, `fallback_ports`, `scan_prefixes`
- `parse_log_every`, `status_log_period`

### `line_follower.tracking`

- `strategy`: `line_sensor` | `huskylens` | `hybrid`
- `strict_mode`: strategy invalid -> STOP khi bật
- `line_frame_stale_sec`, `huskylens_frame_stale_sec`: ngưỡng timeout dữ liệu
- `log_invalid_period`: chu kỳ log khi input strategy không hợp lệ

### `line_follower.huskylens`

- `enabled`: bật/tắt tích hợp HuskyLens (default `false`)
- `topic_frame`: topic input frame
- `max_abs_error`: ngưỡng error hợp lệ
- `control_gain`: hệ số bias từ error
- `deadband`: vùng chết quanh error=0 để đi thẳng
- `fallback_on_invalid`: fallback về line_sensors khi HuskyLens invalid

### `mqtt_bridge.topics`

- `huskylens_frame_ros`, `huskylens_frame_mqtt`
- `huskylens_valid_ros`, `huskylens_valid_mqtt`

## 3. Run commands

Build:

```bash
colcon build --symlink-install
source install/setup.bash
```

Run HuskyLens node:

```bash
ros2 run huskylens_sensor huskylens_sensor
```

Run full bringup:

```bash
ros2 run robot robot
```

Run MQTT bridge:

```bash
ros2 run mqtt_bridge mqtt_bridge
```

## 4. Topic contract

### `/huskylens/frame` (`std_msgs/String`)

Payload JSON compact:

```json
{
  "HuskylenSensor": {
    "connected": 1,
    "algorithm_set": 1,
    "valid": 1,
    "error": -12,
    "y_type": 1,
    "line_length_y": 84,
    "direction": 0
  }
}
```

### `/huskylens/valid` (`std_msgs/Bool`)

- `true` khi `connected=1 && algorithm_set=1 && valid=1`
- `false` trong các trường hợp còn lại

## 5. Troubleshooting

- Không thấy data `/huskylens/frame`:
  - kiểm tra quyền đọc `/dev/ttyACM2`
  - kiểm tra baudrate và dây UART
  - xem log `[huskylens_sensor] [SERIAL]`
- Data có nhưng `valid=false`:
  - kiểm tra firmware HuskyLens có set line tracking algorithm
  - kiểm tra trường `valid/connected/algorithm_set` từ nguồn serial
- `line_follower` chưa dùng HuskyLens:
  - đặt `line_follower.tracking.strategy` thành `huskylens` hoặc `hybrid`
  - bật `line_follower.huskylens.enabled=true`
  - kiểm tra topic `line_follower.huskylens.topic_frame`
- MQTT dashboard không thấy HuskyLens:
  - kiểm tra `mqtt_bridge` đang chạy
  - kiểm tra map topic trong `mqtt_bridge.topics`
