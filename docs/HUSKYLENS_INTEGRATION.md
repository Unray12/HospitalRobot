# HuskyLens Integration Guide

Tài liệu mô tả luồng `output -> input` khi dùng `huskylens_sensor` với `line_follower`.

## 1. Luồng dữ liệu chuẩn

1. Thiết bị HuskyLens xuất JSON qua serial.
2. `huskylens_sensor` đọc serial, normalize dữ liệu và publish:
   - `/huskylens/frame` (`std_msgs/String`)
   - `/huskylens/valid` (`std_msgs/Bool`)
3. `line_follower` subscribe `/huskylens/frame`, parse JSON và lấy các trường:
   - `connected`
   - `algorithm_set`
   - `valid`
   - `error`
4. `line_follower` dùng `error` để tạo lệnh motor và publish `/motor_cmd`.

## 2. Topic contract

### `/huskylens/frame` (`std_msgs/String`)

Payload canonical sau normalize:

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

## 3. Input được chấp nhận ở HuskyLens sensor

`huskylens_sensor` chấp nhận 2 kiểu input:

1. Có sẵn `error`.
2. Không có `error` nhưng có `tail_offset_x` và `angle_deg`.

Nếu là kiểu 2 thì node tự quy đổi:

`error = round(0.8 * tail_offset_x + 0.2 * angle_deg)`

Sau đó luôn publish theo format canonical ở trên để `line_follower` đọc ổn định.

## 4. Cấu hình cần giữ

Trong `robot_common/robot_common/config.json`, giữ block:

```json
"huskylens": {
  "enabled": true,
  "topic_frame": "/huskylens/frame",
  "max_abs_error": 120,
  "control_gain": 1.0,
  "deadband": 1.0,
  "fallback_on_invalid": false
}
```

## 5. Kiểm tra nhanh

```bash
ros2 topic echo /huskylens/frame --once
ros2 topic echo /huskylens/valid --once
```

Nếu `/huskylens/frame` có `HuskylenSensor.error` và `line_follower` đang chạy strategy `huskylens`/`hybrid`, luồng I/O đã nối đúng.
