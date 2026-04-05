# Line Tracking Strategy

Tài liệu mô tả cơ chế chọn thuật toán dò line runtime trong `line_follower`.

## 1. Decision Flow

```text
tracking.strategy
  ├─ line_sensor  -> dùng frame /line_sensors/frame
  ├─ huskylens    -> dùng frame /huskylens/frame
  └─ hybrid       -> ưu tiên huskylens, lỗi thì fallback line_sensor

Nếu strategy trả None:
  - strict_mode=true  -> STOP
  - strict_mode=false -> fallback line_sensor
```

## 2. Config Fields

`robot_common/robot_common/config.json -> line_follower`

- `tracking.strategy`:
  - `line_sensor`
  - `huskylens`
  - `hybrid`
- `tracking.strict_mode`: nếu true thì strategy invalid sẽ STOP.
- `tracking.line_frame_stale_sec`: timeout dữ liệu line sensor.
- `tracking.huskylens_frame_stale_sec`: timeout dữ liệu huskylens.
- `tracking.log_invalid_period`: chu kỳ log khi strategy invalid liên tục.

`line_follower.huskylens`:

- `enabled`
- `topic_frame`
- `max_abs_error`
- `control_gain`
- `deadband`
- `fallback_on_invalid`

## 3. Sample Config

### huskylens mode

```json
"tracking": {
  "strategy": "huskylens",
  "strict_mode": true
},
"huskylens": {
  "enabled": true,
  "topic_frame": "/huskylens/frame",
  "max_abs_error": 120,
  "control_gain": 1.0,
  "deadband": 1.0,
  "fallback_on_invalid": false
}
```

### hybrid mode

```json
"tracking": {
  "strategy": "hybrid",
  "strict_mode": false
}
```
