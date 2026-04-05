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
  - `line_sensor` (mặc định, tương thích hành vi cũ)
  - `huskylens`
  - `hybrid`
- `tracking.strict_mode`: nếu true thì strategy invalid sẽ STOP.
- `tracking.line_frame_stale_sec`: timeout dữ liệu line sensor.
- `tracking.huskylens_frame_stale_sec`: timeout dữ liệu huskylens.
- `tracking.log_invalid_period`: chu kỳ log khi strategy invalid liên tục.

`line_follower.huskylens`:

- `enabled`: cờ tương thích ngược cho subscription huskylens.
- `topic_frame`: topic JSON từ `huskylens_sensor`.
- `max_abs_error`: giới hạn `abs(error)` hợp lệ.
- `control_gain`: hệ số khuếch đại error.
- `deadband`: vùng chết quanh 0 để đi thẳng.
- `fallback_on_invalid`: dùng cho routing subscription/fallback policy.

## 3. Sample Config

### line_sensor mode (recommended default)

```json
"tracking": {
  "strategy": "line_sensor",
  "strict_mode": false
}
```

### huskylens mode

```json
"tracking": {
  "strategy": "huskylens",
  "strict_mode": false
},
"huskylens": {
  "enabled": true,
  "topic_frame": "/huskylens/frame",
  "max_abs_error": 120,
  "control_gain": 1.0,
  "deadband": 1.0
}
```

### hybrid mode

```json
"tracking": {
  "strategy": "hybrid",
  "strict_mode": false
}
```

## 4. Tuning Guide (HuskyLens)

- `control_gain`:
  - tăng để phản ứng quay mạnh hơn.
  - giảm nếu xe rung/lắc khi bám line.
- `deadband`:
  - tăng để xe ít quay khi error nhỏ.
  - giảm nếu xe phản ứng chậm với lệch line.
- `max_abs_error`:
  - giảm để lọc outlier mạnh hơn.
  - tăng nếu camera placement gây sai lệch lớn hợp lệ.

## 5. Troubleshooting

- Xe luôn fallback line_sensor:
  - kiểm tra `/huskylens/frame` có `connected=1`, `algorithm_set=1`, `valid=1`.
  - kiểm tra `huskylens_frame_stale_sec` không quá nhỏ.
- Xe dừng ngay khi mất HuskyLens:
  - kiểm tra `tracking.strict_mode=true` hoặc đổi sang `false`.
- Strategy không đúng mong đợi:
  - kiểm tra log event `STRATEGY`, `HUSKYLENS`, `LINE_SENSOR`, `FALLBACK`, `TRACKING`.
