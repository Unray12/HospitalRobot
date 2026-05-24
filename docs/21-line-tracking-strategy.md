# Line Tracking Strategy

Tài liệu mô tả cơ chế chọn thuật toán dò line runtime trong `line_follower`.

> **Đọc cùng:** [`22-huskylens-integration.md`](22-huskylens-integration.md) cho HuskyLens
> details. [`10-plan-authoring.md`](10-plan-authoring.md) cho cách plan tương tác với strategy.

---

## 1. Decision flow

```
config.tracking.strategy
  ├─ line_sensor  →  dùng frame /line_sensors/frame
  ├─ huskylens    →  dùng frame /huskylens/frame
  └─ hybrid       →  ưu tiên huskylens, fallback line_sensor

Nếu strategy.compute(context) → None:
  ├─ strict_mode=true  →  STOP (FSM enter STATE_STOPPED)
  ├─ tracking_allow_line_sensor_fallback=false  →  STOP
  └─ otherwise (default)  →  fallback gọi _compute_line_sensor_command
                              với log warning theo log_invalid_period
```

`tracking_allow_line_sensor_fallback` được derive từ `not only_huskylens`. Khi
`only_huskylens=true`, line sensor topic không subscribe, không có frame để
fallback → behavior tương đương strict_mode.

---

## 2. Config fields (line_follower section)

`robot_common/robot_common/config.yaml -> line_follower`:

```json
"tracking": {
    "strategy": "huskylens",
    "strict_mode": true,
    "only_huskylens": true,
    "line_frame_stale_sec": 0.4,
    "huskylens_frame_stale_sec": 0.6,
    "log_invalid_period": 1.0
}
```

| Field                       |    Mặc định | Mô tả                                                   |
| --------------------------- | ----------: | ------------------------------------------------------- |
| `strategy`                  | `huskylens` | `line_sensor` / `huskylens` / `hybrid`                  |
| `strict_mode`               |      `true` | Strategy → None ⇒ STOP, không fallback                  |
| `only_huskylens`            |      `true` | Không subscribe `/line_sensors/frame`, disable fallback |
| `line_frame_stale_sec`      |       `0.4` | Line frame cũ hơn timeout này → stale                   |
| `huskylens_frame_stale_sec` |       `0.6` | HuskyLens frame cũ hơn → stale                          |
| `log_invalid_period`        |       `1.0` | Throttle log warn khi strategy invalid liên tục (giây)  |

### HuskyLens-specific

```json
"huskylens": {
    "enabled": true,
    "topic_frame": "/huskylens/frame",
    "lateral_deadband": 10.0,
    "heading_deadband": 3.0,
    "y_type_rotate_timeout": 5.0
}
```

| Field                   |           Mặc định | Mô tả                                                              |
| ----------------------- | -----------------: | ------------------------------------------------------------------ |
| `enabled`               |             `true` | Subscribe `/huskylens/frame`, bật xử lý huskylens                  |
| `topic_frame`           | `/huskylens/frame` | Topic name (config-driven)                                         |
| `lateral_deadband`      |             `10.0` | px, deadband cho `tail_offset_x` → Left/Right strafe               |
| `heading_deadband`      |              `3.0` | deg, deadband cho `angle_deg` → RotateLeft/Right                   |
| `y_type_rotate_timeout` |              `5.0` | Safety timeout cho rotate-until-y_type                             |
| `fallback_on_invalid`   |             `true` | (legacy alias) Cho phép fallback line_sensor khi huskylens invalid |

---

## 3. Strategy compute details

### LineSensor strategy

Input: `frame_context["line_sensor_frame"]` shape:

```python
{
  "left_count": int, "mid_count": int, "right_count": int,
  "left_full": bool, "mid_full": bool, "right_full": bool,
}
```

Compute logic (trong `_compute_line_sensor_command`):

```
total_black = left_count + mid_count + right_count

total_black == 0          →  Stop (lost line) hoặc hold trong plan
mid_full && sides ≤ 1     →  Forward (centered)
left_count > right_count  →  RotateLeft
right_count > left_count  →  RotateRight
mid_count == 0:
  left_count > 0          →  RotateLeft
  right_count > 0         →  RotateRight
else                       →  Forward
```

### HuskyLens strategy

Input: `frame_context["huskylens_frame"]` shape (từ
`huskylens_parser._build_frame`):

```python
{
  "connected": 0|1, "algorithm_set": 0|1, "valid": 0|1,
  "tail_offset_x": float, "angle_deg": float,
  "y_type": int, "line_length_y": int, "direction": int,
}
```

Compute logic:

```
not (connected && algorithm_set && valid)  →  None  (strategy invalid)

tail_offset_x < -lateral_deadband  →  Right    (strafe phải)
tail_offset_x >  lateral_deadband  →  Left     (strafe trái)
angle_deg    < -heading_deadband   →  RotateRight
angle_deg    >  heading_deadband   →  RotateLeft
else                                →  Forward
```

Lateral correction (mecanum strafe) **ưu tiên trước** heading correction.

### Hybrid strategy

```python
command = huskylens.compute(context)
if command is not None:
    return command
return line_sensor.compute(context)
```

Đơn giản nhưng hiệu quả: HuskyLens là "happy path", line sensor cover edge
case khi HuskyLens lost (vd robot nằm trên cross, line sensor thấy 3 zone đầy
nhưng HuskyLens không có arrow rõ).

---

## 4. Sample configs

### HuskyLens-only (production hospital robot)

```json
"tracking": {
    "strategy": "huskylens",
    "strict_mode": true,
    "only_huskylens": true,
    "huskylens_frame_stale_sec": 0.6
}
```

Behavior: chỉ tin HuskyLens. Frame stale → STOP. Phù hợp khi HuskyLens
ổn định và line sensor có nhiễu.

### Hybrid (resilient fallback)

```json
"tracking": {
    "strategy": "hybrid",
    "strict_mode": false,
    "only_huskylens": false
}
```

Behavior: ưu tiên HuskyLens, mất valid → line sensor. Phù hợp khi HuskyLens
có blind spot (vd góc cua gắt, ánh sáng kém).

### LineSensor-only (legacy / không có HuskyLens)

```json
"tracking": {
    "strategy": "line_sensor",
    "strict_mode": false
},
"huskylens": {
    "enabled": false
}
```

Behavior: chỉ dùng 3-zone line sensor. Không subscribe HuskyLens topic.

---

## 5. State transitions liên quan strategy

```
STATE_FOLLOWING
    │ strategy.compute → command
    ▼
publish /motor_cmd

STATE_FOLLOWING
    │ strategy.compute → None
    │ strict_mode=true
    ▼
STATE_STOPPED
    │ wait for valid frame
    │ (auto reset khi /auto_mode toggle)
```

---

## 6. Debug nhanh

```bash
# Check strategy đang active
ros2 topic echo /plan_status --once  # state field

# Check HuskyLens frame fresh
ros2 topic hz /huskylens/frame       # kỳ vọng ~10-20 Hz

# Check line sensor fresh
ros2 topic hz /line_sensors/frame    # kỳ vọng ~100 Hz

# Force strategy fallback log
ros2 topic pub /huskylens/frame std_msgs/String "{data: ''}" --once
# → line_follower log "FALLBACK" event nếu fallback enabled
```

---

## 7. Tham chiếu code

| Component             | File                                            | Line |
| --------------------- | ----------------------------------------------- | ---: |
| Strategy registry     | `line_follow_strategies.py`                     |    1 |
| FSM strategy dispatch | `line_follower.py:_resolve_tracking_strategy`   |  762 |
| LineSensor compute    | `line_follower.py:_compute_line_sensor_command` |  614 |
| HuskyLens compute     | `line_follower.py:_compute_huskylens_command`   |  663 |
| Fallback logic        | `line_follower.py:_follow_line`                 |  568 |
| Node wiring           | `line_follower_node.py:__init__`                |   17 |
