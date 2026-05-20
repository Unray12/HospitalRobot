# HuskyLens Integration Guide

Tài liệu mô tả luồng `output -> input` khi dùng `huskylens_sensor` với
`line_follower`.

> **Status (2026-05-19):** Đã refactor — bỏ heuristic synthesize `error` từ
> `tail_offset_x`/`angle_deg`. Field `error` chỉ tồn tại nếu firmware gửi
> explicit. Xem [`CHANGELOG.md`](CHANGELOG.md).

---

## 1. Luồng dữ liệu chuẩn

```
HuskyLens IC
    │ UART @ 9600 baud
    ▼
ESP32 firmware (huskylens_sensor/device_code/main.py)
    │ JSON envelope v1 qua USB-CDC @ 115200 baud
    │ {"dev_id":"hrbot_huskylens","event":"data","payload":{...}}
    ▼
huskylens_sensor (ROS2 node)
    │ parse envelope, normalize
    ▼
/huskylens/frame  (std_msgs/String, JSON canonical)
/huskylens/valid  (std_msgs/Bool)
    │
    ▼
line_follower
    │ subscribe, parse, FSM compute
    ▼
/motor_cmd  (Direction:Speed)
```

---

## 2. Topic contract

### `/huskylens/frame` — `std_msgs/String`

Payload canonical sau normalize:

```json
{
  "HuskylenSensor": {
    "connected": 1,
    "algorithm_set": 1,
    "valid": 1,
    "tail_offset_x": -16,
    "angle_deg": 3.7,
    "y_type": 1,
    "line_length_y": 84,
    "direction": 0
  }
}
```

Field `error` xuất hiện **chỉ khi** firmware gửi explicit (không synthesize).

### `/huskylens/valid` — `std_msgs/Bool`

- `true` khi `connected=1 && algorithm_set=1 && valid=1`
- `false` trong các trường hợp còn lại

---

## 3. Input mà parser chấp nhận

`huskylens_sensor/huskylens_parser.normalize_huskylens_payload(raw)`:

- **Envelope chuẩn** — JSON string khớp Device Protocol v1 với
  `dev_id="hrbot_huskylens"` và `event="data"`. Đây là path production.
- **Dict trực tiếp** — `{"HuskylenSensor": {...}}` hoặc
  `{"HuskyLensSensor": {...}}`. Dành cho test legacy / firmware cũ.

Required fields (validate):

- `connected`, `algorithm_set`, `valid`
- Có **một trong hai** combo:
  - `tail_offset_x` + `angle_deg`, hoặc
  - `error`

Optional fields được giữ nếu có: `y_type`, `line_length_y`, `direction`,
`y_head`, `y_tail`.

---

## 4. Y-type classification (firmware-side)

Firmware (`huskylens_sensor/device_code/main.py`) phân loại vị trí line:

| Code | Tên                    | Ý nghĩa                                           |
| ---: | ---------------------- | ------------------------------------------------- |
|    0 | `Y_TYPE_NO_LINE`       | Không có arrow / rejected                         |
|    1 | `Y_TYPE_BOTTOM_TO_MID` | Tail BOT, head MID — line ngắn gần robot (cross)  |
|    2 | `Y_TYPE_MID_TO_TOP`    | Tail MID, head TOP — line dài phía trước (follow) |
|    3 | `Y_TYPE_BOTTOM_TO_TOP` | Tail BOT, head TOP — line full-frame              |
|    4 | `Y_TYPE_MID_TO_MID`    | Tail & head trong MID — floating segment          |

Code đồng bộ với `line_follower/line_follower.py:Y_TYPE_*` constants.

Sử dụng:

- Plan rotate `until: "y_type"` đợi `y_type == MID_TO_TOP` để kết thúc rotate.
- `BOTTOM_TO_MID` trigger cross_pre phase trong FSM (chuyển sang plan action).

---

## 5. Cấu hình line_follower huskylens

`robot_common/robot_common/config.yaml -> line_follower`:

```json
"tracking": {
    "strategy": "huskylens",
    "strict_mode": true,
    "only_huskylens": true,
    "line_frame_stale_sec": 0.4,
    "huskylens_frame_stale_sec": 0.6,
    "log_invalid_period": 1.0
},
"huskylens": {
    "enabled": true,
    "topic_frame": "/huskylens/frame",
    "lateral_deadband": 10.0,
    "heading_deadband": 3.0,
    "y_type_rotate_timeout": 5.0
}
```

| Field                       | Mặc định | Mô tả                                                 |
| --------------------------- | -------: | ----------------------------------------------------- |
| `lateral_deadband`          |     10.0 | Tail offset (px) trong band này → forward, ngoài → rẽ |
| `heading_deadband`          |      3.0 | Angle (deg) trong band này → forward, ngoài → rotate  |
| `y_type_rotate_timeout`     |      5.0 | Safety timeout cho rotate until y_type=MID_TO_TOP     |
| `huskylens_frame_stale_sec` |      0.6 | Frame cũ hơn timeout này → coi như invalid            |

---

## 6. Algorithm steering

`line_follower._compute_huskylens_command`:

```
tail_offset_x < -lateral_deadband  →  Right    (strafe phải)
tail_offset_x >  lateral_deadband  →  Left     (strafe trái)
angle_deg    < -heading_deadband   →  RotateRight
angle_deg    >  heading_deadband   →  RotateLeft
else                                →  Forward
```

Lateral correction (strafe) **ưu tiên trước** heading correction (rotate).

> **Note:** Lateral dùng `Left`/`Right` (mecanum strafe), không phải
> `RotateLeft`/`RotateRight`. Nếu robot không phải mecanum, cần đổi
> `_compute_huskylens_command` để dùng rotate cho lateral.

---

## 7. Firmware behavior

`huskylens_sensor/device_code/main.py` (target: MicroPython ESP32):

- 10 Hz deadline-driven loop.
- Boot banner 10 lần \* 300ms = 3s overlap với probe window.
- Hardware WDT 3s, feed mỗi loop iteration.
- **Arrow selection** (Medium-4 fix): sort arrows theo `y_tail` desc, chọn
  arrow gần robot nhất (`y_tail` lớn nhất). Không lấy ngẫu nhiên `arrows[0]`.
- Rejection rules trong `arrow_to_line_data`:
  - Arrow None hoặc thiếu endpoint → no-line.
  - Head không phía trên tail → no-line.
  - Endpoint ngoài image bounds → no-line.
  - `|tail_offset_x| > MAX_ABS_TAIL_OFFSET_X (110)` → no-line.

---

## 8. Kiểm tra nhanh

```bash
ros2 topic echo /huskylens/frame --once
ros2 topic echo /huskylens/valid --once
ros2 topic hz   /huskylens/frame
```

Kỳ vọng:

- Rate ~10–20 Hz.
- `HuskylenSensor.connected = 1`, `algorithm_set = 1`, `valid = 1` khi đặt
  HuskyLens trước line đen.
- `tail_offset_x` đổi dấu khi di chuyển robot trái/phải.

Debug firmware trực tiếp (bỏ qua ROS):

```bash
sudo systemctl stop hospitalrobot-serial
PORT=/dev/ttyACM2
stty -F $PORT 115200 raw -echo
timeout 5 cat $PORT | head -20
sudo systemctl start hospitalrobot-serial
```

Output kỳ vọng: dòng JSON `{"dev_id":"hrbot_huskylens","event":"data",...}`.

---

## 9. Tham chiếu code

| Component          | File                                                           | Line |
| ------------------ | -------------------------------------------------------------- | ---: |
| Firmware loop      | `huskylens_sensor/device_code/main.py`                         |  328 |
| Y-type classifier  | same file                                                      |  127 |
| Parser (host)      | `huskylens_sensor/huskylens_sensor/huskylens_parser.py`        |   54 |
| Reader (host)      | `huskylens_sensor/huskylens_sensor/huskylens_sensor_reader.py` |    8 |
| Sensor node        | `huskylens_sensor/huskylens_sensor/huskylens_sensor_node.py`   |   58 |
| Strategy compute   | `line_follower/line_follower/line_follower.py`                 |  663 |
| Y-type FSM trigger | same file                                                      |  177 |
