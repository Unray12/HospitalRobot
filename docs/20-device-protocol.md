# HospitalRobot Device Serial Protocol v1

Quy chuẩn giao tiếp giữa firmware thiết bị (ESP32/MicroPython, motor MCU, ...) và ROS
host qua USB-CDC serial. Mục tiêu: 1 schema duy nhất, dễ thêm thiết bị mới, dễ debug
bằng `cat /dev/...`.

> **Phiên bản:** 1 (current). Field `version` không xuất hiện trong envelope; mọi
> firmware bám theo schema dưới mặc định là v1. Khi đổi schema sẽ thêm field
> `protocol` ở envelope root và bump số.

> **Đọc cùng:** [`22-huskylens-integration.md`](22-huskylens-integration.md) cho payload
> cụ thể của HuskyLens. [`01-architecture.md`](01-architecture.md) §6 cho firmware
> overview.

## Envelope chuẩn

Mỗi dòng gửi qua serial là **một** JSON object có shape:

```json
{
  "dev_id":  "hrbot_<role>",
  "event":   "<event_type>",
  "payload": { ... }
}
```

| Field     | Bắt buộc | Mô tả                                                                                       |
| --------- | -------- | ------------------------------------------------------------------------------------------- |
| `dev_id`  | ✅       | Định danh thiết bị: `hrbot_motor` / `hrbot_line` / `hrbot_camera` / `hrbot_huskylens` / ... |
| `event`   | ✅       | Loại sự kiện: `boot` / `data` / `info` / `error` / `ack`                                    |
| `payload` | ✅       | Object chứa dữ liệu — schema tuỳ `event` và `dev_id`                                        |

Field không bắt buộc nhưng được khuyến nghị:

| Field | Mô tả                                            |
| ----- | ------------------------------------------------ |
| `ts`  | Số ms uptime của device, hữu ích để debug timing |

## Event types

### `boot` — banner khởi động (10 lần liên tiếp)

```json
{
  "dev_id": "hrbot_line",
  "event": "boot",
  "payload": { "fw": "line", "ver": 1 }
}
```

Probe (`scripts/init_serial_symlinks.py`) match `"dev_id":"hrbot_<role>"` trong banner để
gán symlink `/dev/hospitalrobot/<role>` → đúng port.

### `data` — payload sensor (runtime)

Schema phụ thuộc thiết bị, ví dụ:

```json
// hrbot_line
{"dev_id":"hrbot_line","event":"data","payload":{"sensors":{"0x23":{"s1":1,"s2":0,"s3":0,"s4":1},...}}}

// hrbot_huskylens
{"dev_id":"hrbot_huskylens","event":"data","payload":{"connected":1,"algorithm_set":1,"valid":1,"tail_offset_x":-72,"angle_deg":4.12,"y_type":1,"line_length_y":84,"direction":0}}

// hrbot_camera (face)
{"dev_id":"hrbot_camera","event":"data","payload":{"kind":"face","ids":[1,2,3]}}

// hrbot_camera (no object)
{"dev_id":"hrbot_camera","event":"data","payload":{"kind":"no_object"}}
```

### `info` — sự kiện không phải lỗi

```json
{
  "dev_id": "hrbot_camera",
  "event": "info",
  "payload": { "msg": "huskylens_connected" }
}
```

### `error` — lỗi runtime (không fatal)

```json
{
  "dev_id": "hrbot_camera",
  "event": "error",
  "payload": { "code": "no_connection", "msg": "HuskyLens not responding" }
}
```

### `ack` — phản hồi command (future)

Dùng khi host gửi command xuống device. Payload: `{"cmd":"<id>","ok":true,"detail":"..."}`.

## Quy ước về log spam

- ROS reader **PHẢI** bỏ qua thầm lặng `boot`, `info`, `error` frames (không log warning).
- ROS reader chỉ log `warning` cho frame có shape sai (không phải JSON, thiếu field bắt buộc).

## Thêm thiết bị mới — checklist

1. **Đặt `role`** — vd `imu`, `gripper`. Định danh là `hrbot_<role>`.
2. **Firmware** copy template `device_code/main.py` của 1 thiết bị, đổi `DEV_ID`, viết
   hàm `build_payload()` cho sensor cụ thể.
3. **Probe**: thêm regex `"dev_id"\s*:\s*"hrbot_<role>"` vào `scripts/init_serial_symlinks.py`
   (key `PROBES`). Hoặc dựa vào `dev_id` chung — xem mục "Probe match" dưới.
4. **Config** `robot_common/config.yaml`: thêm section `<role>` với `serial.port = "/dev/hospitalrobot/<role>"`.
5. **ROS node** mới: import `from robot_common.device_protocol import parse_envelope`,
   xử lý `event == "data"`.
6. **udev rules** (nếu cần) — symlink đã do `init_serial_symlinks.py` tạo, không cần
   thay đổi udev.

## Probe match

Probe chỉ cần dò chuỗi `"dev_id":"hrbot_<role>"` trong stream. Thêm role mới = thêm 1
entry vào `PROBES` trong `scripts/init_serial_symlinks.py`:

```python
PROBES["imu"] = {
    "pattern": re.compile(rb'"dev_id"\s*:\s*"hrbot_imu"'),
    "bauds":   [115200],
    "timeout": 3.0,
}
```

## Quy tắc bất biến

| Quy tắc                                       | Lý do                                                 |
| --------------------------------------------- | ----------------------------------------------------- |
| 1 JSON object **trên 1 dòng** (kết thúc `\n`) | Host đọc theo line; cấm xuống dòng giữa JSON          |
| Baudrate USB **luôn 115200**                  | Đồng nhất config, probe nhanh                         |
| `dev_id` viết thường, prefix `hrbot_`         | Nhất quán + tránh trùng với device khác trên hệ thống |
| Boot banner in **đúng 10 lần**                | Probe có window 3s — 10 lần \* 100ms ≈ 1s đủ overlap  |
| `event` ∈ tập đóng                            | Để consumer biết cách dispatch                        |
| Không in log/comment ngoài JSON               | Tránh corrupt stream                                  |

## Tham chiếu file

| Component                         | File                                                        |
| --------------------------------- | ----------------------------------------------------------- |
| Shared parser                     | `robot_common/robot_common/device_protocol.py`              |
| Probe                             | `scripts/init_serial_symlinks.py`                           |
| Firmware template (đơn giản)      | `line_sensors/device_code/main.py`                          |
| Firmware template (full features) | `huskylens_sensor/device_code/main.py` (WDT, deadline loop) |
| ROS reader template               | `line_sensors/line_sensors/line_sensor_reader.py`           |
| Serial base class                 | `robot_common/robot_common/serial_device.py`                |

## Motor MCU wire format (out-of-band)

Motor MCU **không dùng** envelope JSON. Đây là legacy binary-ish text protocol
riêng. ROS → motor MCU dùng format:

```text
(fl,fr,rl,rr)\n
```

Trong đó `fl,fr,rl,rr` là tốc độ 4 bánh integer (positive/negative). Ví dụ:

```text
(-8,-8,-8,-8)\n    # Forward speed 8
(0,0,0,0)\n        # Stop
(-5,-5,5,5)\n      # RotateRight speed 5
```

> Xem [`02-project-flow.md`](02-project-flow.md) §9 và [`CHANGELOG.md`](CHANGELOG.md)
> Critical-1 cho lịch sử fix wire format (newline vị trí sai trước 2026-05-19).
