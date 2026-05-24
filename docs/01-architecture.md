# HospitalRobot Architecture

Module map, dependency graph, và rationale cho các quyết định kiến trúc.

> **Phạm vi:** Tài liệu này mô tả _cấu trúc_ (modules, layers, contracts). Để
> biết _luồng dữ liệu runtime_, xem [`02-project-flow.md`](02-project-flow.md).

---

## 1. Layer overview

```
App / VR Client / Dashboard
        │  MQTT (publish/subscribe)
        ▼
MQTT broker (mosquitto)
        │
        ▼
Raspberry Pi 4  (ROS2 Humble)
  │
  ├── mqtt_bridge      ← edge: MQTT ↔ ROS
  │     • Transport (paho client)
  │     • Router (PlanCommandResolver)
  │     • Keyboard teleop
  │     • LogBridge (rosout → MQTT)
  │           │
  │           ▼  ROS topics
  │
  ├── Decision / Control
  │     • line_follower (FSM)
  │     • manual_control
  │           │  /motor_cmd
  │           ▼
  │     • motor_driver
  │           │  serial
  │           ▼
  │       Motor MCU  (mecanum x4)
  │
  └── Sensor ROS nodes
        • line_sensors
        • huskylens_sensor
        • camera_sensor
              │  serial USB-CDC @ 115200
              ▼
        Firmware ESP32 x3  (MicroPython)
```

---

## 2. ROS2 packages

9 packages, mỗi package 1 trách nhiệm:

| Package            | Trách nhiệm                                              | Entry point                      |
| ------------------ | -------------------------------------------------------- | -------------------------------- |
| `robot_common`     | Shared config, protocol, logging, serial base class      | (library)                        |
| `robot`            | Bringup launcher (chạy tất cả node bằng `LaunchService`) | `robot/main.py:29`               |
| `line_sensors`     | Đọc cảm biến line I2C từ ESP32                           | `line_sensor_driver_node.py:137` |
| `huskylens_sensor` | Đọc HuskyLens tracking data từ ESP32                     | `huskylens_sensor_node.py:117`   |
| `camera_sensor`    | Đọc face detection từ ESP32 camera                       | `camera_sensor/main.py:151`      |
| `line_follower`    | FSM điều khiển tự động theo line + plan                  | `line_follower_node.py:515`      |
| `motor_driver`     | Gửi lệnh motor qua serial CAN/MCU                        | `motor_driver_node.py:90`        |
| `manual_control`   | Xử lý lệnh tay từ VR/keyboard                            | `manual_control_node.py:105`     |
| `mqtt_bridge`      | Cầu nối MQTT ↔ ROS2 + keyboard teleop                    | `MQTTBridgeROS.py:479`           |

---

## 3. `robot_common` — shared foundation

Module pure-Python được mọi package khác import. Không phụ thuộc rclpy ngoài
runtime injection.

```
robot_common/
├── config_manager.py        # Load YAML/JSON config, cache, plan name validation
├── command_protocol.py      # parse_command / format_command
├── device_protocol.py       # Envelope v1 parser (boot/data/info/error/ack)
├── serial_device.py         # SerialDevice base class (reconnect, scan ports)
├── logging_utils.py         # LogAdapter (component+event structured logs)
├── config.yaml              # Config tất cả node (9 sections)
└── plans/                   # a15..a19.yaml + master reference
```

**Convention:**

- Mỗi `ConfigManager(package_name).load()` trả deep-copy → caller free mutate.
- Plan name phải khớp `^[A-Za-z0-9_-]+$` (sanitize path traversal).
- `LogAdapter(self.get_logger(), "component_name")` thay vì rclpy logger raw.

---

## 4. `line_follower` — internal structure

```
line_follower/line_follower/
├── line_follower.py          # LineFollowerFSM (FSM + plan executor, 990 LOC)
├── line_follow_strategies.py # Strategy wrappers (line_sensor, huskylens, hybrid)
└── line_follower_node.py     # ROS2 node, subscribes/publishes, wires FSM
```

### FSM states

```
STATE_FOLLOWING   ←─ default tracking
STATE_CROSSING    ←─ vừa detect cross, drive forward 2s
STATE_CROSS_PRE   ←─ 2-phase pre-cross (forward → stop) trước rotate
STATE_PLAN        ←─ thực thi step plan action (rotate/wait/follow/...)
STATE_STOPPED     ←─ paused, plan progress preserved
STATE_TURN_LEFT   ←─ snapshot state cho debugging (không có behavior riêng)
STATE_TURN_RIGHT  ←─ snapshot state
STATE_BACKWARD    ←─ unused (reserved)
```

### Strategy pattern

`HuskyLensStrategy` / `LineSensorStrategy` / `HybridStrategy` là **wrappers
mỏng** quanh FSM compute methods. Không tự đứng độc lập vì compute methods
có side effects (set `self.state`, gọi `self.stop()`, update
`self._plan_lost_line_since`).

Lý do giữ wrapper: cho phép FSM swap strategy qua config string mà không
branch khắp nơi. Xem docstring trong `line_follow_strategies.py`.

### Plan executor

Cùng class với FSM (tách thành `PlanExecutor` riêng đã được đề xuất nhưng
defer — xem [`30-code-review.md`](30-code-review.md) §Arch-1).

Actions hỗ trợ: `Forward`, `Backward`, `Left`, `Right`, `RotateLeft`,
`RotateRight`, `Follow`, `Stop`, `Wait`, `AutoLine`, `Goto`, `Label`.

Aliases (case-insensitive): `TurnLeft → RotateLeft`, `Pause → Wait`,
`Go → Forward`, etc. Đầy đủ tại `line_follower.py:_normalize_action`.

---

## 5. `mqtt_bridge` — internal structure

```
mqtt_bridge/mqtt_bridge/
├── MQTTBridgeROS.py     # MQTTBridgeNode (ROS2 node, paho client lifecycle)
├── plan_resolver.py     # PlanCommandResolver + extract_face_ids (pure)
├── log_throttler.py     # LogThrottler (throttle/change-detect, pure)
├── keyboard_input.py    # KeyboardInput (cross-platform, daemon thread)
└── log_bridge.py        # LogBridge (rosout → MQTT, filtered)
```

### Bridge data flow

```
MQTT broker                          ROS2 topics
   │                                     │
   │   _on_message (paho thread)         │
   ├──────────────► _inbound_queue ──────┼──► _drain_inbound_queue (20 Hz)
   │                                     │                │
   │                                     │                ▼
   │                                     │    _handle_inbound_event
   │                                     │                │
   │                                     │                ▼
   │   client.publish ◄──── (ROS callbacks publish MQTT)
   │                                     │
   ▼                                     ▼
```

### Echo suppression

Bridge vừa subscribe vừa publish cùng topic (vd `VR_control`). `_mark_local_publish`
ghi `(topic, payload) → timestamp` khi publish; `_is_recent_local_echo` filter
inbound nằm trong `echo_suppress_window_sec` (default 0.35s). Dict được prune
opportunistically khi > 64 entries để không leak.

### Pure extractable modules

- **`PlanCommandResolver`** — Nhận free-form MQTT plan command, trả canonical
  plan name. Pure function, 10 unit test, không cần rclpy. Hỗ trợ
  `room:X` / `phong:X` / `plan:X` / direct name / single digit.
- **`LogThrottler`** — Decide-only helper cho throttle theo period và emit
  on-change. Tách khỏi MQTTBridgeNode để test độc lập (5 test).

### Lifecycle

`MQTTBridgeNode.__init__` dùng `connect_async()` + `loop_start()` +
`reconnect_delay_set(1, 10)`. Node không crash khi broker chưa ready;
paho tự retry với exponential backoff.

---

## 6. Firmware (MicroPython)

Cả 3 firmware tuân theo [Device Protocol v1](20-device-protocol.md):

| Firmware                               | `dev_id`          | Output rate |       Protocol UART | USB-CDC baud |
| -------------------------------------- | ----------------- | ----------: | ------------------: | -----------: |
| `huskylens_sensor/device_code/main.py` | `hrbot_huskylens` |       10 Hz | 9600 → HuskyLens IC |       115200 |
| `line_sensors/device_code/main.py`     | `hrbot_line`      |       10 Hz |         I2C 100 kHz |       115200 |
| `camera_sensor/device_code/main.py`    | `hrbot_camera`    |       10 Hz | 9600 → HuskyLens IC |       115200 |

### Boot banner

Mọi firmware in 10 lần `{"dev_id":"hrbot_X","event":"boot","payload":{"fw":...,"ver":1}}`
với khoảng cách ~100–300ms. Probe (`scripts/init_serial_symlinks.py`) match
chuỗi `"dev_id":"hrbot_X"` để gán symlink `/dev/hospitalrobot/X`.

### Watchdog

HuskyLens firmware có hardware WDT (`WDT_TIMEOUT_MS = 3000`), feed mỗi loop
iteration. Hung UART → reset trong vài giây.

---

## 7. Serial layer

`SerialDevice` (base class) xử lý:

- Open với optional `exclusive=True` (HuskyLens, camera dùng).
- Reset I/O buffer + sleep 50ms cho USB-CDC enumerate.
- Reconnect: thử `port` chính → `fallback_ports` → scan `scan_prefixes`.
- Logger callback (info/warn/error).

Concrete classes: `LineSensorReader`, `HuskyLensSensorReader`,
`CameraSensorReader`, `MotorController`.

---

## 8. Dependency graph

```
                     ┌──────────────┐
                     │ robot_common │ ◄──────────────────┐
                     └──────┬───────┘                    │
                            │ (imported by all)          │
          ┌─────────────────┼─────────────────┐          │
          ▼                 ▼                 ▼          │
   line_sensors      huskylens_sensor   camera_sensor    │
          │                 │                 │          │
          └────────┬────────┘                 │          │
                   ▼                          │          │
            line_follower ◄───────────────────┘          │
                   │                                      │
                   ▼                                      │
             motor_driver ◄──── manual_control ───────────┤
                   ▲                   ▲                  │
                   └──── mqtt_bridge ──┘                  │
                               │       (imports robot_common)
                               ▼
                         robot (bringup)
```

Quy tắc:

- `robot_common` không import từ package khác.
- Mỗi sensor package không biết về `line_follower` (publish topic, không
  có direct call).
- `motor_driver` chỉ subscribe `/motor_cmd`, không biết publisher là ai.
- `mqtt_bridge` là edge: chuyển đổi protocol giữa MQTT và ROS.

---

## 9. Configuration boundary

Một file duy nhất: `robot_common/robot_common/config.yaml` với 9 top-level
sections (1 per package + 1 cho bringup).

```json
{
  "robot":           { "bringup": { "nodes": [...] } },
  "camera_sensor":   { "serial": {...}, "publish": {...}, "reconnect_*": ... },
  "line_sensors":    { "serial": {...}, "filter": {...}, "publish": {...} },
  "line_follower":   { "tracking": {...}, "huskylens": {...}, "plan_alias": {...} },
  "motor_driver":    { "serial": {...}, "subscribe": {...} },
  "manual_control":  { "base_speed": 10, "topics": {...}, "service": {...} },
  "huskylens_sensor":{ "serial": {...}, "publish": {...} },
  "mqtt_bridge":     { "broker": {...}, "topics": {...}, "keyboard": {...},
                       "plan_keys": {...}, "room_plans": {...},
                       "log_control": {...}, "log_bridge": {...} }
}
```

> **Caveat:** Plan→key mapping được duplicate ở 3 chỗ
> (`line_follower.plan_alias`, `mqtt_bridge.plan_keys`, `mqtt_bridge.room_plans`).
> Xem [`30-code-review.md`](30-code-review.md) §Low-1.

---

## 10. Threading model

| Component        |            Threads | Note                                                             |
| ---------------- | -----------------: | ---------------------------------------------------------------- |
| Sensor nodes     | 1 (rclpy executor) | Single-threaded executor, timer-driven                           |
| `line_follower`  |                  1 | 100 Hz timer (`0.01s`), all logic in executor                    |
| `motor_driver`   |                  1 | Subscriber callback + reconnect timer                            |
| `manual_control` |                  1 | + async service client retry                                     |
| `mqtt_bridge`    |                  3 | rclpy executor + paho `loop_start` thread + KeyboardInput thread |

`mqtt_bridge` là node duy nhất có cross-thread state. Echo suppression dùng
dict không lock (acceptable: paho publish thread-safe per paho docs;
single-key insertion/lookup race window ngắn so với suppress window).

---

## 11. Testing strategy

```
robot_common/test/              # Pure: config, protocol, logging (no rclpy)
line_sensors/test/              # Parser only (no node integration)
huskylens_sensor/test/          # Parser + reader reconnect
line_follower/test/             # FSM + strategy (no rclpy)
motor_driver/test/              # Wire format + command protocol (no rclpy)
manual_control/test/            # AutoModeSync state machine
mqtt_bridge/test/               # Pure helpers + skip-if-rclpy node tests
```

Test count (2026-05-19): **111 passed, 5 skipped** (rclpy-only tests skip
trên Windows dev).

Pytest discovery dùng `conftest.py` ở root để map ROS2 `<pkg>/<pkg>` →
short-form import.

---

## 12. Tham chiếu

- Code review chi tiết: [`30-code-review.md`](30-code-review.md)
- Runtime flow: [`02-project-flow.md`](02-project-flow.md)
- Operator guide: `../Note.txt`
- Serial spec: [`20-device-protocol.md`](20-device-protocol.md)
