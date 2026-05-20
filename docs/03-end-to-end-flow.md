# End-to-end Operational Flow

Walkthrough chi tiết các kịch bản vận hành thực tế: từ lúc cấp nguồn Pi đến
khi robot hoàn thành plan, kèm timing, message sequence, error recovery.

> **Đọc cùng:**
>
> - [`01-architecture.md`](01-architecture.md) — Module map, dependency graph
> - [`02-project-flow.md`](02-project-flow.md) — Mô tả flow theo subsystem
>   (file hiện tại mô tả theo _scenario_, không trùng lặp)
> - [`Note.txt`](../Note.txt) — Operator quick reference

---

## Mục lục

1. [Boot sequence — từ cold start đến ready](#1-boot-sequence)
2. [Scenario A: Operator chọn plan qua MQTT](#2-scenario-a-operator-chọn-plan-qua-mqtt)
3. [Scenario B: Manual control qua VR/keyboard](#3-scenario-b-manual-control)
4. [Scenario C: HuskyLens tracking liên tục (auto follow)](#4-scenario-c-huskylens-tracking)
5. [Scenario D: Cross detection + plan execution](#5-scenario-d-cross-detection)
6. [Scenario E: Face detection trigger plan message](#6-scenario-e-face-detection)
7. [Plan execution lifecycle chi tiết](#7-plan-execution-lifecycle)
8. [Recovery scenarios — khi có sự cố](#8-recovery-scenarios)
9. [Priority & arbitration giữa các source](#9-priority--arbitration)
10. [Timing budget chi tiết](#10-timing-budget)

---

## 1. Boot sequence

### 1.1 Hardware power-on

```
T=0s     │ Pi cấp nguồn
T=0–5s   │ Ubuntu boot (systemd init)
T=5–8s   │ ESP32 boards (line/camera/huskylens) boot lên USB
         │   → emit boot banner JSON 10 lần
T=8s     │ Motor MCU sẵn sàng (115200 baud)
```

### 1.2 systemd service `hospitalrobot-serial`

```
systemd unit: /etc/systemd/system/hospitalrobot-serial.service
Trigger:      ExecStartPre=/bin/sleep 2  (chờ udev settle)
ExecStart:    /usr/bin/python3 scripts/init_serial_symlinks.py
```

Probe sequence trong `init_serial_symlinks.py:main`:

```
1. Liệt kê /dev/ttyUSB* + /dev/ttyACM*
2. Nếu chỉ có 1 ttyUSB* → gán cho motor (USB-Serial adapter)
3. Với mỗi port còn lại:
   - Mở @ 115200 baud, đọc 3s
   - Match regex "dev_id":"hrbot_<role>" trong stream
   - Nếu match → gán symlink /dev/hospitalrobot/<role>
4. Atomic create symlink (os.symlink → os.replace)
5. chmod 0o666 cho user truy cập không cần sudo
```

Output kỳ vọng:

```bash
$ ls -l /dev/hospitalrobot/
lrwxrwxrwx 1 root root 13 May 19 10:23 camera    → /dev/ttyACM0
lrwxrwxrwx 1 root root 13 May 19 10:23 huskylens → /dev/ttyACM1
lrwxrwxrwx 1 root root 13 May 19 10:23 line      → /dev/ttyACM2
lrwxrwxrwx 1 root root 13 May 19 10:23 motor     → /dev/ttyUSB0
```

### 1.3 ROS2 node startup

User chạy 2 terminal:

```bash
# Terminal 1
ros2 run robot robot                    # Bringup tất cả sensor + control node

# Terminal 2
ros2 run mqtt_bridge mqtt_bridge        # Bridge + keyboard
```

`robot/main.py` đọc `config.bringup.nodes` và spawn từng node qua
`launch_ros.actions.Node`:

```
line_sensor_driver → line_follower → motor_driver → manual_control → camera_sensor → huskylens_sensor
```

Mỗi node tự đọc config từ `robot_common/config.yaml`, mở serial nếu cần, đăng
ký topic, bắt đầu timer.

### 1.4 MQTT bridge handshake

`MQTTBridgeNode.__init__`:

```
T=0    │ ConfigManager.load()
T+0.1  │ Tạo paho client, set callbacks
T+0.1  │ client.reconnect_delay_set(1, 10)
T+0.1  │ client.connect_async("127.0.0.1", 1883, 60)  ← non-blocking
T+0.1  │ client.loop_start()  ← paho thread bắt đầu
T+0.5  │ paho TCP connect → mosquitto chấp nhận
T+0.5  │ on_connect callback → subscribe VR_control, pick_robot, plan_select
T+0.5  │ _mqtt_connected.set()
T+0.5  │ Log "Connected to MQTT broker"
```

Nếu broker chưa ready (vd mosquitto chậm hơn bridge):

```
T+0.1  │ connect_async không raise (broker chưa nghe)
T+1s   │ Paho retry (reconnect_delay_set min=1)
T+3s   │ Paho retry
       │ ...exponential backoff đến max=10s
T+Xs   │ mosquitto ready → on_connect fires → subscribed
```

Bridge **không crash** khi startup ngược thứ tự (fix Critical-3, 2026-05-19).

### 1.5 Trạng thái "ready"

Robot sẵn sàng nhận lệnh khi:

- `/dev/hospitalrobot/{motor,line,camera,huskylens}` đều có symlink.
- 4 sensor node + line_follower + motor_driver + manual_control đăng ký xong topic.
- `mqtt_bridge` set `_mqtt_connected`.
- `huskylens_sensor` log `STATUS: connected=1, algorithm_set=1, valid=0/1`.

Quan sát nhanh:

```bash
ros2 node list
# /camera_sensor, /huskylens_sensor, /line_follower, /line_sensor_driver,
# /manual_control, /motor_driver, /mqtt_bridge_ros2

ros2 topic hz /huskylens/frame    # ~10–20 Hz
ros2 topic hz /line_sensors/frame # ~100 Hz
```

---

## 2. Scenario A: Operator chọn plan qua MQTT

### Setup

- Robot đang ở vị trí xuất phát (line đen dưới sensor).
- `auto_mode = false`.
- App publish: `mosquitto_pub -t plan_select -m "room:1"`.

### Sequence

```
App
  │  mosquitto_pub -t plan_select -m "room:1"
  ▼
mosquitto broker
  │  distribute → on_message callback (paho thread)
  ▼
mqtt_bridge._on_message
  │  echo-suppress check → pass
  │  _inbound_queue.put(("plan_select", "room:1"))
  ▼
mqtt_bridge._drain_inbound_queue  (20 Hz timer, max 50 ms wait)
  │  _handle_inbound_event("plan_select", "room:1")
  │  _resolve_plan_command("room:1") → "a19"
  │  ros_plan_pub.publish(String("a19"))   → /plan_select
  ▼
line_follower._plan_cb("a19")
  │  ConfigManager.load_plan("a19")   (cache hit after first)
  │  follower.set_plan(steps, end_state)
  │  set_autoline_mode(True)          (theo plan.autoline)
  │  pick_pub.publish("1")
  │  _set_auto_mode(True)             → autoMode = True
  │  request_plan_start()             (start_without_cross=true)
  │  publish PLAN_STATUS "selected"
  ▼
mqtt_bridge._plan_status_cb
  │  client.publish("plan_status", payload)
  ▼
App  ← nhận confirm trên MQTT subscribe
```

### Step-by-step timing

| T (ms) | Component     | Action                                                         |
| -----: | ------------- | -------------------------------------------------------------- |
|      0 | App           | `mosquitto_pub`                                                |
|     ~5 | mosquitto     | Distribute message                                             |
|    ~10 | mqtt_bridge   | `_on_message` (paho thread), put queue                         |
| ~15–50 | mqtt_bridge   | `_drain_inbound_queue` timer (20 Hz, max 50ms latency)         |
|    ~50 | mqtt_bridge   | `_resolve_plan_command("room:1") → "a19"`                      |
|    ~50 | mqtt_bridge   | `ros_plan_pub.publish("a19")`                                  |
|    ~55 | line_follower | `_plan_cb("a19")`                                              |
|    ~55 | line_follower | `ConfigManager.load_plan("a19")` (cached after first)          |
|    ~55 | line_follower | `follower.set_plan(steps)`, reset FSM                          |
|    ~55 | line_follower | `_set_auto_mode(True)` → `autoMode=True`                       |
|    ~60 | line_follower | Publish `/plan_status` "selected" event                        |
|    ~60 | mqtt_bridge   | Subscribe `/plan_status` callback → publish MQTT `plan_status` |
|    ~65 | App           | Nhận confirm trên MQTT subscribe                               |

End-to-end latency: **~65 ms** từ app publish đến app nhận confirm.

### Plan validation (security)

Nếu app publish payload nguy hiểm:

```bash
mosquitto_pub -t plan_select -m "../config"
```

```
mqtt_bridge._resolve_plan_command("../config")
  → không match prefix room:/plan:/clear
  → không có trong room_plan_map/plan_key_map/known_plans
  → return text as-is → "../config"
  → ros_plan_pub.publish("../config")

line_follower._plan_cb("../config")
  → strip → "../config"
  → ConfigManager.load_plan("../config")
  → regex check ^[A-Za-z0-9_-]+$ → FAIL
  → log warning "plan name rejected (invalid characters): '../config'"
  → return None

line_follower._plan_cb tiếp:
  → if not plan_data: log "Plan not found" + return
```

Robot vẫn an toàn — file traversal bị block ở ConfigManager (fix H2,
2026-05-19).

---

## 3. Scenario B: Manual control

### Setup

- Robot ở chế độ `auto_mode = false`.
- Operator nhấn phím `w` trong terminal mqtt_bridge, hoặc app publish
  `VR_control "Forward"`.

### Sequence (keyboard path)

```
KeyboardInput thread (daemon)
    │ get_key(timeout=0.2) → "w"
    │ keyboard_map["w"] → "Forward"
    │ _inbound_queue.put(("__keyboard_vr__", "Forward"))
    ▼
ROS executor thread, _drain_inbound_queue (timer 20 Hz)
    │ _handle_inbound_event("__keyboard_vr__", "Forward")
    │ ros_pub.publish(String("Forward"))    ← /VR_control
    │ client.publish(VR_control, "Forward")  ← MQTT (for observability)
    │ _mark_local_publish(VR_control, "Forward")  ← echo suppress
    ▼
manual_control._manual_cb(String("Forward"))
    │ format_command("Forward", base_speed=10) → "Forward:10"
    │ cmd_pub.publish(String("Forward:10"))  ← /motor_cmd
    ▼
motor_driver._cmd_cb(String("Forward:10"))
    │ parse_command("Forward:10") → ("Forward", 10)
    │ motor.move("Forward", 10)
    │   → wheel_speeds = [int(round(-1*10))] * 4 = [-10, -10, -10, -10]
    │   → serial.write(b"(-10,-10,-10,-10)\n")
    ▼
Motor MCU
    → 4 bánh chạy
```

### Manual override khi đang auto

Nếu `auto_mode = true` và operator nhấn phím:

```
manual_control._manual_cb("Forward"):
    │ self.autoMode == True
    │ self.manual_override_on_input == True (config default)
    │ → self.autoMode = False
    │ → auto_pub.publish(Bool(False))       ← /auto_mode → False
    │ → _auto_sync.queue(False)             ← sync /set_auto_mode service
    │ → log "Manual override: auto mode disabled by operator input"
    │ → cmd_pub.publish(String("Forward:10"))

line_follower._auto_cb(Bool(False)):
    │ _set_auto_mode(False)
    │ → autoMode = False
    │ → follower.stop()
    │ → _publish_stop()
    │ → log "Auto Mode Disabled"
    │ → publish PLAN_STATUS "paused"
```

Plan progress được **preserved** (`STATE_STOPPED` giữ `_plan_index`).
Khi operator bật lại auto, plan tiếp tục từ step đang chờ.

### Echo suppression

`/VR_control` được publish bởi _cả_ bridge (từ keyboard) _và_ MQTT inbound
khi app cũng publish cùng topic. Echo suppress:

```
_mark_local_publish(VR_control, "Forward") tại T=0
   _recent_local_pub[(VR_control, "Forward")] = 0

MQTT echo về tại T+0.05s (round-trip qua broker):
   _is_recent_local_echo(VR_control, "Forward")
   → 0.05 < echo_suppress_window_sec (0.35)
   → return True → drop

MQTT echo về tại T+0.5s:
   → 0.5 > 0.35
   → return False → del entry, process normally
```

Dict prune khi > 64 entries để chống leak (fix H3).

---

## 4. Scenario C: HuskyLens tracking

Robot ở `auto_mode = true`, plan đang `STATE_FOLLOWING`, line đen phía trước
trong tầm nhìn HuskyLens.

### Loop @ 10 Hz (line_follower timer)

```
HuskyLens IC  (9600 baud UART)
  │  ARROW detection: x_head, y_head, x_tail, y_tail
  ▼
ESP32 firmware  →  JSON envelope v1  (115200 USB-CDC)
  ▼
huskylens_sensor ROS node  (20 Hz timer)
  │  reader.read_line() → JSON string
  │  normalize_huskylens_payload() → frame dict
  │  frame_pub.publish(String)   → /huskylens/frame
  │  valid_pub.publish(Bool)     → /huskylens/valid
  ▼
line_follower._huskylens_cb
  │  JSON parse → _last_huskylens_frame
  ▼
line_follower._timer_cb  (100 Hz)
  │  set_tracking_context({
  │    huskylens_frame: _last_huskylens_frame,
  │    huskylens_stale: age > 0.6s,
  │    line_sensor_frame: _last_frame or None,
  │  })
  │  follower.update(frame, now)
  │    STATE_FOLLOWING → _follow_line()
  │    HuskyLensStrategy.compute()
  │      tail_offset_x=-15.0 > lateral_deadband(10.0)
  │      → return ("Right", 6)
  │  cmd_pub.publish("Right:6")  → /motor_cmd
  ▼
motor_driver
  │  serial.write(b"(-6,6,-6,6)\n")   ← Right strafe (mecanum)
  ▼
Motor MCU  →  motors
```

### Steering decision tree (`_compute_huskylens_command`)

```
context.huskylens_frame.valid == 1 && connected == 1 && algorithm_set == 1
    │
    ▼
tail_offset_x:
    < -lateral_deadband (10.0)   →  Right (strafe phải)
    >  lateral_deadband (10.0)   →  Left (strafe trái)
    │
    ▼ trong band lateral
angle_deg:
    < -heading_deadband (3.0)   →  RotateRight
    >  heading_deadband (3.0)   →  RotateLeft
    │
    ▼ trong cả 2 band
Forward (base_speed=8)
```

### Stale frame protection

```
T=0    │ HuskyLens emit frame OK
T+0.6s │ ESP32 firmware crash, không emit nữa
T+0.6s │ line_follower._timer_cb:
       │   now - _last_huskylens_ts = 0.6s
       │   huskylens_stale = True
       │ HuskyLensStrategy.compute → None (stale check)
       │ _follow_line:
       │   strategy returned None
       │   strict_mode = True (config default)
       │   → state = STATE_STOPPED
       │   → return ("Stop", 0)
       │   → log warn "strict_mode stop: strategy=huskylens no valid input"
T+0.6s │ Robot dừng an toàn
T+8s   │ ESP32 reset (WDT firmware), bắt đầu emit lại
T+8s   │ huskylens_stale = False, valid=1
T+8s   │ Tự động tiếp tục (chưa, vì state vẫn STOPPED)
       │ → Cần toggle auto_mode hoặc plan_select lại để reset state
```

---

## 5. Scenario D: Cross detection

Plan a19 có step `RotateRight, until=line` sau cross. Robot đang follow line,
gặp giao cắt (line đen trên cả 3 zone).

### Line sensor frame snapshot

```
Trước cross:        Tại cross:           Sau cross:
[0,3,0,F,T,F]       [4,4,4,T,T,T]        [0,3,0,F,T,F]
   ↑                   ↑                    ↑
   line giữa           cross detected       quay lại line
```

Format: `[left_count, mid_count, right_count, left_full, mid_full, right_full]`

### FSM transition

```
STATE_FOLLOWING (đang dùng HuskyLens hoặc line_sensor)
    │
    │ frame: left_full=T, mid_full=T, right_full=T
    │ cross_detected = True
    │ cross_event = True (chưa active trước đó)
    │
    ▼
Check cross_plan tồn tại && _should_skip_cross_pre()
    │
    │ Nếu next actionable step là RotateLeft/RotateRight
    │ và NOT in autoline_mode:
    │   → skip cross_pre, _start_plan_action() ngay
    │
    │ Else (default):
    ▼
STATE_CROSS_PRE phase 0
    │ _cross_pre_until = now + cross_pre_forward_duration (2.7s)
    │ return ("Forward", cross_pre_forward_speed=8)
    │
    │ ... loop next tick (10ms timer) ...
    │ now < _cross_pre_until → return ("Forward", 8)
    │
    ▼
now >= _cross_pre_until (2.7s đã qua)
    │ phase = 1
    │ _cross_pre_until = now + cross_pre_stop_duration (1.0s)
    │ return ("Stop", 0)
    │
    │ ... loop tick ...
    │ now < _cross_pre_until → return ("Stop", 0)
    │
    ▼
now >= _cross_pre_until (1.0s đã qua)
    │ state = STATE_FOLLOWING
    │ phase = 0
    │ → _start_plan_action(now)
    │
    ▼
_start_plan_action:
    step = cross_plan[_plan_index]
    _plan_index += 1
    action = "RotateRight", until="line", speed=5, min_duration=20

    state = STATE_PLAN
    _plan_action = "RotateRight"
    _plan_action_speed = 5
    _plan_action_until_line = True
    _plan_action_min_until = now + 20  ← rotate ít nhất 20s
    _plan_action_timeout = None (không có timeout config)

    _queue_step_messages(step)  ← messages publish ở next tick
    return ("RotateRight", 5)
```

### Plan messages

Step có `messages: [{"topic": "/plan_message", "message": "say: ..."}]`:

```
line_follower._timer_cb tick sau khi _start_plan_action:
    follower.consume_requested_step_messages() → [{"topic":..., "message":...}]
    for entry in entries:
        topic = "/plan_message"
        message = "say: Con là robot..."
        pub = _get_string_publisher(topic)
        pub.publish(String(message))

        if topic == "/plan_status":
            log [PLAN_STATUS_MQTT]
        log [PLAN_MSG] topic=/plan_message payload="say: Con là..."

mqtt_bridge._plan_message_cb (subscriber /plan_message):
    payload = "say: Con là robot..."
    client.publish("plan_message", payload)  ← MQTT topic

App / speaker subscribe MQTT "plan_message":
    Phát TTS / hiện UI
```

### Rotate-until-line completion

```
STATE_PLAN, _plan_action = "RotateRight"
    │ tick @ 100 Hz
    │ now < _plan_action_min_until (20s) → return ("RotateRight", 5)
    │
    │ ... rotate liên tục ...
    │
    ▼ now >= _plan_action_min_until
    │ frame được cập nhật (line vẫn ở dưới)
    │ _is_line_reacquired(frame, "RotateRight"):
    │   strict_center=False, allow_side_stop=True
    │   if _is_centered(frame): return True
    │   else if right_count >= rotate_line_side_min_count (1): return True
    │
    │ → giả sử centered → return True
    ▼
_clear_plan_action_state()
_after_plan_action(now):
    _plan_index < len(cross_plan):
        if _next_action_is_autoline():
            → _start_plan_action(now)  ← chạy step kế ngay
        else:
            state = STATE_FOLLOWING
            return _follow_default() → ("Forward", base_speed)
```

---

## 6. Scenario E: Face detection

Camera nhận diện gương mặt ID=1 (đã learn trước). Bridge trigger TTS message
qua MQTT.

### Sequence

```
ESP32 camera firmware (camera_sensor/device_code/main.py):
    hl.get_blocks() → [Block(ID=1, x=160, y=120, ...)]
    emit("data", {"kind": "face", "ids": [1]})
    serial → JSON envelope v1

camera_sensor ROS node:
    reader.read_line() → JSON
    _normalize_face_payload(json) → "<DEV1,FACE,1>"
    pub.publish(String("<DEV1,FACE,1>"))   ← /face/camera

mqtt_bridge._camera_cb(String("<DEV1,FACE,1>")):
    payload = "<DEV1,FACE,1>"

    if _camera_face_forward_enabled:
        client.publish("face/camera", payload)  ← MQTT raw

    _publish_camera_plan_message(payload):
        require_plan_autoline check:
            _plan_autoline_active && _active_plan_name

        face_ids = extract_face_ids(payload) → [1]
        for face_id in face_ids:
            message = _camera_face_plan_messages.get("1")
            # vd "say: Con kính chào bác Chiến..."
            client.publish("plan_message", message)
            log "Camera FACE id=1 → MQTT plan_message: say: ..."

App speaker subscribes "plan_message" → TTS
```

### Gate behavior

`require_plan_autoline` = True (config default):

| Scenario                       | Plan active? | autoline? | Face message? |
| ------------------------------ | :----------: | :-------: | :-----------: |
| Robot idle (no plan)           |      ❌      |    ❌     |    **NO**     |
| Plan selected nhưng chưa start |      ✅      |    ❌     |    **NO**     |
| Plan đang AutoLine step        |      ✅      |    ✅     |    **YES**    |
| Plan đang Rotate step          |      ✅      |    ❌     |    **NO**     |
| Plan completed                 |      ❌      |    ❌     |    **NO**     |

Lý do: tránh phát TTS chào khi robot đang rotate/stop không phù hợp ngữ cảnh.

`_update_plan_runtime_state` cập nhật state này khi receive `/plan_status`:

```python
event "selected"        → _active_plan_name = name
event "autoline_step"   → _plan_autoline_active = (autoline payload field)
event "cleared"         → reset cả 2
event "completed_reset" → reset cả 2
```

---

## 7. Plan execution lifecycle

### 7.1 Selection phase

```
T=0     │ /plan_select "a19"
T+0     │ _plan_cb:
        │   ConfigManager.load_plan("a19") → plan_data
        │   follower.set_plan(steps, end_state)
        │   _reset_state() → _plan_index = 0
        │   set_autoline_mode(True) (theo plan.autoline)
T+0     │ pick_pub.publish("1") + _set_auto_mode(True)
        │   autoMode = True
        │   follower.reset()
T+0     │ if start_without_cross:
        │   request_plan_start() → _plan_start_requested = True
T+0     │ publish PLAN_STATUS "selected", "autoline_enabled", "triggered_without_cross"
```

### 7.2 Step dispatch

Mỗi tick (100 Hz), `_timer_cb`:

```python
if not autoMode:
    _check_and_publish_plan_completed()
    return

now = time.time()
set_tracking_context({...})  # frame, stale flags

result = follower.update(frame, now)

if result is None:
    return    # FSM kẹt vô lý? (đã có guard High-1)

direction, speed = result
cmd_pub.publish(...)
_log_plan_status(now)
_check_and_publish_plan_completed()
```

### 7.3 Action types

| Action                        | Behavior                                             | Min/Max duration               |
| ----------------------------- | ---------------------------------------------------- | ------------------------------ |
| `Forward/Backward/Left/Right` | Strafe trong `duration`, default 0.5s                | ≥ 0.5s                         |
| `RotateLeft/RotateRight`      | 2 mode: timed (`duration`) hoặc until-line (default) | min 20s default cho until-line |
| `Follow`                      | Auto follow line trong duration, default 0.6s        | ≥ 0.6s                         |
| `Wait`                        | Stop trong duration, default 0.3s                    | ≥ 0.3s                         |
| `Stop`                        | Có duration → tạm dừng; không duration → end plan    | —                              |
| `AutoLine`                    | Toggle auto mode (`enabled`), immediate transition   | 0                              |
| `Goto`                        | Nhảy đến `target` (int index hoặc label string)      | 0                              |
| `Label`                       | Marker, skip                                         | 0                              |

### 7.4 Until-conditions

```python
# until="line" (default cho rotate không có duration)
if frame_centered or (allow_side_stop and side_count >= 1):
    → complete step

# until="y_type" (huskylens mode)
if huskylens_y_type() == Y_TYPE_MID_TO_TOP:
    → complete step

# Safety timeout
if _plan_action_timeout and now >= _plan_action_timeout:
    → log warn "rotate timeout reached"
    → complete step
```

### 7.5 Inter-step transition

`_after_plan_action(now)` sau khi step complete:

```
if _plan_force_complete (step có end_state=true):
    _plan_index = len(cross_plan)  ← jump to end

if _plan_index >= len(cross_plan):
    log "[PLAN] Completed all steps"
    if plan_end_state == "follow":
        return _follow_default()
    else:
        enter STATE_STOPPED
        return ("Stop", 0)

if _plan_continue_immediate or _next_action_is_autoline():
    → _start_plan_action(now)   ← chạy step kế ngay
else:
    state = STATE_FOLLOWING
    return ("Forward", base_speed)  ← follow line cho đến cross kế
```

### 7.6 Completion

`_check_and_publish_plan_completed`:

```python
if _active_plan_name and not _plan_completion_reported:
    status = follower.get_plan_status()
    if status["plan_done"]:
        publish PLAN_STATUS "completed"
        if end_state != "follow":
            _set_auto_mode(False)
            follower.clear_plan()
            pick_pub.publish("0")
            _active_plan_name = None
            publish PLAN_STATUS "completed_reset"
        log "Plan completed and reset: {name}"
```

App/UI nhận MQTT `plan_status` event `completed_reset` → biết robot
sẵn sàng nhận plan mới.

---

## 8. Recovery scenarios

### 8.1 Serial disconnect (sensor)

```
T=0     │ ESP32 line_sensor hoạt động bình thường
T=5s    │ User rút cáp USB
T=5s    │ LineSensorReader.read_frame:
        │   self.ser.read() raise OSError
        │   _log_error("Line Sensors read error: ...")
        │   self.close()  ← self.ser = None
T=5s    │ read_frame trả None mỗi tick
T=5s    │ /line_sensors/frame ngừng publish
T=5s    │ line_follower _last_frame_ts không update
T=5.4s  │ now - _last_frame_ts > line_frame_stale_sec (0.4s)
        │ line_stale = True
        │ → strategy không có line sensor data
        │ → nếu huskylens vẫn OK: tiếp tục tracking bằng huskylens
        │ → nếu huskylens cũng stale: strict_mode stop

T=7s    │ User cắm lại
T=7s    │ udev fire add event
T=7s    │ systemctl restart hospitalrobot-serial
T=7.2s  │ init_serial_symlinks chạy probe
T=10s   │ Symlink /dev/hospitalrobot/line tái tạo
T=10s   │ Lần next reconnect_timer (2s period):
        │   _reconnect_cb → reader.reconnect()
        │   → open /dev/hospitalrobot/line OK
        │   → log "Line sensor serial reconnected"
T=10s   │ Frame publish tiếp tục
T=10s   │ line_follower nhận frame, line_stale = False
        │ → tracking resume
```

### 8.2 MQTT broker crash

```
T=0     │ mosquitto chạy bình thường, bridge connected
T=10s   │ systemctl stop mosquitto
T=10s   │ paho client phát hiện TCP RST
T=10s   │ on_disconnect callback:
        │   _mqtt_connected.clear()
        │   log "Disconnected from MQTT broker"
T=10s   │ Bridge vẫn chạy:
        │   _camera_cb, _plan_status_cb, ... check _mqtt_connected.is_set()
        │   → False → skip publish (log throttled)
T=10s   │ ROS side không bị ảnh hưởng (subscribers vẫn nhận ROS topics)

T=15s   │ systemctl start mosquitto
T=16s   │ paho auto-reconnect (reconnect_delay_set min=1, max=10)
T=16s   │ on_connect fires:
        │   subscribe(VR_control, pick_robot, plan_select)
        │   _mqtt_connected.set()
        │   log "Connected to MQTT broker"
T=16s   │ Bridge resume forward ROS → MQTT
```

### 8.3 Plan execution lỗi giữa chừng

```
Đang ở step "RotateRight until line"
T=0     │ Rotate bắt đầu
T+5s    │ Robot bị ai đó chặn line bằng vật cản
T+5s    │ frame mất line: total_black = 0
T+5s    │ FSM trong STATE_PLAN, đang đợi line reacquire
T+5s    │ now < _plan_action_min_until (20s) → tiếp tục rotate
T+20s   │ min_duration đã qua
        │ _is_line_reacquired(frame, "RotateRight") → False (no line)
        │ → tiếp tục rotate
T+25s   │ vẫn không có line
        │ Nếu không có timeout config → rotate vô hạn (BUG tiềm năng!)
        │ Nếu có timeout config (vd 30s):
T+30s   │ now >= _plan_action_timeout
        │ log warn "Plan rotate timeout reached: RotateRight"
        │ _clear_plan_action_state()
        │ _after_plan_action → next step or end plan
```

**Khuyến nghị plan author:** Mọi rotate-until-line nên có `timeout` field
để tránh rotate vô hạn (xem [`10-plan-authoring.md`](10-plan-authoring.md)
§7).

### 8.4 Motor MCU không phản hồi

```
T=0     │ motor_driver gửi (8,8,8,8)\n
T=0     │ MCU không acknowledge (firmware crash hoặc cáp lỏng)
T=0     │ serial.write thành công (kernel buffer)
T=0     │ motor.move trả wheel_speeds OK
T=0     │ /motor_cmd tiếp tục được publish
T=0     │ ROS không biết MCU đã chết
T+30s   │ Operator nhận ra robot không di chuyển
T+30s   │ Kiểm tra: ros2 topic echo /motor_cmd
        │   → vẫn có message → ROS side OK
        │ Kiểm tra: cat /dev/hospitalrobot/motor
        │   → nếu MCU emit periodic status → check timing
        │ Reset MCU vật lý
```

**Gap:** Hệ thống không có ACK loop motor → ROS để detect MCU dead. Đây là
known limitation; mở rộng tương lai có thể thêm `event=ack` từ MCU và
watchdog ở motor_driver_node.

### 8.5 Lost line during plan (line_sensor mode)

```
STATE_FOLLOWING, đang follow line_sensor
frame total_black = 0
    │
    ▼
_has_pending_plan() == True
    │
    ▼
_hold_stop_for_plan_lost_line(now):
    if _plan_lost_line_since is None:
        _plan_lost_line_since = now
    elapsed = now - _plan_lost_line_since

    if elapsed < plan_lost_line_hold_sec (1.2s):
        return  # vẫn trong window grace

    # Đã quá hold time, log warn periodically
    if (now - _last_plan_lost_line_warn_ts) >= plan_lost_line_warn_period:
        log warn "LOST LINE during active plan; hold STOP and wait line reacquire"
        _last_plan_lost_line_warn_ts = now

state = STATE_FOLLOWING
return ("Stop", 0)
```

Robot dừng nhưng plan progress **không bị reset**. Khi line quay lại
(`total_black > 0`), `_plan_lost_line_since = None` và tracking resume.

---

## 9. Priority & arbitration

### 9.1 Manual vs Auto

```
/VR_control + /pick_robot là input từ MQTT/keyboard
/auto_mode               là state derived

manual_control xử lý:
    /VR_control → cmd_vel (luôn pass-through)
        nếu autoMode = True và manual_override_on_input = True:
            → tắt auto trước khi pass-through

    /pick_robot → set autoMode + publish /auto_mode

line_follower nhận /auto_mode:
    autoMode → control vòng lặp tracking
    autoMode=False → publish Stop, không tracking
```

Quy tắc: **Manual input luôn win**. Operator có thể dừng auto bất cứ lúc nào
bằng cách nhấn phím hoặc publish VR_control.

### 9.2 Service vs Topic for auto_mode

Có 2 cơ chế đổi auto mode:

| Cơ chế                             | Caller                                                                | Effect        |
| ---------------------------------- | --------------------------------------------------------------------- | ------------- |
| Topic `/auto_mode` (Bool)          | `manual_control` publish, `line_follower._auto_cb` nhận               | Direct, async |
| Service `/set_auto_mode` (SetBool) | `manual_control.auto_client` call, `line_follower._auto_srv_cb` xử lý | Sync, có ack  |

Topic chính: trạng thái UI realtime.
Service phụ: đảm bảo `line_follower` đã nhận và xác nhận (`AutoModeSync` retry).

### 9.3 Plan-driven auto toggle

`AutoLine` step trong plan có thể bật/tắt auto giữa chừng:

```
Step: {"action": "AutoLine", "enabled": false}
    │
    ▼
follower.consume_requested_autoline() → False (sau step execute)
    │
    ▼
_consume_autoline_action(False):
    set_autoline_mode(False)
    publish PLAN_STATUS "autoline_step"
    pick_pub.publish("0")
    _set_auto_mode(False)

Robot dừng tracking, nhưng plan vẫn tiếp tục step kế (vì _plan_continue_immediate)
```

Use case: plan có giai đoạn "chỉ rotate, không tracking" — đến cross thì
tắt autoline để FSM không bị line sensor lái sai.

### 9.4 Strategy fallback priority

```
config.tracking.strategy = "hybrid"
    │
    ▼ tick
HuskyLensStrategy.compute(context):
    if huskylens valid → return command
    if huskylens invalid → return None
    │
    ▼ command is None
LineSensorStrategy.compute(context):
    if line_sensor frame có data → return command
    if frame stale/missing → return None
    │
    ▼ vẫn None
strict_mode? → STOP
tracking_allow_line_sensor_fallback? → fallback
otherwise → STOP
```

---

## 10. Timing budget

### 10.1 Critical path: HuskyLens → motor

```
HuskyLens IC capture + AI                       ~50-80 ms
   ↓ UART 9600 baud, ARROW message
ESP32 read + parse + emit JSON                  ~5-10 ms
   ↓ USB-CDC 115200
Linux USB stack + tty driver                    ~1-5 ms
   ↓
huskylens_sensor.read_line + normalize          ~1-2 ms
   ↓ /huskylens/frame ROS topic
line_follower._huskylens_cb (JSON parse)        ~1 ms
   ↓
line_follower._timer_cb @ 100 Hz                up to 10 ms wait
   ↓ FSM compute
cmd_pub.publish("Right:6")                       <1 ms
   ↓ /motor_cmd ROS topic
motor_driver._cmd_cb                             ~1 ms
   ↓
parse_command + motor.move + serial.write        ~1-2 ms
   ↓ USB-CDC
Motor MCU receive + parse                        ~1-5 ms
   ↓
Motor PWM update                                 ~1 ms

Total end-to-end: ~70-120 ms
```

Bottleneck: HuskyLens IC inference (~50-80ms), không thể giảm thêm.

### 10.2 Loop rates (timer-driven)

| Component                                 | Period |  Hz | Purpose                   |
| ----------------------------------------- | -----: | --: | ------------------------- |
| Firmware huskylens loop                   | 100 ms |  10 | Emit data envelope        |
| Firmware line loop                        | 100 ms |  10 | Emit sensors envelope     |
| Firmware camera loop                      | 100 ms |  10 | Emit face envelope        |
| `huskylens_sensor._timer_cb`              |  50 ms |  20 | Read serial + publish     |
| `line_sensor_driver._timer_cb`            |  10 ms | 100 | Read serial + publish     |
| `camera_sensor._timer_cb`                 |  33 ms |  30 | Read serial + publish     |
| `line_follower._timer_cb`                 |  10 ms | 100 | FSM compute + publish cmd |
| `motor_driver` reconnect_timer            |     2s | 0.5 | Check + reconnect serial  |
| `mqtt_bridge._drain_inbound_queue`        |  50 ms |  20 | Drain MQTT inbound queue  |
| `manual_control._sync_auto_mode_timer_cb` | 200 ms |   5 | Retry service call        |

### 10.3 Watchdog & timeouts

| Watchdog                          |             Value | Component                      |
| --------------------------------- | ----------------: | ------------------------------ |
| HuskyLens firmware WDT            |           3000 ms | Hardware reset nếu loop hang   |
| `huskylens_frame_stale_sec`       |            600 ms | FSM coi frame là stale         |
| `line_frame_stale_sec`            |            400 ms | FSM coi frame là stale         |
| `plan_lost_line_hold_sec`         |           1200 ms | Grace trước khi log warn       |
| Plan rotate `min_duration`        | 5–20 s (per step) | Chống stop sớm do nhiễu        |
| Plan rotate `timeout`             |      configurable | Chống rotate vô hạn            |
| `huskylens_y_type_rotate_timeout` |           5000 ms | Safety cho rotate-until-y_type |
| MQTT `reconnect_delay_set`        |            1–10 s | Exponential backoff            |
| Serial `timeout` (read)           |        100–200 ms | pyserial blocking timeout      |

---

## 11. Tham chiếu code cho mỗi scenario

| Scenario             | File chính                                                     | Line |
| -------------------- | -------------------------------------------------------------- | ---: |
| Boot probe           | `scripts/init_serial_symlinks.py`                              |  107 |
| MQTT plan select     | `mqtt_bridge/MQTTBridgeROS.py:_resolve_plan_command`           |  409 |
| Plan name validation | `robot_common/config_manager.py:load_plan`                     |   64 |
| Manual control       | `manual_control/manual_control_node.py:_manual_cb`             |   47 |
| HuskyLens tracking   | `line_follower/line_follower.py:_compute_huskylens_command`    |  663 |
| Cross detection      | `line_follower/line_follower.py:update`                        |  152 |
| Cross_pre FSM        | `line_follower/line_follower.py:_handle_cross_pre`             |  265 |
| Plan dispatch        | `line_follower/line_follower.py:_dispatch_plan_action`         |  342 |
| Face → plan_message  | `mqtt_bridge/MQTTBridgeROS.py:_publish_camera_plan_message`    |  352 |
| Echo suppression     | `mqtt_bridge/MQTTBridgeROS.py:_mark_local_publish`             |  445 |
| Strategy fallback    | `line_follower/line_follower.py:_follow_line`                  |  568 |
| Motor wire format    | `motor_driver/motor_controller.py:move`                        |   20 |
| Lost line policy     | `line_follower/line_follower.py:_hold_stop_for_plan_lost_line` |  867 |

---

## 12. Debug recipe cho từng scenario

### Scenario A (MQTT plan select không có hiệu lực)

```bash
# 1. Check broker
mosquitto_sub -h 127.0.0.1 -t plan_select -v
# Publish from another terminal
mosquitto_pub -h 127.0.0.1 -t plan_select -m "room:1"
# Phải thấy echo

# 2. Check bridge nhận
ros2 topic echo /plan_select

# 3. Check follower nhận
ros2 topic echo /plan_status
# Phải thấy event="selected"

# 4. Nếu không thấy "selected" → check plan validation
journalctl -u hospitalrobot-* | grep "plan name rejected"
```

### Scenario B (manual không có hiệu lực)

```bash
# 1. Check VR_control publish
ros2 topic echo /VR_control

# 2. Check motor_cmd output
ros2 topic echo /motor_cmd

# 3. Check serial gửi xuống MCU
sudo systemctl stop hospitalrobot-serial
PORT=/dev/hospitalrobot/motor
stty -F $PORT 115200 raw -echo
timeout 3 cat $PORT
# Phải thấy bytes (N,N,N,N) khi gửi command
```

### Scenario C (HuskyLens không track)

```bash
# 1. Check firmware emit
ros2 topic echo /huskylens/frame --once
# Phải có connected=1, algorithm_set=1, valid=1

# 2. Check FSM nhận
ros2 topic echo /plan_status --once
# state="FOLLOWING" hoặc "PLAN"

# 3. Check tracking strategy log
journalctl -u hospitalrobot-* | grep -E "STRATEGY|TRACKING|FALLBACK"
```

### Scenario D (plan kẹt giữa chừng)

```bash
# 1. Check current step
ros2 topic echo /plan_status
# state, current_action, next_step, total_steps

# 2. Check FSM internal
journalctl | grep "\[PLAN\]"
# Sequence các step đã pass

# 3. Force exit plan
mosquitto_pub -h 127.0.0.1 -t plan_select -m "clear"
```

---

## Phụ lục: Glossary

| Thuật ngữ                | Định nghĩa                                                               |
| ------------------------ | ------------------------------------------------------------------------ |
| **Cross**                | Giao cắt line, detect khi line đen phủ kín 3 zone của line sensor        |
| **Cross_pre**            | Phase chuẩn bị trước khi thực thi plan action sau cross (forward → stop) |
| **AutoLine**             | Trạng thái auto-mode dynamically bật/tắt trong plan                      |
| **Y-type**               | Phân loại vị trí line theo trục Y trong khung HuskyLens (0-4)            |
| **Until-line**           | Điều kiện dừng rotate khi line tái hiện ở giữa hoặc bên đúng             |
| **Until-y_type**         | Điều kiện dừng rotate khi HuskyLens y_type = MID_TO_TOP                  |
| **Tail offset**          | Khoảng cách (px) từ tail của arrow đến tâm ảnh (signed)                  |
| **Stale frame**          | Frame cũ hơn `*_stale_sec` config → coi như invalid                      |
| **Lost line**            | Tất cả zone line sensor đều 0 (không thấy line đen)                      |
| **Strict mode**          | Khi strategy invalid, dừng ngay thay vì fallback                         |
| **Echo suppress**        | Bỏ qua message MQTT inbound trùng với local publish gần đây              |
| **Y_TYPE_BOTTOM_TO_MID** | Tail BOT, head MID → line ngắn gần robot (cross signal)                  |
| **Y_TYPE_MID_TO_TOP**    | Tail MID, head TOP → line dài phía trước (follow target)                 |
