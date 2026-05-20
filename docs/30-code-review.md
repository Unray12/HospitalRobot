# HospitalRobot — Đánh giá Code Chi Tiết

**Ngày review:** 2026-05-19
**Branch:** `feature/dev-20260514`
**Phạm vi:** Toàn bộ 9 ROS2 package + 3 firmware MicroPython + scripts setup
**Phương pháp:** Đọc trực tiếp source code, không chạy runtime test

---

## 0. Tổng quan

Kiến trúc tổng thể **mạch lạc và đúng nguyên tắc**: tách lớp `robot_common` (config, protocol, serial, logging) làm hạ tầng dùng chung, mỗi loại sensor có 1 ROS2 package riêng, `line_follower` chứa FSM thuần Python để dễ test, `mqtt_bridge` đóng vai trò edge giữa MQTT và ROS2. Firmware MicroPython tuân theo Device Protocol v1 (JSON envelope) — một quyết định đúng đắn, giúp host-side parsing thống nhất.

**Điểm mạnh nổi bật:**

- `robot_common/device_protocol.py` & `serial_device.py`: refactor tốt, loại ~300 LOC duplicate.
- `LineFollowerFSM`: tách hoàn toàn khỏi rclpy, test được pure Python (16 test pass theo `test_huskylens_integration.py`).
- Firmware HuskyLens (`huskylens_sensor/device_code/main.py`): có watchdog, deadline-driven loop, boot banner lặp lại 10 lần — production-grade.
- Logging có cấu trúc với `LogAdapter` + structured fields.
- Plan system data-driven qua JSON, hỗ trợ LABEL/GOTO/messages.

**Điểm cần cải thiện chính:**

- Bug wire format ở `MotorController.move` (xem [Critical-1](#critical-1)).
- Path traversal qua plan name từ MQTT (xem [High-2](#high-2)).
- `MQTTBridgeNode` thiếu reconnect logic khi broker chết lúc startup.
- Một số node thiếu `try/finally` cleanup trong `main()`.
- Coverage test còn nhiều khoảng trống ở các điểm boundary (motor, serial reopen, MQTT bridge end-to-end).

---

## 1. CRITICAL — phải sửa trước khi deploy

### Critical-1: `MotorController.move` sai wire format

**File:** `motor_driver/motor_driver/motor_controller.py:25-29`

```python
cmd = ",".join(f"{v}" for v in wheel_speeds) + "\n"
if self.ser and self.ser.is_open:
    try:
        self.ser.write(f"({cmd})".encode())
```

Khi `speed=8`, `wheel_speeds = [-8, -8, -8, -8]`:

- `cmd = "-8,-8,-8,-8\n"`
- `f"({cmd})" = "(-8,-8,-8,-8\n)"`

Frame được gửi xuống MCU là `(-8,-8,-8,-8\n)` — dấu `)` nằm **sau** newline. Nếu firmware motor parse dòng kết thúc bằng `\n`, parser sẽ thấy `(-8,-8,-8,-8` (thiếu `)`) trong frame đầu, và `)` đứng đầu frame sau. Gần như chắc chắn không khớp protocol.

Thêm nữa, `wheel_speeds = [s * speed for s in DIR[direction]]`. Vì `speed` là `float` (đến từ `parse_command` rồi qua route ROS), kết quả thường là float (`-8.0,-8.0,...`) — nếu firmware kỳ vọng integer cũng có thể parse fail.

**Đề xuất:**

```python
wheel_speeds = [int(round(s * speed)) for s in DIR[direction]]
cmd = "(" + ",".join(str(v) for v in wheel_speeds) + ")\n"
```

**Mức ảnh hưởng:** Robot có thể không di chuyển hoặc di chuyển sai. Không có test cho hàm này nên bug không bị phát hiện.

---

### Critical-2: Serial write fail bị nuốt im lặng

**File:** `motor_driver/motor_driver/motor_controller.py:30-32`

```python
except Exception as exc:
    self._log_error(f"Serial write error: {exc}")
    self.close()
return wheel_speeds  # vẫn return non-None
```

`return wheel_speeds` luôn trả non-None kể cả khi write fail. Caller `_cmd_cb` (motor_driver_node.py:53) chỉ check `if wheel_speeds is not None` rồi log debug — không biết lệnh bị drop. Reconnect timer sẽ heal sau 2s nhưng trong khoảng đó **command bị mất im lặng**.

**Đề xuất:** trả `None` khi write fail; caller log warning ngay (không phụ thuộc debug toggle).

---

### Critical-3: MQTT bridge không reconnect ở startup

**File:** `mqtt_bridge/mqtt_bridge/MQTTBridgeROS.py:122-127`

```python
self.client.connect(self.broker_address, self.broker_port, 60)

self._stop_event = Event()
self._mqtt_thread = threading.Thread(target=self.client.loop_forever, daemon=True)
self._mqtt_thread.start()
```

`client.connect()` raise nếu broker không có sẵn (e.g. mosquitto chưa start). `loop_forever()` có auto-reconnect _runtime_ nhưng initial `connect()` thì không — node sẽ crash.

Với hospital robot, nếu Pi boot trước khi `mosquitto.service` ready, `mqtt_bridge` chết và keyboard control mất luôn.

**Đề xuất:**

```python
try:
    self.client.connect_async(self.broker_address, self.broker_port, 60)
except Exception as exc:
    self.log.warning(f"MQTT initial connect failed, will retry: {exc}")
self.client.reconnect_delay_set(min_delay=1, max_delay=10)
self.client.loop_start()  # không cần thread thủ công
```

---

## 2. HIGH — sửa trong sprint này

### High-1: `_handle_cross_pre` có thể trả `None`

**File:** `line_follower/line_follower/line_follower.py:265-285`

Hai nhánh `if self._cross_pre_phase == 0` và `if self._cross_pre_phase == 1`, không có `else` cuối hàm. Nếu `_cross_pre_phase` bị set sai (ví dụ qua re-entry partial reset), function return `None` → FSM kẹt vĩnh viễn ở `STATE_CROSS_PRE` vì `_timer_cb` chỉ skip publish khi `result is None`.

**Đề xuất:** thêm guard cuối:

```python
self._log_warn(f"cross_pre invalid phase {self._cross_pre_phase}; reset")
self._cross_pre_phase = 0
self._cross_pre_until = None
self.state = self.STATE_FOLLOWING
return None
```

---

### High-2: Path traversal qua plan name từ MQTT

**File:** `line_follower/line_follower/line_follower_node.py:247-256` + `robot_common/robot_common/config_manager.py:64-77`

```python
# line_follower_node.py:255
cfg_mgr = ConfigManager("line_follower", logger=self.log)
plan_data = cfg_mgr.load_plan(name)  # name từ msg.data
```

```python
# config_manager.py:70
path = self._resource_path("plans", f"{plan_name}.json")
```

`joinpath` **không normalize** dấu `..`. Nếu MQTT gửi `plan_select` với payload `../config`, đường dẫn resolve thành `robot_common/plans/../config.yaml` = `robot_common/config.yaml`. File sẽ được load (vì exception trong try block chỉ catch JSON / FileNotFound, không catch path escape).

`MQTTBridgeROS._resolve_plan_command:423` có regex `r"^(room|phong|plan)\s*[:/_-]?\s*([a-z0-9_-]+)$"` nhưng chỉ áp dụng cho input dạng `room:X`. Input thẳng (ví dụ `plan_select`-direct via ROS topic, hoặc qua `MQTTBridgeROS:443` `return text` ở cuối `_resolve_plan_command`) **không qua regex sanitize**.

**Đề xuất:** validate ngay đầu `_plan_cb` và đầu `ConfigManager.load_plan`:

```python
import re
if not re.fullmatch(r"[A-Za-z0-9_-]+", plan_name):
    self.log.warning(f"Reject invalid plan name: {plan_name!r}", event="PLAN")
    return
```

**Đánh giá rủi ro:** Trong setup hiện tại (localhost-only MQTT, không auth), tấn công cần access mạng nội bộ. Tuy nhiên nếu broker bind public về sau, đây là vector RCE-adjacent (đọc tùy ý file JSON nếu có cấu trúc khớp).

---

### High-3: `_recent_local_pub` rò rỉ memory

**File:** `mqtt_bridge/mqtt_bridge/MQTTBridgeROS.py:445-457`

```python
def _mark_local_publish(self, topic: str, payload: str):
    self._recent_local_pub[(topic, payload)] = time.time()

def _is_recent_local_echo(self, topic: str, payload: str):
    ...
    if (now - ts) <= self._echo_suppress_window_sec:
        return True
    del self._recent_local_pub[key]  # chỉ xoá khi *lần sau* check thấy stale
    return False
```

Entries chỉ bị xóa khi `_is_recent_local_echo` được gọi với cùng key trong cửa sổ timeout đã hết. Local publish nào không có echo trả về (ví dụ broker drop, hoặc topic không subscribed) sẽ **ở mãi**. Trong 24h uptime với ~10 command/phút, dict có thể tích lũy hàng nghìn entry — không nghiêm trọng nhưng là leak thực sự.

**Đề xuất:** prune trên timer:

```python
def _prune_local_pub(self, now=None):
    now = now or time.time()
    cutoff = now - self._echo_suppress_window_sec
    self._recent_local_pub = {k: v for k, v in self._recent_local_pub.items() if v >= cutoff}
```

Gọi trong `_drain_inbound_queue` mỗi 100 lần hoặc trong timer riêng.

---

### High-4: `parse_command` chấp nhận separator nhập nhằng

**File:** `robot_common/robot_common/command_protocol.py:41-48`

```python
for sep in (":", ",", " "):
    if sep in raw:
        parts = [part.strip() for part in raw.split(sep) if part.strip()]
        if parts:
            direction = parts[0]
        if len(parts) > 1:
            speed = _coerce_speed(parts[1])
        break
```

- `"Forward:5,10"` parse thành `("Forward", 5)` — drop `,10` im lặng (vì `:` match trước, break ngay).
- `"Forward, 5, 10"` cũng drop phần thứ 3.
- Asymmetric với `format_command` (chỉ emit `Direction:Speed`).

**Đề xuất:** chỉ chấp nhận `:` chính thức, log warning với input lạ. Hoặc nếu muốn giữ backward-compat, validate `len(parts) <= 2` và reject nếu vượt.

---

### High-5: `_compute_line_sensor_command` raise `KeyError` với frame thiếu key

**File:** `line_follower/line_follower/line_follower.py:621-626`

```python
left_count = frame["left_count"]
mid_count = frame["mid_count"]
right_count = frame["right_count"]
left_full = frame["left_full"]
mid_full = frame["mid_full"]
right_full = frame["right_full"]
```

Dùng subscript thay vì `.get()`. Trên flow `_frame_cb → _last_frame` đã có validate length, nhưng `set_tracking_context` và `set_huskylens_frame` accept dict bất kỳ → test/integration code có thể truyền partial dict và crash strategy thread.

**Đề xuất:** dùng `.get("left_count", 0)` hoặc validate one-shot ở đầu function.

---

### High-6: HuskyLens `_build_frame` synthesize `error` bằng heuristic không document

**File:** `huskylens_sensor/huskylens_sensor/huskylens_parser.py:82-83`

```python
if "error" not in frame and "tail_offset_x" in frame and "angle_deg" in frame:
    frame["error"] = round(0.8 * frame["tail_offset_x"] + 0.2 * frame["angle_deg"])
```

Magic coefficient `0.8` / `0.2` không có document, không có test riêng. Field `error` ở downstream (`line_follower._huskylens_cb` không dùng, MQTT bridge cũng không) — vậy tại sao phải synthesize? Nếu xóa trường này, có gì break không?

Test `test_huskylens_contract_requires_tail_offset_and_angle_not_error_only` (line_follower/test/test_huskylens_integration.py:247) chứng minh contract đảo: frame chỉ có `error` thì FSM ignore. Nghĩa là synthesize chỉ để **tự fail validation kia** — tự mâu thuẫn.

**Đề xuất:** Xóa hai dòng synthesize. Nếu cần error code thật từ firmware, định nghĩa rõ ràng trong Device Protocol v1.

---

## 3. MEDIUM — sửa khi có thời gian

### Medium-1: Nhiều `main()` thiếu `try/finally` cleanup

**File:** `line_follower/line_follower/line_follower_node.py:515-521`, `motor_driver/motor_driver/motor_driver_node.py:90-95`, `line_sensors/line_sensors/line_sensor_driver_node.py:137-142`, `manual_control/manual_control/manual_control_node.py:105-110`

Pattern hiện tại:

```python
def main(args=None):
    rclpy.init(args=args)
    node = LineFollowerNode()
    rclpy.spin(node)        # Ctrl+C raise KeyboardInterrupt
    node.destroy_node()     # bị bypass
    rclpy.shutdown()
```

Mẫu đúng ở `huskylens_sensor_node.py:117-125`, `camera_sensor/main.py:151-161`, `MQTTBridgeROS.py:479-489`:

```python
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
finally:
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
```

**Tác động:** Serial port không close sạch khi Ctrl+C → lần restart kế tiếp có thể gặp "Resource temporarily unavailable". Note.txt đã document workaround pkill — chính là dấu hiệu của bug này.

---

### Medium-2: `SerialDevice` sleep 0.2s mỗi lần open

**File:** `robot_common/robot_common/serial_device.py:81`

```python
time.sleep(min(self.timeout, 0.2))
```

Với USB-CDC (ESP32) không cần DTR settling. Trong `reconnect()` (line 41), nếu có nhiều candidate port, mỗi lần thử open thành công đều ngủ 0.2s — cộng dồn với probe time.

**Đề xuất:** hoặc xóa hẳn, hoặc cố định 0.05s.

---

### Medium-3: `_drain_inbound_queue` chạy 50 Hz polling

**File:** `mqtt_bridge/mqtt_bridge/MQTTBridgeROS.py:140`

```python
self._bridge_timer = self.create_timer(0.02, self._drain_inbound_queue)
```

50 Hz timer chỉ để check `queue.SimpleQueue` — wakeup mỗi 20ms dù không có gì làm. Trong ROS2 có thể dùng MutuallyExclusiveCallbackGroup + condition variable, hoặc đơn giản giảm xuống 100ms (10 Hz) vì keyboard input vốn rất chậm.

**Đề xuất:** giảm period xuống 0.1 (10 Hz đủ cho keyboard) hoặc 0.05 (20 Hz).

---

### Medium-4: HuskyLens firmware lấy `arrows[0]` mà không sort

**File:** `huskylens_sensor/device_code/main.py:361-363`

```python
arrows = read_arrows(hl)
if arrows:
    line_tracking = arrow_to_line_data(arrows[0])
```

Nếu HuskyLens trả nhiều arrow trong khung hỗn loạn, `arrows[0]` là phần tử đầu — không nhất thiết là arrow đáng tin nhất. Strategy hợp lý:

1. Sort theo `line_length_y` desc (line dài hơn = tin cậy hơn).
2. Hoặc lấy arrow có `y_tail` lớn nhất (gần robot nhất).

**Đề xuất:**

```python
if arrows:
    arrows = sorted(arrows, key=lambda a: int(getattr(a, "y_tail", 0)), reverse=True)
    line_tracking = arrow_to_line_data(arrows[0])
```

---

### Medium-5: ConfigManager re-instantiate mỗi plan select

**File:** `line_follower/line_follower/line_follower_node.py:255`

```python
def _plan_cb(self, msg: String):
    ...
    cfg_mgr = ConfigManager("line_follower", logger=self.log)
    plan_data = cfg_mgr.load_plan(name)
```

`_global_cache` là class variable nên không re-read disk, nhưng tạo instance mới mỗi plan select. Cache 1 instance ở `self` trong `__init__`.

---

### Medium-6: Echo suppression theo `(topic, payload)` có thể nuốt retry hợp lệ

**File:** `mqtt_bridge/mqtt_bridge/MQTTBridgeROS.py:446-456`

`echo_suppress_window_sec = 0.35`. Nếu user spam phím cùng plan 2 lần trong 0.35s, lần thứ 2 bị suppress như echo. Edge case hiếm nhưng cần document hoặc dùng nonce/sequence number nếu nghiêm túc.

---

### Medium-7: `LineSensorReader.read_frame` discard frame cũ khi frame mới hỏng

**File:** `line_sensors/line_sensors/line_sensor_reader.py:36-43`

```python
lines = self._buffer.split("\n")
self._buffer = lines[-1]
for line in reversed(lines[:-1]):
    candidate = line.strip()
    if not candidate:
        continue
    return self.parse_line(candidate)  # luôn return ngay, kể cả khi None
```

`parse_line` return `None` cho frame malformed → `read_frame` cũng return `None` ngay, **dù các dòng cũ hơn có thể hợp lệ**. Với 100 Hz publish rate ít khi thấy, nhưng nếu firmware gửi banner xen kẽ data thì sẽ drop data hợp lệ.

**Đề xuất:**

```python
for line in reversed(lines[:-1]):
    candidate = line.strip()
    if not candidate:
        continue
    parsed = self.parse_line(candidate)
    if parsed is not None:
        return parsed
return None
```

---

### Medium-8: Plan a19.json có `"end_state": "true"` khả nghi

**File:** `robot_common/robot_common/plans/a19.yaml:70`

```json
{
    "label": "S6_RETURN_DONE",
    ...
    "end_state": "true",
    ...
}
```

`end_state` ở root plan có giá trị `"stop"` / `"follow"` (string), nhưng ở step `_to_bool(step.get("end_state"), False)` (line_follower.py:326) → `"true"` parse thành boolean True, **không phải string `"stop"`**. Hai semantic khác nhau dùng cùng key — confusing.

**Đề xuất:** đổi tên field cấp step thành `complete_plan: true` cho rõ.

---

## 4. LOW — nice to have

### Low-1: Plan→key mapping duplicate ở 3 chỗ trong `config.yaml`

**File:** `robot_common/robot_common/config.yaml:92-99, 195-201, 203-214`

`line_follower.plan_alias`, `mqtt_bridge.plan_keys`, `mqtt_bridge.room_plans` đều chứa `1→a19, 2→a18, ..., 5→a15`. Sửa một chỗ quên hai chỗ → bug khó tìm.

**Đề xuất:** đưa mapping vào section riêng `plans.aliases`, các package khác đọc tham chiếu.

### Low-2: `safe_symlink` race condition

**File:** `scripts/init_serial_symlinks.py:96-99`

```python
def safe_symlink(link: Path, target: str) -> None:
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(target)
```

TOCTOU giữa `unlink` và `symlink_to` nếu udev re-fire concurrent.

**Đề xuất:** atomic swap qua `os.symlink` + `os.replace`:

```python
import tempfile, os
tmp = link.with_suffix(f".tmp.{os.getpid()}")
os.symlink(target, tmp)
os.replace(tmp, link)
```

### Low-3: `pkill -f "ros2 launch|ros2 run"` quá rộng

**File:** `scripts/setup_serial_auto.sh:48, 95`

Match bất kỳ ROS process của user khác trên Pi. Trên hospital robot Pi chỉ có 1 user nên ít rủi ro, nhưng vẫn nên hẹp lại:

```bash
pkill -f "ros2 (launch|run) (line_sensor|camera_sensor|huskylens_sensor|motor_driver|line_follower|manual_control|mqtt_bridge|robot)"
```

### Low-4: `KeyboardInput.stop()` là no-op

**File:** `mqtt_bridge/mqtt_bridge/keyboard_input.py:62-63`

```python
def stop(self):
    pass
```

Vì `get_key()` block đến khi có ký tự. Nếu muốn join thread sạch cần `select.select([sys.stdin], [], [], 0.1)` để có timeout. Hiện tại thread keyboard chết khi process exit (daemon=True), nên không gây hại nhưng API nhìn lừa người dùng.

### Low-5: `LogAdapter` không check `isatty()` khi quyết định ANSI

**File:** `robot_common/robot_common/logging_utils.py:14-21`

```python
def _color_enabled(default=True):
    if os.getenv("NO_COLOR"):
        return False
    value = os.getenv("ROBOT_LOG_COLOR")
    if value is None:
        return default
    ...
```

Default `True`, không kiểm tra `sys.stderr.isatty()`. Khi systemd capture stderr vào journal, log dính escape `\033[36m...` xấu.

**Đề xuất:**

```python
import sys
def _color_enabled(default=True):
    if os.getenv("NO_COLOR"):
        return False
    value = os.getenv("ROBOT_LOG_COLOR")
    if value is not None:
        return value.strip().lower() not in ("0", "false", "off", "no")
    return default and sys.stderr.isatty()
```

### Low-6: MQTT broker không TLS/auth

**File:** `robot_common/robot_common/config.yaml:148-150`

```json
"broker": {"address": "127.0.0.1", "port": 1883}
```

Localhost-only thì OK, nhưng nên document trong `docs/11-mqtt-messaging.md` rằng config này **không an toàn nếu broker bind public**. Nếu sau này deploy cluster nhiều robot, cần TLS + username/password.

### Low-7: `__pycache__` còn trong git index

File `gitStatus` đầu phiên cho thấy `__pycache__/` nằm ngoài `.gitignore`. Cần thêm vào `.gitignore`:

```
__pycache__/
*.pyc
.pytest_cache/
```

### Low-8: `huskylens_sensor_node._timer_cb:67-78` set `frame["connected"]` sau khi `default_frame()`

**File:** `huskylens_sensor/huskylens_sensor/huskylens_sensor_node.py:67-72`

```python
if frame is None:
    self._parse_error_count += 1
    frame = default_frame()
    frame["connected"] = 1 if self.reader.is_connected() else 0
```

OK nhưng inconsistency: parser fail không có nghĩa serial disconnect. Frame default có `valid=0, algorithm_set=0` rồi, set `connected=1` riêng tạo trạng thái mới "connected nhưng không nhận diện" — downstream phải hiểu nuance này. Cần document trong `docs/22-huskylens-integration.md`.

---

## 5. Architecture — đề xuất refactor

### Arch-1: `LineFollowerFSM` quá lớn (990 dòng, 50+ method) — **DEFERRED**

File `line_follower/line_follower/line_follower.py` gánh vác:

- Tracking strategy dispatching (line_sensor / huskylens / hybrid).
- Cross detection FSM (FOLLOWING / CROSSING / CROSS_PRE).
- Plan executor (LABEL, GOTO, AutoLine, Rotate-until, messages).
- Sensor frame interpretation.

**Đề xuất tách:**

- `LineFollowerCore` — tracking strategies + cross detection (~400 dòng).
- `PlanExecutor` — chỉ chứa plan FSM, nhận command từ Core và `cross_event` callback (~400 dòng).
- `LineFollowerFSM` — facade compose hai class trên.

Lợi ích: test PlanExecutor riêng không cần mock frame.

**Status (2026-05-19):** Defer. Tests hiện truy cập trực tiếp `fsm._plan_index`, `fsm._cross_pre_phase`, `fsm._cross_pre_until`, `fsm._y_type_cross_active` (≥5 chỗ trong `test_line_follower_fsm.py`, `test_huskylens_integration.py`). Tách PlanExecutor cần dịch chuyển 7+ private fields và rewire tests; chỉ có test snapshot 1-tick, không có integration test full-sequence. Risk/value chưa thuyết phục — chờ khi có lý do business cụ thể (thêm tracking strategy thứ 4, hoặc plan FSM phức tạp hơn).

### Arch-2: Strategy pattern nửa vời — **KEPT AS WRAPPERS (documented)**

**File:** `line_follower/line_follower/line_follow_strategies.py`

```python
class HuskyLensStrategy(BaseLineFollowStrategy):
    def __init__(self, compute_fn):
        self._compute_fn = compute_fn
    def compute(self, frame_context):
        return self._compute_fn(frame_context)
```

`Strategy` chỉ là wrapper của method `LineFollowerFSM._compute_huskylens_command`. **Status (2026-05-19):** Attempted to extract standalone strategies but the underlying compute methods carry FSM side effects (`self.state =`, `self.stop()`, `self._plan_lost_line_since` bookkeeping). Full extraction requires extracting the FSM lifecycle simultaneously — out of scope. Kept as wrappers with explicit docstring explaining why.

### Arch-3: `MQTTBridgeNode` constructor làm quá nhiều — **PARTIAL EXTRACTION DONE**

**File:** `mqtt_bridge/mqtt_bridge/MQTTBridgeROS.py:29-142`

113 dòng `__init__` setup: config, ROS pubs/subs, MQTT client, log throttle state, log bridge, keyboard, timer. Khó unit test vì không thể `MQTTBridgeNode()` mà không có rclpy + paho.

**Status (2026-05-19):** Extracted:

- `mqtt_bridge/plan_resolver.py` — `PlanCommandResolver` + `extract_face_ids`, pure functions, fully unit-testable (10 tests).
- `mqtt_bridge/log_throttler.py` — `LogThrottler` for throttling/change-detection (5 tests).

Còn lại để tách trong tương lai:

- `MQTTTransport` — wrap paho client lifecycle (connect_async, loop_start, reconnect_delay_set).
- `BridgeRouter` — handle `_handle_inbound_event` với DI cho transport + ros pubs (cần rclpy mocks).

### Arch-4: `ConfigManager._global_cache` là class variable

**File:** `robot_common/robot_common/config_manager.py:17`

Class-level cache shared toàn process. Tests có gọi `clear_caches()` (xem `test_config_manager.py` chắc có) nhưng nếu một test quên gọi, các test sau dùng config stale.

**Đề xuất:** chuyển sang instance cache, hoặc dùng `functools.lru_cache` decorator có thể `cache_clear()`.

### Arch-5: `set_huskylens_frame` vs `set_tracking_context` chồng chéo — **FIXED**

**File:** `line_follower/line_follower/line_follower.py:754-760`

```python
def set_tracking_context(self, frame_context):
    self._tracking_context = frame_context if isinstance(frame_context, dict) else {}

def set_huskylens_frame(self, frame):
    if not isinstance(self._tracking_context, dict):
        self._tracking_context = {}
    self._tracking_context["huskylens_frame"] = frame if isinstance(frame, dict) else None
```

Hai API set chung state. `_node` chỉ gọi `set_tracking_context`; `set_huskylens_frame` không có caller production — chỉ test legacy dùng. **Status (2026-05-19):** Removed `set_huskylens_frame`. Tests đều dùng `set_tracking_context`.

---

## 6. Test coverage — gap analysis

### Có test tốt:

- `LineFollowerFSM` (line_follower/test/) — 16+ test cho strategy + plan flow.
- `device_protocol.py` (robot_common/test/test_device_protocol.py).
- `huskylens_parser.py` (huskylens_sensor/test/test_huskylens_parser.py).
- `HuskyLensSensorReader.reconnect` (huskylens_sensor/test/test_huskylens_reader_reconnect.py).
- `AutoModeSync` (manual_control/test/test_auto_mode_sync_integration.py).
- `LineSensorReader.parse_*` (line_sensors/test/test_line_sensor_parser.py).
- `_resolve_plan_command` (mqtt_bridge/test/test_plan_resolution.py).

### Thiếu test ở các điểm quan trọng:

- **`MotorController.move`** — không có test nào → bug Critical-1 không bị phát hiện.
- **`SerialDevice.reconnect`** với fallback_ports + scan_prefixes — chỉ test HuskyLens nhánh.
- **`MQTTBridgeNode` end-to-end** — không có integration test cho echo suppression, log throttling, plan_status forwarding.
- **`KeyboardInput`** — hoàn toàn không test.
- **`LogBridge.rosout_cb`** — không test.
- **`LineFollowerNode._huskylens_cb`** với JSON malformed (huge string, deeply nested, BOM, …) — chưa fuzz.
- **`ConfigManager.load_plan`** với plan name đặc biệt (`""`, `"../config"`, unicode) — High-2 sẽ lộ ngay.
- **`command_protocol.parse_command`** với ambiguous separator — chưa test edge case High-4.

### Đề xuất bổ sung tối thiểu:

```python
# motor_driver/test/test_motor_controller.py
def test_move_wire_format_no_newline_before_paren():
    written = []
    mc = MotorController(...)
    mc.ser = FakeSerial(write=lambda b: written.append(b))
    mc.move("Forward", 8)
    assert written[-1] == b"(-8,-8,-8,-8)\n"  # newline AFTER )

# robot_common/test/test_config_manager.py
def test_load_plan_rejects_path_traversal():
    cm = ConfigManager("line_follower")
    assert cm.load_plan("../config") is None  # phải reject

# line_follower/test/test_handle_cross_pre.py
def test_handle_cross_pre_recovers_from_invalid_phase():
    fsm = LineFollowerFSM(...)
    fsm.state = LineFollowerFSM.STATE_CROSS_PRE
    fsm._cross_pre_phase = 99  # invalid
    out = fsm.update(None, now=0.0)
    assert out is not None  # không kẹt
```

---

## 7. Security/safety summary

| Vector                            | Mức             | Vị trí                                                   | Status        |
| --------------------------------- | --------------- | -------------------------------------------------------- | ------------- |
| Path traversal qua plan_select    | High            | `config_manager.py:70`, `line_follower_node.py:256`      | **chưa fix**  |
| Hardcoded credentials             | None            | —                                                        | OK (không có) |
| Subprocess injection              | Low             | `scripts/setup_serial_auto.sh` (chỉ chạy lúc setup)      | acceptable    |
| MQTT TLS/auth                     | Low (localhost) | `config.yaml:148`                                        | document only |
| Shell injection via plan messages | None            | `_consume_step_messages` chỉ publish string, không exec  | OK            |
| File write outside cwd            | None            | Code không write file user-controlled                    | OK            |
| Watchdog firmware                 | OK              | `huskylens_sensor/device_code/main.py:74, 315` feed đúng | OK            |

---

## 8. Đề xuất ưu tiên (cho dev tiếp theo)

**Sprint hiện tại (must-fix):**

1. Critical-1: Fix wire format `MotorController.move` + viết test.
2. High-2: Validate plan name regex trước khi `load_plan`.
3. Critical-3: Wrap `client.connect` try/except + dùng `reconnect_delay_set`.

**Sprint sau (should-fix):** 4. Critical-2: `MotorController.move` return None khi write fail. 5. High-1: Guard `_handle_cross_pre` cuối hàm. 6. High-3: Prune `_recent_local_pub` định kỳ. 7. Medium-1: Sửa `main()` của 4 node thiếu try/finally.

**Khi rảnh (nice-to-have):** 8. Arch-1, Arch-3: refactor `LineFollowerFSM` và `MQTTBridgeNode`. 9. High-6: Quyết định giữ/bỏ `error` synthesize trong huskylens_parser. 10. Test coverage cho `MotorController`, `MQTTBridgeNode`, `KeyboardInput`.

---

## 9. Kết luận

Project ở trạng thái **production-ready với một số rủi ro tập trung**. Kiến trúc khoẻ, refactor recent (SerialDevice base, MQTTBridge module split) là hướng đúng. Critical-1 (motor wire format) và High-2 (path traversal) là hai vấn đề cần xử lý trước khi field-deploy.

Code có dấu hiệu của codebase trưởng thành: linter, structured logging, watchdog firmware, config-driven plan, retry/reconnect ở nhiều nơi. Tuy nhiên một số chỗ chưa được test kỹ (motor, MQTT bridge end-to-end) — đây là nơi bug ẩn dễ chui vào.

Khuyến nghị: dành 1 sprint focus vào fix Critical/High + bù test coverage cho motor và mqtt_bridge.
