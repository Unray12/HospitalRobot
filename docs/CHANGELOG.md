# Changelog

Tóm tắt các thay đổi quan trọng. Format: keep-a-changelog style.

---

## [feature/dev-20260514] — 2026-05-19

### Fixed — Critical

- **Motor wire format** (`motor_driver/motor_driver/motor_controller.py`)
  Frame gửi xuống motor MCU trước đây có dạng `(N,N,N,N\n)` (newline đứng _trước_
  dấu `)`). Đã sửa thành `(N,N,N,N)\n` đúng convention. Wheel speeds nay là
  integer (`int(round(s * speed))`) thay vì float.
  → **Yêu cầu verify trên hardware:** MCU parser phải khớp format mới.
- **Silent motor write failure** — `MotorController.move` nay trả `None` khi
  `serial.write` raise, caller nhận thông báo thay vì bị nuốt lỗi.
- **MQTT bridge no reconnect at startup** — Đổi từ `connect()` + `loop_forever`
  sang `connect_async()` + `loop_start()` + `reconnect_delay_set(1, 10)`. Node
  không còn crash khi broker chưa ready lúc boot.

### Fixed — High

- **`_handle_cross_pre` could return None** — Thêm fallback branch reset về
  phase 0 forward khi `_cross_pre_phase` ở giá trị bất thường.
- **Path traversal qua `plan_select`** — `ConfigManager.load_plan` từ chối tên
  plan không khớp regex `^[A-Za-z0-9_-]+$`. Block input như `../config`,
  `..\config`, `plans/../config`, empty, non-string.
- **`_recent_local_pub` memory leak** — Prune opportunistically khi dict
  vượt 64 entries; entries cũ hơn `echo_suppress_window_sec` bị xóa.
- **`_compute_line_sensor_command` KeyError** — Dùng `.get()` defaults cho
  tất cả frame keys; không crash khi nhận frame thiếu field.
- **HuskyLens parser `error` synthesis removed** — Heuristic
  `0.8*tail_offset_x + 0.2*angle_deg` đã bị xóa. Field `error` chỉ tồn tại
  khi firmware gửi explicit.

### Fixed — Medium

- **Try/finally cleanup** trong `motor_driver_node`, `line_sensor_driver_node`,
  `manual_control_node`, `line_follower_node` — `destroy_node()` luôn chạy
  kể cả khi Ctrl+C.
- **`_drain_inbound_queue` polling** — 50 Hz → 20 Hz (0.05s).
- **HuskyLens firmware arrow selection** — Sort theo `y_tail` desc; chọn
  arrow gần robot nhất thay vì `arrows[0]` ngẫu nhiên.
- **ConfigManager re-instantiate** — `LineFollowerNode` cache `self._cfg_mgr`,
  không tạo instance mới mỗi plan select.
- **`LineSensorReader.read_frame`** — Loop tìm frame hợp lệ thay vì return None
  ngay khi line cuối cùng malformed.
- **`SerialDevice` open sleep** — 0.2s → 0.05s (USB-CDC không cần DTR settle).

### Fixed — Low

- **`safe_symlink` race condition** — Dùng `os.symlink` + `os.replace` cho
  atomic swap thay vì `unlink` + `symlink_to`.
- **`pkill -f` scope** — Thu hẹp vào tên package HospitalRobot, không kill
  ROS process của user khác.
- **ANSI color khi không phải TTY** — `LogAdapter` check `sys.stderr.isatty()`;
  log file/journalctl không còn dính escape codes.
- **`KeyboardInput.stop()` thực sự shutdown** — `select.select` timeout 0.2s
  trên POSIX, `msvcrt.kbhit` poll trên Windows.

### Added — Architecture

- **`mqtt_bridge/plan_resolver.py`** — `PlanCommandResolver` + `extract_face_ids`,
  pure module, 10 unit tests, không cần rclpy/paho.
- **`mqtt_bridge/log_throttler.py`** — `LogThrottler` cho throttle/change-detect,
  5 unit tests.
- **`Arch-5`** — Xóa `LineFollowerFSM.set_huskylens_frame` (duplicate API,
  không caller production).

### Added — Tests

| File                                                    | Cases | Coverage                                   |
| ------------------------------------------------------- | ----: | ------------------------------------------ |
| `motor_driver/test/test_motor_controller.py`            |     7 | Wire format, write fail, invalid direction |
| `mqtt_bridge/test/test_plan_resolver.py`                |    10 | Resolver pure tests                        |
| `mqtt_bridge/test/test_log_throttler.py`                |     5 | Throttle + change-detect                   |
| `mqtt_bridge/test/test_local_pub_prune.py`              |     5 | Skip nếu không có rclpy                    |
| `line_follower/test/test_line_follower_fsm.py` (append) |     2 | `_handle_cross_pre` guard                  |
| `robot_common/test/test_config_manager.py` (append)     |     3 | Path traversal reject                      |

Tổng: **111 passed, 5 skipped** (baseline 84 → +27 test).

### Deferred — Architecture

- **`Arch-1`** PlanExecutor split khỏi `LineFollowerFSM` — Tests truy cập trực
  tiếp 7+ private fields; risk/value chưa thuyết phục. Chờ business case
  cụ thể.
- **`Arch-2`** Strategy pattern truly standalone — Compute methods có FSM
  side effects (`self.state`, `self.stop()`, `_plan_lost_line_since`); full
  extraction cần extract FSM lifecycle song song. Giữ wrappers với docstring.
- **`Arch-4`** ConfigManager class-var cache → instance-var — Doc-only concern,
  không cải thiện đáng kể.

### Docs

- `docs/30-code-review.md` — Báo cáo audit chi tiết (~530 dòng).
- `docs/README.md` — Index folder docs.
- `docs/CHANGELOG.md` — File này.
- `docs/01-architecture.md` — Module map cập nhật.

---

## Older history

Xem `git log --oneline main..feature/dev-20260514` cho commits từ trước
2026-05-19. Các thay đổi đáng kể trước đó:

- `9b818a1` — Extract `SerialDevice` base class, split `MQTTBridge` into modules.
- `257febd` — Bootstrap simplification: remove launch stages, only build + serial.
- `1d73ed4` — Note.txt rewrite (current setup, protocol, run commands).
