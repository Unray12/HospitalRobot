# HospitalRobot Documentation

Tài liệu kỹ thuật cho dự án HospitalRobot (ROS2 Humble + MicroPython ESP32 + Motor MCU).

> **Quick start:** `../Note.txt` chứa hướng dẫn vận hành & setup nhanh. Bắt đầu từ đó nếu bạn chưa quen project.

---

## Cấu trúc tài liệu

Numbered prefix theo loại nội dung:
`0x` — kiến trúc / overview · `1x` — guides / how-to ·
`2x` — reference / spec · `3x` — reviews / audits

### Overview & Architecture

| File                                             | Mô tả                                                                                    | Đối tượng             |
| ------------------------------------------------ | ---------------------------------------------------------------------------------------- | --------------------- |
| [`01-architecture.md`](01-architecture.md)       | Module map + dependency graph + threading model                                          | Developer, tech lead  |
| [`02-project-flow.md`](02-project-flow.md)       | Luồng dữ liệu theo subsystem (camera, line, huskylens, ...)                              | Developer, integrator |
| [`03-end-to-end-flow.md`](03-end-to-end-flow.md) | **Scenario-driven walkthrough**: boot, plan execution, tracking, recovery, timing budget | Developer, QA, ops    |

### Guides — How-to

| File                                           | Mô tả                                                  | Đối tượng            |
| ---------------------------------------------- | ------------------------------------------------------ | -------------------- |
| [`10-plan-authoring.md`](10-plan-authoring.md) | Đặc tả file plan YAML cho `line_follower`              | Plan author          |
| [`11-mqtt-messaging.md`](11-mqtt-messaging.md) | Contract MQTT: topic, payload, ví dụ publish/subscribe | App/UI developer, QA |
| [`12-logging.md`](12-logging.md)               | Chuẩn log thống nhất, event taxonomy, throttle         | Developer            |

### Reference — Specs

| File                                                           | Mô tả                                                  | Đối tượng          |
| -------------------------------------------------------------- | ------------------------------------------------------ | ------------------ |
| [`20-device-protocol.md`](20-device-protocol.md)               | Serial JSON envelope v1 giữa firmware ↔ host           | Firmware developer |
| [`21-line-tracking-strategy.md`](21-line-tracking-strategy.md) | Cơ chế chọn strategy: line_sensor / huskylens / hybrid | Developer          |
| [`22-huskylens-integration.md`](22-huskylens-integration.md)   | Luồng HuskyLens sensor ↔ line_follower                 | Developer          |

### Reviews & Decisions

| File                                     | Mô tả                                                                             |
| ---------------------------------------- | --------------------------------------------------------------------------------- |
| [`30-code-review.md`](30-code-review.md) | Audit code chi tiết (2026-05-19): Critical/High/Medium/Low + Arch refactor status |
| [`CHANGELOG.md`](CHANGELOG.md)           | Tóm tắt thay đổi đáng kể theo branch                                              |

---

## Đọc theo vai trò

### Tôi là...

**...developer mới join project**

1. `../Note.txt` — setup & cách chạy
2. [`03-end-to-end-flow.md`](03-end-to-end-flow.md) — kịch bản vận hành chi tiết, dễ hình dung
3. [`02-project-flow.md`](02-project-flow.md) — luồng theo subsystem
4. [`01-architecture.md`](01-architecture.md) — module map
5. [`12-logging.md`](12-logging.md) — convention khi viết code mới

**...firmware developer thêm thiết bị**

1. [`20-device-protocol.md`](20-device-protocol.md) — schema envelope v1, checklist thêm role mới
2. Template tham khảo: `line_sensors/device_code/main.py` (đơn giản nhất)

**...app/UI developer tích hợp MQTT**

1. [`11-mqtt-messaging.md`](11-mqtt-messaging.md) — contract đầy đủ
2. [`10-plan-authoring.md`](10-plan-authoring.md) — gửi plan_select dạng nào

**...plan author viết kịch bản robot**

1. [`10-plan-authoring.md`](10-plan-authoring.md) — actions, fields, alias
2. Plan mẫu: `robot_common/robot_common/plans/a19.yaml`

**...QA test hệ thống**

1. [`11-mqtt-messaging.md`](11-mqtt-messaging.md) §5–6 — checklist publish/subscribe
2. [`30-code-review.md`](30-code-review.md) §6 — test coverage gap (biết chỗ nào dễ regress)

**...code reviewer / tech lead**

1. [`30-code-review.md`](30-code-review.md) — full audit, Arch decisions
2. [`CHANGELOG.md`](CHANGELOG.md) — gần đây đổi gì

---

## Convention

- **Ngôn ngữ:** Tiếng Việt, code snippet/identifier tiếng Anh.
- **Markdown:** GitHub flavored, 80–100 cột text.
- **Code reference:** `file_path:line_number` (vd `motor_controller.py:25`).
- **Khi update doc, ghi date đầu mục:** `**Status (YYYY-MM-DD):** ...`
- **File naming:** `NN-kebab-case.md`. Số prefix theo loại
  (`0x` overview, `1x` guide, `2x` reference, `3x` review).

---

## Đường dẫn tham chiếu nhanh

```
robot_common/robot_common/config.yaml     # Config tất cả node
robot_common/robot_common/plans/          # Plan files (a15..a19)
scripts/bootstrap_all.sh                  # Setup all-in-one
scripts/setup_serial_auto.sh              # Probe + symlink /dev/hospitalrobot/*
Note.txt                                  # Operator guide (Vietnamese)
```

---

## Build typst report (optional)

`index.typ` + `theme.typ` + `packages/` là source để build PDF từ markdown docs qua [Typst](https://typst.app/). Không bắt buộc để hiểu code.
