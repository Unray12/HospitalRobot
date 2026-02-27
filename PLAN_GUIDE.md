# Plan Guide - HospitalRobot

Tài liệu này là đặc tả chi tiết cho file plan JSON dùng bởi `line_follower`.

## 1. Mục tiêu

- Chuẩn hóa cách viết plan để robot chạy ổn định.
- Giảm lỗi runtime do sai format step/action.
- Cung cấp mẫu có thể dùng ngay cho phát triển tính năng mới.

## 2. Vị trí và cơ chế nạp plan

- Thư mục plan: `robot_common/robot_common/plans/`
- Nạp plan qua:
  - topic `/plan_select` (`std_msgs/String`)
  - hoặc `line_follower.cross_plan_name` trong `config.json`
- Cơ chế load:
  - `ConfigManager.load_plan(plan_name)`
  - cache theo tên plan trong process

## 3. Cấu trúc JSON chuẩn

```json
{
  "name": "a20",
  "start_without_cross": true,
  "steps": [
    { "action": "RotateRight", "speed": 5, "until": "line", "timeout": 8 },
    { "action": "AutoLine", "enabled": true },
    { "action": "Follow", "duration": 0.8 },
    { "action": "Stop" }
  ],
  "end_state": "stop"
}
```

Trường cấp plan:
- `name` (khuyến nghị): tên plan
- `steps` (bắt buộc): mảng step
- `end_state` (optional): `follow` hoặc `stop`
- `start_without_cross` (optional): kích hoạt chạy step đầu ngay, không đợi giao cắt
- `autoline` (optional): bật auto mode khi chọn plan
- `auto_on_select` (legacy alias): tương thích ngược với `autoline`

## 4. Chuẩn step

Mỗi phần tử trong `steps` là object.

Trường chung thường dùng:
- `label`: nhãn step để `Goto` nhảy đến
- `action`: tên hành động
- `speed`: tốc độ int
- `duration`: thời gian giây
- `until`: điều kiện kết thúc, hiện dùng `line`
- `timeout`: timeout giây cho step chờ điều kiện
- `min_duration`: thời gian tối thiểu trước khi cho phép complete step
- `strict_line`: bắt line nghiêm ngặt cho rotate-until-line
- `continue_immediately`: chạy thẳng sang step kế không chờ cross event
- `end_state`: nếu true thì force complete plan sau step
- `messages`: mảng message cần publish tại step

Ví dụ `messages`:
```json
{
  "action": "RotateRight",
  "until": "line",
  "messages": [
    { "topic": "/plan_message", "message": "say: dang den phong A20" }
  ]
}
```

## 5. Danh sách action hỗ trợ

### 5.1 Motion actions

- `Forward`
- `Backward`
- `Left`
- `Right`

Rule:
- Nếu thiếu `duration` hoặc `duration <= 0`: fallback `0.5s`
- `speed` sai kiểu: fallback về `base_speed`

### 5.2 Rotate actions

- `RotateLeft`
- `RotateRight`

Hai mode:
- Time mode: có `duration > 0`
- Until-line mode: không có duration hoặc đặt `until: "line"`

Khuyến nghị an toàn:
- Luôn thêm `timeout` cho until-line mode
- Có thể thêm `min_duration` để tránh stop sớm do nhiễu line
- Dùng `strict_line=true` khi cần bám đúng line giữa

### 5.3 Control actions

- `Wait`
  - nếu thiếu `duration`: mặc định `0.3s`
- `Stop`
  - có `duration`: dừng tạm thời rồi đi step tiếp
  - không `duration`: kết thúc logic theo `end_state`
- `Follow`
  - nếu thiếu `duration`: mặc định `0.6s`
- `Auto`
  - trả về hành vi follow mặc định
- `AutoLine`
  - bật/tắt auto mode động trong plan
  - tham số được nhận: `enabled`, `value`, `autoline`

### 5.4 Logic actions

- `Goto`
  - `target` nhận index số hoặc label string
- `Label`
  - dùng trường `label` trong step

## 6. Action alias (normalize)

Alias hợp lệ:
- `TurnLeft` -> `RotateLeft`
- `TurnRight` -> `RotateRight`
- `Rotate_Left` -> `RotateLeft`
- `Rotate_Right` -> `RotateRight`
- `Pause` -> `Wait`
- `Sleep` -> `Wait`
- `Go` -> `Forward`
- `Straight` -> `Forward`
- `Auto_Line` -> `AutoLine`

## 7. Hành vi thực thi quan trọng

- Plan được chạy khi robot ở auto mode.
- `plan_select` có debounce để bỏ duplicate command.
- Sau mỗi step, FSM có thể:
  - chuyển step ngay (`continue_immediately`)
  - hoặc chờ cross event tiếp theo
- Khi mất line trong lúc plan còn pending:
  - robot giữ STOP và chờ line quay lại (theo cấu hình hold/warn)

## 8. Mapping plan mặc định runtime

Theo `config.json` hiện tại:
- `1 -> a20`
- `2 -> a19`
- `3 -> a18`
- `4 -> a17`
- `5 -> a16`
- `6 -> a15`
- `0/clear -> clear plan`

MQTT room map mặc định:
- `room:a20..a15` hoặc `room:1..6`

## 9. Ví dụ plan nâng cao

```json
{
  "name": "plan_demo_autoline_callback",
  "start_without_cross": true,
  "steps": [
    {
      "label": "rotate_start",
      "action": "RotateRight",
      "speed": 5,
      "until": "line",
      "min_duration": 1.8,
      "timeout": 8,
      "strict_line": true
    },
    { "action": "AutoLine", "enabled": true },
    {
      "action": "RotateLeft",
      "speed": 5,
      "until": "line",
      "timeout": 6,
      "messages": [
        { "topic": "/plan_message", "message": "say: tiep tuc theo toi" }
      ]
    },
    { "action": "Follow", "duration": 0.8 },
    { "action": "AutoLine", "enabled": false },
    { "action": "Stop" }
  ],
  "end_state": "stop"
}
```

## 10. Checklist trước khi commit plan mới

1. Tất cả rotate-until-line có `timeout`.
2. Không có vòng lặp `Goto` vô hạn.
3. `end_state` phù hợp mục tiêu nhiệm vụ.
4. Test thực tế với tốc độ thấp trước.
5. Có log/plan_message đủ để theo dõi khi chạy.

## 11. Lỗi thường gặp

- `Plan not found`:
  - tên plan không trùng tên file `.json`.
- Robot quay mãi:
  - rotate-until-line thiếu `timeout`.
- Plan kết thúc nhưng robot dừng ngoài ý muốn:
  - `end_state=stop` hoặc có step `Stop` cuối.
- Step message không xuất MQTT:
  - topic không đúng (`/plan_message`) hoặc bridge chưa chạy.
