# HospitalRobot Plan Guide

Tài liệu này hướng dẫn cách viết file plan JSON cho `line_follower`.

## 1. Vị trí file plan

- Thư mục: `robot_common/robot_common/plans/`
- Mỗi plan là 1 file JSON, ví dụ: `a20.json`, `a21.json`.

## 2. Cấu trúc plan

```json
{
  "name": "plan_example",
  "autoline": true,
  "start_without_cross": false,
  "steps": [
    { "action": "Wait", "duration": 0.2 },
    { "action": "RotateRight", "speed": 6, "until": "line", "timeout": 2.5 },
    { "action": "Follow", "duration": 0.5 }
  ],
  "end_state": "follow"
}
```

- `name`: tên plan.
- `steps`: danh sách bước thực thi.
- `autoline` (optional): khi chọn plan sẽ tự bật auto mode (`pick_robot=1`).
  - Nếu không khai báo, dùng mặc định từ config `line_follower.auto_on_plan_select` (hiện tại là `true`).
  - Tương thích key cũ: `auto_on_select`.
- `start_without_cross` (optional): chạy plan ngay khi select, không cần gặp giao cắt.
  - Tương thích key cũ: `start_immediately`.
  - Lưu ý: vẫn chạy pha `cross_pre` (forward -> stop) trừ khi step đầu là `Rotate...` và `autoline` đang tắt.
- `end_state`:
  - `follow`: xong plan thì quay về bám line.
  - `stop`: xong plan thì dừng.

## 3. Quy tắc chung cho `steps`

- `steps` là mảng object JSON, thực thi tuần tự từ trên xuống.
- Mỗi step nên có `action`.
- Tất cả key phân biệt chữ hoa/thường theo JSON, nhưng giá trị `action` được hệ thống normalize (có alias).
- Nếu step thiếu `action`, hệ thống sẽ xem như `Stop`.
- Nếu action không hợp lệ: step đó bị bỏ qua, log warning và chạy step kế tiếp.

## 4. Action được hỗ trợ

### 4.1 Nhóm điều hướng có duration

- `Forward`
- `Backward`
- `Left`
- `Right`

Tham số:
- `speed` (số nguyên, >= 0), mặc định `base_speed`.
- `duration` (giây), nếu không có sẽ dùng mặc định 0.5s.

Ví dụ:
```json
{ "action": "Forward", "speed": 8, "duration": 0.7 }
```

### 4.2 Nhóm quay

- `RotateLeft`
- `RotateRight`

Chế độ hỗ trợ:
- Quay theo line:
  - Đặt `until: "line"` (hoặc bỏ qua `until`).
  - Có thể đặt `timeout` để tránh quay vô hạn.
  - Có thể đặt `min_duration` để ép quay tối thiểu trước khi xét line (mặc định `rotate_min_duration`).
- Quay theo thời gian:
  - Đặt `duration` > 0.

Tham số:
- `speed`
- `duration` (tùy chọn)
- `until` (tùy chọn, giá trị đúng: `"line"`)
- `timeout` (tùy chọn, giây)
- `min_duration` (tùy chọn, giây)
- `strict_line` (tùy chọn, bool): chỉ cho phép dừng khi line ở giữa (alias `center_only`).

Ví dụ quay theo line:
```json
{ "action": "RotateRight", "speed": 6, "until": "line", "timeout": 2.5 }
```

Ví dụ quay theo thời gian:
```json
{ "action": "RotateLeft", "speed": 6, "duration": 1.2 }
```

### 4.3 Nhóm điều khiển động

- `Wait`: dừng tạm thời.
  - Dùng `duration`; nếu bỏ trống, hệ thống dùng mặc định ngắn.
- `Stop`:
  - Nếu có `duration`: dừng trong khoảng đó rồi chạy step tiếp.
  - Nếu không có `duration`: kết thúc plan theo `end_state`.
- `Follow`: bám line trong `duration`.
- `Auto`: trả lại chế độ mặc định (follow).
- `AutoLine`: bật/tắt auto mode ngay tại step.
  - Hỗ trợ tham số: `enabled`, `value`, hoặc `autoline` (true/false).
  - Sau step `AutoLine`, FSM quay về FOLLOWING và chờ giao cắt tiếp theo để chạy step kế tiếp.

Ví dụ:
```json
{ "action": "Wait", "duration": 0.3 }
```
```json
{ "action": "Follow", "duration": 0.8 }
```
```json
{ "action": "AutoLine", "enabled": false }
```

### 4.4 Nhóm điều hướng logic

- `Goto`: nhảy tới step khác.
  - `target` có thể là:
    - số index step (bắt đầu từ 0), ví dụ `2`
    - tên label, ví dụ `"finish"`

Ví dụ:
```json
{ "action": "Goto", "target": "finish" }
```

### 4.5 Label

- Mỗi step có thể gán:
  - `label`: tên nhãn để `Goto` nhảy đến.

Ví dụ:
```json
{
  "steps": [
    { "label": "start", "action": "Wait", "duration": 0.1 },
    { "action": "Goto", "target": "finish" },
    { "action": "RotateLeft", "duration": 0.4 },
    { "label": "finish", "action": "Stop" }
  ]
}
```

### 4.6 Step messages (ROS topic publish)

- Mỗi step có thể gửi message ra ROS topic ngay khi bắt đầu step:
  - `messages`: danh sách object `{ topic, message }` hoặc `{ topic, payload }`.
  - Nếu `message` là object/list, hệ thống sẽ JSON-encode trước khi publish.

Ví dụ:
```json
{
  "action": "RotateRight",
  "messages": [
    { "topic": "/plan_message", "message": "say:sap toi roi" },
    { "topic": "/plan_status", "payload": { "tag": "scan_right" } }
  ]
}
```

## 5. Bảng tham số chi tiết theo action

### 5.0 Quy ước thời gian (`duration`, `timeout`, `min_duration`)

- Đơn vị: **giây**.
- Kiểu dữ liệu: `float` hoặc `int`.
- Ví dụ:
  - `0.2` = 0.2 giây (200 ms)
  - `1` = 1 giây
  - `1.5` = 1.5 giây
- Nếu bạn nhập `duration <= 0`, hệ thống xem như không hợp lệ và dùng mặc định (nếu action đó có mặc định).
- Khuyến nghị:
  - Tinh chỉnh theo bước nhỏ: `0.1` - `0.2` giây.
  - Không tăng đột ngột để tránh robot giật/hụt line.

### 5.1 `Forward` / `Backward` / `Left` / `Right`

- Mục đích: chạy theo hướng tương ứng trong một khoảng thời gian.
- Tham số:
  - `speed` (optional): `int`, mặc định `base_speed`, giá trị âm sẽ bị clamp về `0`.
  - `duration` (optional): `float` (giây), mặc định `0.5`.
- Kết thúc step: khi hết `duration`.

Hành vi khi thiếu `duration`:
- Nếu không khai báo `duration` hoặc `duration <= 0`: dùng **0.5s**.

Ví dụ:
```json
{ "action": "Forward", "speed": 8, "duration": 0.5 }
```

### 5.2 `RotateLeft` / `RotateRight`

- Mục đích: xoay robot để canh line hoặc xoay theo thời gian.
- Tham số:
  - `speed` (optional): `int`, mặc định `base_speed`.
  - `duration` (optional): `float` (giây).
  - `until` (optional): `"line"` để xoay đến khi line centered.
  - `timeout` (optional): `float` (giây), chỉ dùng tốt nhất khi `until: "line"`.
  - `min_duration` (optional): ép quay tối thiểu trước khi xét line.
  - `strict_line` (optional): chỉ cho phép dừng khi line vào giữa (alias `center_only`).
- Luật hoạt động:
  - Nếu có `duration` > 0: ưu tiên chế độ theo thời gian.
  - Nếu không có `duration`: dùng chế độ `until line`.
  - Nếu có `timeout` và quá thời gian: tự thoát step để tránh quay vô hạn.
  - Trong chế độ `until line`, hệ thống vẫn ép quay tối thiểu `min_duration` trước khi được phép kết thúc.

Hành vi mặc định:
- Không có `duration` -> quay theo line.
- Không có `timeout` -> có thể quay rất lâu nếu không bắt lại line.
- Khuyến nghị luôn có `timeout` cho action quay theo line.

### 5.3 `Wait`

- Mục đích: chèn khoảng dừng ngắn giữa các động tác.
- Tham số:
  - `duration` (optional): `float` (giây), nếu thiếu hệ thống dùng mặc định **0.3s**.
- Kết thúc step: khi hết thời gian chờ.

Ví dụ:
```json
{ "action": "Wait", "duration": 0.2 }
```
```json
{ "action": "Wait" }
```

Với ví dụ thứ 2, robot sẽ đợi **0.3s**.

### 5.4 `Stop`

- Mục đích: dừng robot.
- Tham số:
  - `duration` (optional): `float` (giây).
- Luật hoạt động:
  - Có `duration`: dừng tạm rồi chạy step tiếp theo.
  - Không có `duration`: coi như điểm kết thúc logic (phụ thuộc `end_state`).

Lưu ý quan trọng:
- `Stop` **không có `duration`** thường được dùng làm điểm kết plan.
- `Stop` **có `duration`** dùng như 1 nhịp dừng kỹ thuật giữa plan.

### 5.5 `Follow`

- Mục đích: trả robot về bám line tự động trong khoảng thời gian ngắn.
- Tham số:
  - `duration` (optional): `float` (giây), nếu thiếu dùng mặc định **0.6s**.
- Kết thúc step: khi hết duration.

Ví dụ:
```json
{ "action": "Follow", "duration": 0.8 }
```
```json
{ "action": "Follow" }
```

Với ví dụ thứ 2, robot sẽ follow trong **0.6s**.

### 5.6 `AutoLine`

- Mục đích: bật/tắt auto mode trong plan.
- Tham số:
  - `enabled` / `value` / `autoline`: `true/false`.

### 5.7 `Auto`

- Mục đích: thoát nhanh khỏi plan step hiện tại về hành vi mặc định follow.
- Tham số: không cần.

### 5.8 `Goto`

- Mục đích: nhảy tới step khác trong cùng plan.
- Tham số:
  - `target` (required):
    - `int` index step, hoặc
    - `string` label.
- Lưu ý:
  - Nếu target sai: step bị bỏ qua, log warning.
  - Tránh tạo vòng lặp vô hạn bằng `Goto`.

## 6. Alias action (viết linh hoạt)

Hệ thống tự map các alias sau:

- `TurnLeft` -> `RotateLeft`
- `TurnRight` -> `RotateRight`
- `Rotate_Left` -> `RotateLeft`
- `Rotate_Right` -> `RotateRight`
- `Pause` -> `Wait`
- `Sleep` -> `Wait`
- `Go` -> `Forward`
- `Straight` -> `Forward`
- `Auto_Line` / `AutoLine_On` / `AutoLine_Off` -> `AutoLine`

## 7. Tham số metadata của step

- `label` (optional): tên nhãn của step để `Goto` trỏ tới.
- `messages` (optional): list message publish khi step bắt đầu.
- `continue_immediately` (optional): chạy step kế ngay sau khi action kết thúc, không chờ giao cắt.
  - Alias: `no_wait_cross`.
- `end_state` (optional, bool): ép kết thúc plan sau step hiện tại.
- `min_duration` / `strict_line` (optional): tham số riêng cho rotate.

## 8. Tham số cấp plan

- `name` (required): tên định danh plan.
- `steps` (required): mảng step.
- `autoline` (optional): `true/false`.
  - Tương thích key cũ: `auto_on_select`.
- `start_without_cross` (optional): chạy plan ngay khi chọn.
  - Tương thích key cũ: `start_immediately`.
- `end_state` (optional):
  - `follow` (khuyến nghị cho nhiệm vụ liên tục).
  - `stop` (dừng robot sau plan).

Giải thích rõ `end_state`:
- `end_state: "follow"`:
  - Khi chạy hết `steps`, FSM quay về chế độ bám line.
  - Phù hợp khi robot cần tiếp tục đi sau khi xử lý ngã tư.
- `end_state: "stop"`:
  - Khi chạy hết `steps`, FSM chuyển sang dừng hẳn.
  - Phù hợp khi tới đích hoặc chờ lệnh mới.

Ví dụ:
```json
{ "name": "plan_example", "steps": [...], "end_state": "follow" }
```
```json
{ "name": "plan_parking", "steps": [...], "end_state": "stop" }
```

## 9. Lựa chọn plan trong runtime

- Topic chọn plan: `/plan_select` (`std_msgs/String`).
- Chuẩn khuyến nghị mới: `room:a20`, `room:a21`, `room:a22`, `room:a23`, `room:a24`, `room:a25`.
- Alias plan trong config `line_follower.plan_alias`:
  - `1` -> `a20`
  - `2` -> `a21`
  - `3` -> `a22`
  - `4` -> `a23`
  - `5` -> `a24`
  - `6` -> `a25`
  - `0` hoặc `clear` -> xóa plan hiện tại.
- Chuẩn gửi qua MQTT (khuyến nghị):
  - `room:a20`, `room:a21`, `room:a22`, `room:a23`, `room:a24`, `room:a25`.
  - `room:0` hoặc `clear` để clear plan.
- Tương thích thêm:
  - `room:1..6`, `phong:1..6`, `plan:1..6`
  - gửi trực tiếp tên plan (vd `a20`, `a25`).

## 10. Mẫu plan để dùng ngay

- `a20.json`: plan hướng dẫn phòng, có AutoLine + message
- `a21.json`: đi thẳng qua giao cắt
- `a22.json`: rẽ phải theo line + timeout an toàn
- `a23.json`: dừng robot
- `a24.json`: quay đầu
- `a25.json`: demo AutoLine + message
- Tham khảo thêm ví dụ tổng hợp: `docs/plan_examples.json`

## 11. Ví dụ đầy đủ (label + goto + timeout + messages)

```json
{
  "name": "plan_demo_full",
  "start_without_cross": true,
  "steps": [
    { "action": "Wait", "duration": 0.2 },
    {
      "label": "scan_start",
      "action": "RotateLeft",
      "speed": 5,
      "until": "line",
      "timeout": 1.5,
      "messages": [
        { "topic": "/plan_message", "message": "say:bat dau scan" }
      ]
    },
    { "action": "Follow", "duration": 0.4 },
    { "action": "Goto", "target": "finish" },
    { "action": "RotateRight", "speed": 6, "duration": 0.8 },
    { "label": "finish", "action": "Stop", "duration": 0.2 }
  ],
  "end_state": "follow"
}
```

## 12. Ví dụ timeline để dễ hiểu `duration` và `timeout`

Ví dụ plan:
```json
{
  "steps": [
    { "action": "Wait", "duration": 0.2 },
    { "action": "RotateRight", "until": "line", "timeout": 2.0 },
    { "action": "Forward", "duration": 0.5 },
    { "action": "Stop" }
  ],
  "end_state": "follow"
}
```

Timeline:
- Bước 1: dừng 0.2s.
- Bước 2: quay phải đến khi thấy line giữa, nhưng tối đa 2.0s.
  - Nếu bắt được line sớm (ví dụ sau 0.7s) -> chuyển bước 3 ngay.
  - Nếu không bắt được line -> hết 2.0s thì timeout, vẫn chuyển bước 3.
- Bước 3: đi thẳng 0.5s.
- Bước 4: `Stop` không duration -> kết thúc plan theo `end_state`.
- `end_state = follow` -> sau đó robot quay lại bám line.

## 13. Khuyến nghị khi tạo plan mới

1. Luôn đặt `timeout` cho các step `Rotate...` dùng `until: "line"`.
2. Tránh step `Goto` tạo vòng lặp vô hạn.
3. Thêm `Wait` ngắn trước step quay để robot ổn định.
4. Ưu tiên `end_state: "follow"` nếu robot cần tiếp tục nhiệm vụ.
5. Test trên tốc độ thấp trước khi tăng `speed`.

## 14. Lỗi thường gặp

- `Plan not found`: tên plan gửi lên `/plan_select` không đúng tên file.
- Robot quay mãi: thiếu `timeout` trong step `Rotate...` theo line.
- Robot dừng luôn: step cuối là `Stop` và `end_state` đang `stop`.
- Plan bị bỏ qua: bị duplicate debounce trong thời gian rất ngắn (để tránh nhấn nút trùng).

## 15. Quy trình thêm plan mới

1. Tạo file JSON mới trong `robot_common/robot_common/plans/`.
2. Nếu cần gọi bằng phím số hoặc room code, thêm mapping vào:
   - `robot_common/robot_common/config.json`
   - mục `line_follower.plan_alias`, `mqtt_bridge.plan_keys`, `mqtt_bridge.room_plans`
3. Chạy lại node và publish tên plan lên `/plan_select` để test.
