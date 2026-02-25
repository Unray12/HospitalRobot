# HospitalRobot Plan Guide

Tài liệu này hướng dẫn cách viết file plan JSON cho `line_follower`.

## 1. Vị trí file plan

- Thư mục: `robot_common/robot_common/plans/`
- Mỗi plan là 1 file JSON, ví dụ: `plan_ntp.json`, `plan_turn_right.json`.

## 2. Cấu trúc plan

```json
{
  "name": "plan_example",
  "autoline": true,
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
- `end_state`:
  - `follow`: xong plan thì quay về bám line.
  - `stop`: xong plan thì dừng.

## 3. Quy tắc chung cho `steps`

- `steps` là mảng object JSON, thực thi tuần tự từ trên xuống.
- Mỗi step nên có `action`.
- Tất cả key đều phân biệt chữ hoa/thường theo JSON, nhưng giá trị `action` được hệ thống normalize (có alias).
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
- Quay theo thời gian:
  - Đặt `duration` > 0.

Tham số:
- `speed`
- `duration` (tùy chọn)
- `until` (tùy chọn, giá trị đúng: `"line"`)
- `timeout` (tùy chọn, giây)

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

## 5. Bảng tham số chi tiết theo action

### 5.0 Quy ước thời gian (`duration`, `timeout`)

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
- Luật hoạt động:
  - Nếu có `duration` > 0: ưu tiên chế độ theo thời gian.
  - Nếu không có `duration`: dùng chế độ `until line`.
  - Nếu có `timeout` và quá thời gian: tự thoát step để tránh quay vô hạn.
  - Trong chế độ `until line`, hệ thống vẫn ép quay tối thiểu `rotate_min_duration` trước khi được phép kết thúc (tránh dừng quá sớm do nhiễu cảm biến).

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

### 5.6 `Auto`

- Mục đích: thoát nhanh khỏi plan step hiện tại về hành vi mặc định follow.
- Tham số: không cần.

### 5.7 `Goto`

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

## 7. Tham số metadata của step

- `label` (optional): tên nhãn của step để `Goto` trỏ tới.
- Các key không được dùng sẽ bị bỏ qua.

## 8. Tham số cấp plan

- `name` (required): tên định danh plan.
- `steps` (required): mảng step.
- `autoline` (optional): `true/false`.
  - Tương thích key cũ: `auto_on_select`.
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
{ "name": "plan_turn_right", "steps": [...], "end_state": "follow" }
```
```json
{ "name": "plan_parking", "steps": [...], "end_state": "stop" }
```

## 9. Lựa chọn plan trong runtime

- Topic chọn plan: `/plan_select` (`std_msgs/String`).
- Chuẩn khuyến nghị mới: `room:a20`, `room:a21`, `room:a22`, `room:a23`.
- Alias plan trong config:
  - `1` -> `plan_ntp`
  - `2` -> `plan_straight`
  - `3` -> `plan_turn_right`
  - `4` -> `plan_stop`
  - `0` hoặc `clear` -> xóa plan hiện tại.
- Chuẩn gửi qua MQTT (khuyến nghị):
  - `room:a20`, `room:a21`, `room:a22`, `room:a23` (theo mã phòng).
  - `room:0` hoặc `clear` để clear plan.
  - Tương thích thêm: `room:1..4`, `phong:a20`, `plan:a20`, hoặc gửi trực tiếp tên plan (vd `plan_turn_right`).

## 10. Mẫu plan để dùng ngay

- `plan_ntp.json`: scan trái/phải + follow ngắn
- `plan_straight.json`: đi thẳng qua giao
- `plan_turn_right.json`: rẽ phải theo line + timeout an toàn
- `plan_u_turn.json`: quay đầu
- `plan_stop.json`: dừng robot

## 11. Ví dụ đầy đủ (có label + goto + timeout)

```json
{
  "name": "plan_demo_full",
  "steps": [
    { "action": "Wait", "duration": 0.2 },
    { "label": "scan_start", "action": "RotateLeft", "speed": 5, "until": "line", "timeout": 1.5 },
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
2. Nếu cần gọi bằng phím số, thêm alias vào:
   - `robot_common/robot_common/config.json`
   - mục `line_follower.plan_alias` và `mqtt_bridge.plan_keys`
3. Chạy lại node và publish tên plan lên `/plan_select` để test.
