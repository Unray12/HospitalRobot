# HospitalRobot Plan Catalog

Tài liệu này tổng hợp các plan hiện có trong `robot_common/robot_common/plans/` thành một hướng dẫn duy nhất.

**Phạm vi**
- `a20.json`
- `a21.json`
- `a22.json`
- `a23.json`
- `a24.json`
- `a25.json`

**Cấu trúc chung của plan**
- `name`: định danh plan.
- `start_without_cross` (optional): chạy ngay khi select plan, không cần gặp giao cắt.
- `steps`: danh sách bước thực thi theo thứ tự.
- `end_state`: `follow` hoặc `stop`.

**Plan `a20`**
- Mục đích: hướng dẫn phòng, có AutoLine và message `/plan_message`.
- `start_without_cross`: `true`
- `end_state`: `stop`

Bước thực thi:
| Step | Action | Params | Notes |
| --- | --- | --- | --- |
| 1 | RotateRight | speed=5, until=line | message `/plan_message`: `say:hãy đi theo tôi để đến phòng khám` |
| 2 | AutoLine | enabled=true | |
| 3 | RotateLeft | speed=5, until=line | message `/plan_message`: `say:bạn còn ở đó chứ?` |
| 4 | RotateRight | speed=5, until=line, min_duration=20, timeout=25 | message `/plan_message`: `say:sắp tới rồi bạn nhé` |
| 5 | RotateRight | speed=5, until=line | |
| 6 | RotateRight | speed=5, until=line, strict_line=true, continue_immediately=true, end_state=true | |

JSON nguồn:
```json
{
  "name": "a20",
  "start_without_cross": true,
  "steps": [
    {
      "label": "scan_right",
      "action": "RotateRight",
      "until": "line",
      "speed": 5,
      "messages": [
        {
          "topic": "/plan_message",
          "message": "say:hãy đi theo tôi để đến phòng khám"
        }
      ]
    },
    {
      "action": "AutoLine",
      "enabled": true
    },
    {
      "label": "scan_left",
      "action": "RotateLeft",
      "speed": 5,
      "until": "line",
      "messages": [
        {
          "topic": "/plan_message",
          "message": "say:bạn còn ở đó chứ?"
        }
      ]
    },
    {
      "label": "scan_right",
      "action": "RotateRight",
      "speed": 5,
      "until": "line",
      "min_duration": 20,
      "timeout": 25,
      "messages": [
        {
          "topic": "/plan_message",
          "message": "say:sắp tới rồi bạn nhé"
        }
      ]
    },
    {
      "label": "scan_right",
      "action": "RotateRight",
      "speed": 5,
      "until": "line"
    },
    {
      "label": "scan_right",
      "action": "RotateRight",
      "speed": 5,
      "until": "line",
      "end_state": "true",
      "strict_line": true,
      "continue_immediately": true
    }
  ],
  "end_state": "stop"
}
```

**Plan `a21`**
- Mục đích: đi thẳng qua giao cắt rồi quay lại bám line.
- `start_without_cross`: không đặt
- `end_state`: `follow`

Bước thực thi:
| Step | Action | Params | Notes |
| --- | --- | --- | --- |
| 1 | Follow | duration=1.2 | |
| 2 | Stop | duration=0.2 | dừng ngắn để ổn định |

JSON nguồn:
```json
{
  "name": "a21",
  "steps": [
    {
      "action": "Follow",
      "duration": 1.2
    },
    {
      "action": "Stop",
      "duration": 0.2
    }
  ],
  "end_state": "follow"
}
```

**Plan `a22`**
- Mục đích: rẽ phải theo line khi gặp giao cắt.
- `start_without_cross`: không đặt
- `end_state`: `follow`

Bước thực thi:
| Step | Action | Params | Notes |
| --- | --- | --- | --- |
| 1 | Wait | duration=0.2 | ổn định trước khi quay |
| 2 | RotateRight | speed=6, until=line, timeout=2.5 | bắt line ở hướng rẽ |
| 3 | Follow | duration=0.5 | bám line ngắn |

JSON nguồn:
```json
{
  "name": "a22",
  "steps": [
    {
      "action": "Wait",
      "duration": 0.2
    },
    {
      "action": "RotateRight",
      "speed": 6,
      "until": "line",
      "timeout": 2.5
    },
    {
      "action": "Follow",
      "duration": 0.5
    }
  ],
  "end_state": "follow"
}
```

**Plan `a23`**
- Mục đích: dừng robot theo 2 nhịp.
- `start_without_cross`: không đặt
- `end_state`: `stop`

Bước thực thi:
| Step | Action | Params | Notes |
| --- | --- | --- | --- |
| 1 | Stop | duration=0.5 | dừng tạm |
| 2 | Stop |  | kết thúc plan |

JSON nguồn:
```json
{
  "name": "a23",
  "steps": [
    {
      "action": "Stop",
      "duration": 0.5
    },
    {
      "action": "Stop"
    }
  ],
  "end_state": "stop"
}
```

**Plan `a24`**
- Mục đích: quay đầu 180 độ rồi bám line.
- `start_without_cross`: không đặt
- `end_state`: `follow`

Bước thực thi:
| Step | Action | Params | Notes |
| --- | --- | --- | --- |
| 1 | Wait | duration=0.2 | ổn định trước khi quay |
| 2 | RotateRight | speed=6, duration=1.8 | quay theo thời gian |
| 3 | RotateRight | speed=6, until=line, timeout=2.0 | bắt lại line |
| 4 | Follow | duration=0.6 | bám line ngắn |

JSON nguồn:
```json
{
  "name": "a24",
  "steps": [
    {
      "action": "Wait",
      "duration": 0.2
    },
    {
      "action": "RotateRight",
      "speed": 6,
      "duration": 1.8
    },
    {
      "action": "RotateRight",
      "speed": 6,
      "until": "line",
      "timeout": 2.0
    },
    {
      "action": "Follow",
      "duration": 0.6
    }
  ],
  "end_state": "follow"
}
```

**Plan `a25`**
- Mục đích: demo AutoLine + message `/plan_status`.
- `start_without_cross`: `true`
- `end_state`: `stop`

Bước thực thi:
| Step | Action | Params | Notes |
| --- | --- | --- | --- |
| 1 | RotateRight | speed=5, until=line, min_duration=1.5, timeout=6, strict_line=true | message `/plan_status`: `a25_scan_right` |
| 2 | AutoLine | enabled=true | message `/plan_status`: `a25_autoline_on` |
| 3 | Follow | duration=1.0 | |
| 4 | AutoLine | enabled=false | message `/plan_status`: `a25_autoline_off` |
| 5 | Stop |  | message `/plan_status`: `a25_plan_stop` |

JSON nguồn:
```json
{
  "name": "a25",
  "start_without_cross": true,
  "steps": [
    {
      "label": "scan_right",
      "action": "RotateRight",
      "speed": 5,
      "until": "line",
      "min_duration": 1.5,
      "timeout": 6,
      "strict_line": true,
      "messages": [
        {
          "topic": "/plan_status",
          "message": "a25_scan_right"
        }
      ]
    },
    {
      "action": "AutoLine",
      "enabled": true,
      "messages": [
        {
          "topic": "/plan_status",
          "message": "a25_autoline_on"
        }
      ]
    },
    {
      "action": "Follow",
      "duration": 1.0
    },
    {
      "action": "AutoLine",
      "enabled": false,
      "messages": [
        {
          "topic": "/plan_status",
          "message": "a25_autoline_off"
        }
      ]
    },
    {
      "action": "Stop",
      "messages": [
        {
          "topic": "/plan_status",
          "message": "a25_plan_stop"
        }
      ]
    }
  ],
  "end_state": "stop"
}
```

**Gợi ý sử dụng nhanh**
- Hướng dẫn phòng có voice: `a20`.
- Đi thẳng qua giao cắt: `a21`.
- Rẽ phải: `a22`.
- Dừng robot: `a23`.
- Quay đầu: `a24`.
- Demo AutoLine + status: `a25`.
