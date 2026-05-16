# HospitalRobot Project Flow

## 1. Tổng quan

Hệ thống chạy theo kiến trúc ROS2 nhiều node. Thiết bị phần cứng xuất dữ liệu qua serial/USB, ROS node đọc và chuẩn hóa, sau đó các node điều khiển quyết định lệnh motor.

```text
Device firmware
    ↓ serial / USB
ROS sensor nodes
    ↓ ROS topics
line_follower / manual_control / mqtt_bridge
    ↓ /motor_cmd
motor_driver
    ↓ serial motor protocol
Motor controller
```

4 lớp chính:

- **Device layer**: code chạy trên board/cảm biến (`device_code/main.py`).
- **Sensor ROS layer**: đọc serial, parse, normalize, publish ROS topic.
- **Decision/control layer**: `line_follower`, `manual_control`, plan/FSM.
- **Integration/actuator layer**: `mqtt_bridge`, `motor_driver`.

## 2. Flow camera face-recognition

### 2.1. Device camera

File:

```text
camera_sensor/device_code/main.py
```

Thiết bị:

1. Mở UART tới HuskyLens.
2. Set algorithm `ALGORITHM_FACE_RECOGNITION`.
3. Đọc `hl.get_blocks()`.
4. Gom face ID.
5. In frame qua serial.

Output raw có thể là:

```text
<DEV1,INFO,HUSKYLENS_CONNECTED>
<DEV1,INFO,ALG_SET,FACE_RECOGNITION>
<DEV1,ERR,NO_CONNECTION>
<DEV1,ERR,READ_FAIL>
<DEV1,FACE,0>
<DEV1,FACE,1>
<DEV1,FACE,1;2;3>
<DEV1,NO_OBJECT>
```

Ý nghĩa:

- `FACE,0`: có face nhưng chưa learn ID.
- `FACE,1`: thấy face ID 1.
- `FACE,1;2;3`: thấy nhiều face ID.
- `NO_OBJECT`: không thấy face.

### 2.2. ROS node camera

File:

```text
camera_sensor/camera_sensor/main.py
```

Config:

```text
robot_common/robot_common/config.json -> camera_sensor
baudrate = 9600
publish topic = /face/camera
```

Luồng xử lý:

```text
camera device serial
    ↓
CameraSensorReader.read_line()
    ↓
CameraSensorNode._normalize_face_payload()
    ↓
/face/camera
```

Normalize:

- bỏ ký tự nhiễu
- uppercase
- nhận frame có/không có `< >`
- normalize device token về `DEV1`
- normalize state về `FACE` hoặc `NO_OBJECT`
- giữ nhiều face ID

Ví dụ:

```text
<DEV1,FACE,1;2;3>  ->  <DEV1,FACE,1,2,3>
<DEV1,FACE,1|2/3>  ->  <DEV1,FACE,1,2,3>
<DEV1,NO_OBJECT>   ->  <DEV1,NO_OBJECT>
```

### 2.3. Camera sang MQTT/plan message

File:

```text
mqtt_bridge/mqtt_bridge/MQTTBridgeROS.py
```

`mqtt_bridge` subscribe:

```text
/face/camera
```

Có 2 nhánh:

```text
/face/camera
    ├─ forward raw payload -> MQTT face/camera
    └─ extract face IDs -> MQTT plan_message
```

Với payload:

```text
<DEV1,FACE,1,2,3>
```

Bridge sẽ lấy IDs:

```text
[1, 2, 3]
```

Sau đó tra mapping:

```text
robot_common/robot_common/config.json -> mqtt_bridge.camera_face_plan_message.id_messages
```

Nếu ID có message tương ứng, bridge publish ra MQTT topic:

```text
plan_message
```

Gate quan trọng:

- nếu `require_plan_autoline=true`, camera chỉ kích hoạt `plan_message` khi:
  - đang có `_active_plan_name`
  - `_plan_autoline_active=true`

## 3. Flow line_sensors

### 3.1. Device line_sensors

File:

```text
line_sensors/device_code/main.py
```

Thiết bị:

1. Mở I2C bus.
2. Scan địa chỉ sensor.
3. Với mỗi địa chỉ, đọc 1 byte.
4. Tách bit thành `s1..s4`.
5. In JSON qua serial.

Output raw:

```json
{
  "LineSensor": {
    "0x23": {"s1": 0, "s2": 1, "s3": 0, "s4": 1},
    "0x24": {"s1": 1, "s2": 1, "s3": 0, "s4": 0},
    "0x25": {"s1": 1, "s2": 1, "s3": 1, "s4": 1}
  }
}
```

Code device hiện chạy loop đồng bộ:

```text
print(build_linesensor_json())
time.sleep_ms(100)
```

Không còn phụ thuộc `asleep_ms`, `create_task`, `run_loop`.

### 3.2. ROS node line_sensors

Files:

```text
line_sensors/line_sensors/line_sensor_reader.py
line_sensors/line_sensors/line_sensor_driver_node.py
```

Config:

```text
robot_common/robot_common/config.json -> line_sensors
port = /dev/ttyACM0
baudrate = 115200
publish topic = /line_sensors/frame
```

Address map bắt buộc:

```text
0x25 -> left
0x24 -> middle
0x23 -> right
```

Parse flow:

```text
line sensor serial JSON
    ↓
LineSensorReader.parse_payload()
    ↓
normalize counts/full flags
    ↓
LineSensorDriverNode
    ↓
/line_sensors/frame
```

Output ROS:

```text
/line_sensors/frame : std_msgs/Int16MultiArray
```

Layout cố định:

```text
[left_count, mid_count, right_count, left_full, mid_full, right_full]
```

Ví dụ:

```text
[4, 2, 0, 1, 0, 0]
```

Active predicate:

```text
active = 1 hoặc "1" hoặc True
```

Cả count và full-black dùng chung predicate này:

```text
left_count = số channel active bên trái
left_full = tất cả channel bên trái active
```

Zero-frame filter:

- Nếu frame toàn zero ngắn hạn, node giữ frame non-zero gần nhất.
- Mục tiêu: chống nhiễu/mất frame tức thời.

## 4. Flow HuskyLens line-tracking

### 4.1. Device HuskyLens line-tracking

File:

```text
huskylens_sensor/device_code/main.py
```

Thiết bị:

1. Mở UART tới HuskyLens.
2. Set algorithm `ALGORITHM_LINE_TRACKING`.
3. Đọc `hl.get_arrows()`.
4. Lấy arrow đầu tiên.
5. Validate line.
6. Tính thông số tracking.
7. In JSON qua serial.

Output raw:

```json
{
  "HuskylenSensor": {
    "connected": 1,
    "algorithm_set": 1,
    "valid": 1,
    "tail_offset_x": -16,
    "y_type": 1,
    "line_length_y": 84,
    "direction": 0,
    "angle_deg": 3.7
  }
}
```

Field chính:

- `connected`: đã kết nối HuskyLens.
- `algorithm_set`: đã set line-tracking algorithm.
- `valid`: line hợp lệ.
- `tail_offset_x`: tail lệch X so với tâm ảnh.
- `angle_deg`: góc line tail → head.
- `y_type`: phân loại vị trí line theo trục Y.
- `line_length_y`: độ dài line theo Y.
- `direction`: direction raw từ HuskyLens.

### 4.2. ROS node HuskyLens

Files:

```text
huskylens_sensor/huskylens_sensor/huskylens_sensor_reader.py
huskylens_sensor/huskylens_sensor/huskylens_parser.py
huskylens_sensor/huskylens_sensor/huskylens_sensor_node.py
```

Config:

```text
robot_common/robot_common/config.json -> huskylens_sensor
baudrate = 9600
publish topics = /huskylens/frame, /huskylens/valid
```

Parse flow:

```text
HuskyLens serial JSON
    ↓
normalize_huskylens_payload()
    ↓
compute error nếu cần
    ↓
/huskylens/frame
/huskylens/valid
```

Parser chấp nhận key:

```text
HuskylenSensor
HuskyLensSensor
```

Required fields:

```text
connected
algorithm_set
valid
```

Nếu có `tail_offset_x` + `angle_deg`, parser tự tính thêm:

```text
error = round(0.8 * tail_offset_x + 0.2 * angle_deg)
```

Output ROS `/huskylens/frame`:

```json
{
  "HuskylenSensor": {
    "connected": 1,
    "algorithm_set": 1,
    "valid": 1,
    "tail_offset_x": -16.0,
    "angle_deg": 3.7,
    "error": -12,
    "y_type": 1,
    "line_length_y": 84,
    "direction": 0
  }
}
```

Output ROS `/huskylens/valid`:

```text
true khi connected=1 && algorithm_set=1 && valid=1
false trong các trường hợp còn lại
```

## 5. Flow line_follower auto tracking

Files:

```text
line_follower/line_follower/line_follower_node.py
line_follower/line_follower/line_follower.py
```

Input topics:

```text
/line_sensors/frame
/huskylens/frame
/huskylens/valid
/plan_select
/pick_robot
/auto_mode
```

Output topics/service:

```text
/motor_cmd
/plan_status
/plan_callback
/plan_message
/set_auto_mode
```

Tracking flow chính:

```text
/huskylens/frame
    ↓
line_follower parse tail_offset_x + angle_deg
    ↓
tracking strategy / FSM
    ↓
format motor command
    ↓
/motor_cmd
```

Line sensor flow phụ/fallback:

```text
/line_sensors/frame
    ↓
line_follower parse counts/full flags
    ↓
cross detect / fallback tracking
    ↓
/motor_cmd
```

Theo config hiện tại, tracking ưu tiên HuskyLens:

```text
line_follower.tracking.strategy = huskylens
line_follower.tracking.strict_mode = true
line_follower.tracking.only_huskylens = true
```

Điều khiển HuskyLens dùng:

```text
tail_offset_x
angle_deg
```

Ý nghĩa đơn giản:

- tail lệch trái/phải → chỉnh hướng.
- góc nghiêng trái/phải → rotate hoặc steer.
- line hợp lệ và gần tâm → forward.
- frame invalid/stale → stop hoặc fallback tùy config.

## 6. Flow manual_control

File:

```text
manual_control/manual_control/manual_control_node.py
```

Input:

```text
/VR_control
/pick_robot
```

Output:

```text
/motor_cmd
/auto_mode
/set_auto_mode
```

Manual flow:

```text
MQTT / keyboard / app
    ↓
/VR_control
    ↓
manual_control
    ↓
/motor_cmd
    ↓
motor_driver
```

Command ví dụ:

```text
Forward
Backward
Left
Right
RotateLeft
RotateRight
Stop
```

Auto mode flow:

```text
/pick_robot = 1
    ↓
manual_control bật auto
    ↓
/auto_mode true
    ↓
line_follower chạy auto
```

```text
/pick_robot = 0
    ↓
manual_control tắt auto
    ↓
/auto_mode false
    ↓
line_follower stop
```

Nếu có manual input và config cho phép override, `manual_control` có thể tắt auto để ưu tiên điều khiển tay.

## 7. Flow plan execution

Plan nằm trong:

```text
robot_common/robot_common/plans/*.json
```

Chọn plan qua:

```text
/plan_select
```

Flow:

```text
MQTT plan_select / keyboard
    ↓
mqtt_bridge normalize plan command
    ↓
/plan_select
    ↓
line_follower load plan
    ↓
execute steps/actions
    ↓
/plan_status /plan_callback /plan_message
```

Plan có thể điều khiển:

- follow line
- rotate
- stop
- bật/tắt autoline
- publish message
- kết thúc bằng stop hoặc quay lại follow

Status flow:

```text
line_follower
    ↓
/plan_status
    ↓
mqtt_bridge
    ↓
MQTT plan_status
```

Message flow:

```text
line_follower
    ↓
/plan_message
    ↓
mqtt_bridge
    ↓
MQTT plan_message
```

## 8. Flow MQTT bridge

File:

```text
mqtt_bridge/mqtt_bridge/MQTTBridgeROS.py
```

### 8.1. MQTT → ROS

```text
MQTT VR_control
    ↓
/VR_control
```

```text
MQTT pick_robot
    ↓
/pick_robot
```

```text
MQTT plan_select
    ↓
normalize room/plan alias
    ↓
/plan_select
```

Plan command hỗ trợ:

```text
room:a19
room:1
1
plan:a19
clear
room:0
```

### 8.2. ROS → MQTT

```text
/face/camera      -> MQTT face/camera
/plan_status      -> MQTT plan_status
/plan_message     -> MQTT plan_message
/huskylens/frame  -> MQTT huskylens/frame
/huskylens/valid  -> MQTT huskylens/valid
/rosout filtered  -> MQTT robot_logs
```

### 8.3. Keyboard local

`mqtt_bridge` cũng đọc phím để test nhanh:

```text
w/s/a/d/space -> movement
j/p           -> rotate
k             -> toggle auto
number keys   -> select plan
q             -> quit
```

Keyboard event được đưa vào queue nội bộ rồi xử lý trong ROS thread để tránh publish ROS từ worker thread.

## 9. Flow motor_driver

File:

```text
motor_driver/motor_driver/motor_controller.py
```

Input:

```text
/motor_cmd
```

Command format:

```text
Direction:Speed
```

Ví dụ:

```text
Forward:8
Left:6
Right:6
RotateLeft:5
RotateRight:5
Stop:0
```

Flow:

```text
/motor_cmd
    ↓
motor_driver
    ↓
format/check command protocol
    ↓
serial to motor controller
    ↓
motor chạy
```

`motor_driver` là điểm cuối duy nhất gửi lệnh xuống actuator.

## 10. Flow tổng hợp end-to-end

### 10.1. Auto HuskyLens tracking

```text
huskylens_sensor/device_code/main.py
    ↓ serial JSON
huskylens_sensor ROS node
    ↓ /huskylens/frame + /huskylens/valid
line_follower
    ↓ /motor_cmd
motor_driver
    ↓ serial
motor controller
```

### 10.2. Line sensor tracking/fallback

```text
line_sensors/device_code/main.py
    ↓ serial JSON LineSensor
line_sensors ROS node
    ↓ /line_sensors/frame
line_follower
    ↓ /motor_cmd
motor_driver
    ↓ serial
motor controller
```

### 10.3. Manual control

```text
MQTT app / keyboard
    ↓ VR_control
mqtt_bridge
    ↓ /VR_control
manual_control
    ↓ /motor_cmd
motor_driver
    ↓ serial
motor controller
```

### 10.4. Select plan from app/UI

```text
MQTT plan_select
    ↓
mqtt_bridge
    ↓ /plan_select
line_follower
    ↓ execute plan
/plan_status + /plan_message
    ↓
mqtt_bridge
    ↓
MQTT plan_status + plan_message
```

### 10.5. Face-trigger plan message

```text
camera_sensor/device_code/main.py
    ↓ <DEV1,FACE,1;2;3>
camera_sensor ROS node
    ↓ <DEV1,FACE,1,2,3> on /face/camera
mqtt_bridge
    ↓ extract IDs [1,2,3]
lookup id_messages
    ↓
MQTT plan_message
```

## 11. Topic map nhanh

### Sensor topics

```text
/line_sensors/frame   std_msgs/Int16MultiArray
/huskylens/frame      std_msgs/String
/huskylens/valid      std_msgs/Bool
/face/camera          std_msgs/String
```

### Control topics

```text
/VR_control           std_msgs/String
/pick_robot           std_msgs/String
/auto_mode            std_msgs/Bool
/plan_select          std_msgs/String
/motor_cmd            std_msgs/String
```

### Status topics

```text
/plan_status          std_msgs/String
/plan_callback        std_msgs/String
/plan_message         std_msgs/String
/debug_logs_toggle    std_msgs/Bool
```

### MQTT topics

```text
VR_control
pick_robot
plan_select
face/camera
plan_status
plan_message
huskylens/frame
huskylens/valid
robot_logs
```

## 12. Serial/device contract quan trọng

### Camera

```text
Device baudrate: 9600
Raw:        <DEV1,FACE,1;2;3>
ROS output: <DEV1,FACE,1,2,3>
Topic:      /face/camera
```

### Line sensors

```text
Device baudrate: 115200
Raw key: LineSensor
Address map:
  0x25 = left
  0x24 = middle
  0x23 = right
Topic: /line_sensors/frame
```

### HuskyLens line-tracking

```text
Device baudrate: 9600
Raw key: HuskylenSensor
Fields: tail_offset_x, angle_deg, valid, connected, algorithm_set
ROS adds: error
Topics: /huskylens/frame, /huskylens/valid
```

## 13. Điểm dễ lỗi khi chạy thực tế

1. **Sai port `/dev/ttyACM*`**
   - Camera, line sensor, HuskyLens đều dùng serial.
   - Cần kiểm tra device nào đang nằm ở port nào.

2. **Sai baudrate**
   - Camera: `9600`.
   - HuskyLens line-tracking: `9600`.
   - Line sensors: `115200`.

3. **Line sensor I2C address không đúng**
   - ROS đang map cứng `0x25/0x24/0x23`.
   - Nếu hardware scan ra địa chỉ khác, frame sẽ toàn 0 hoặc thiếu bên.

4. **HuskyLens sai algorithm**
   - Camera device dùng `ALGORITHM_FACE_RECOGNITION`.
   - HuskyLens line device dùng `ALGORITHM_LINE_TRACKING`.
   - Một HuskyLens tại một thời điểm chỉ nên chạy một algorithm.

5. **Multi-face payload**
   - Device gửi `1;2;3`.
   - ROS camera normalize thành `1,2,3`.
   - MQTT bridge đọc được nhiều ID.

6. **Face-trigger bị gate bởi autoline**
   - Nếu `camera_face_plan_message.require_plan_autoline=true`, face ID không publish `plan_message` khi chưa có active plan/autoline.

7. **Hai node cùng mở một serial port**
   - Camera và line sensors không được cùng trỏ về một `/dev/ttyACM*` nếu là hai thiết bị khác nhau.

## 14. Lệnh kiểm tra nhanh

### Camera

```bash
ros2 run camera_sensor camera_sensor
ros2 topic echo /face/camera
```

Expected:

```text
<DEV1,FACE,1>
<DEV1,FACE,1,2,3>
<DEV1,NO_OBJECT>
```

### Line sensors

```bash
ros2 run line_sensors line_sensor_driver
ros2 topic echo /line_sensors/frame
```

Expected:

```text
data: [left_count, mid_count, right_count, left_full, mid_full, right_full]
```

### HuskyLens

```bash
ros2 run huskylens_sensor huskylens_sensor
ros2 topic echo /huskylens/frame
ros2 topic echo /huskylens/valid
```

Expected:

```text
HuskylenSensor.connected = 1
HuskylenSensor.algorithm_set = 1
HuskylenSensor.valid = 1
HuskylenSensor.tail_offset_x exists
HuskylenSensor.angle_deg exists
HuskylenSensor.error exists
```

### Motor command

```bash
ros2 topic echo /motor_cmd
```

Expected examples:

```text
Forward:8
Left:6
RotateRight:5
Stop:0
```

### MQTT bridge

```bash
ros2 run mqtt_bridge mqtt_bridge_ros2
```

Test MQTT:

```bash
mosquitto_pub -h 127.0.0.1 -p 1883 -t pick_robot -m "1"
mosquitto_pub -h 127.0.0.1 -p 1883 -t plan_select -m "room:1"
mosquitto_sub -h 127.0.0.1 -p 1883 -t plan_status
mosquitto_sub -h 127.0.0.1 -p 1883 -t plan_message
```

## 15. Tóm tắt một dòng

```text
Device sensors -> ROS sensor nodes -> line_follower/manual_control -> /motor_cmd -> motor_driver -> motor controller, with mqtt_bridge connecting app/UI to ROS and forwarding robot status back out.
```
