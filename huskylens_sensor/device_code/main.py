# Import các module phần cứng:
# - Pin, UART: giao tiếp UART với HuskyLens
# - HuskyLens, ALGORITHM_LINE_TRACKING: thư viện HuskyLens và chế độ dò line
from machine import Pin, UART
from yolo_uno import D4_PIN, D3_PIN
from huskylens import HuskyLens, ALGORITHM_LINE_TRACKING
import time
import math

# Ưu tiên ujson vì nhẹ hơn trên MicroPython
try:
    import ujson as json
except ImportError:
    import json


# ================== CONFIG ==================
# UART số mấy trên board
UART_ID = 1

# Chân TX/RX dùng để nối với HuskyLens
# TX board -> RX HuskyLens
# RX board <- TX HuskyLens
# Nếu board không có D4_PIN / D3_PIN thì thay bằng GPIO thực tế
UART_TX_PIN = D4_PIN
UART_RX_PIN = D3_PIN

# Baudrate UART hardware giữa ESP32 và HuskyLens IC — giữ 9600 (mặc định HuskyLens,
# không phải USB CDC). KHÔNG đổi trừ khi đã đổi setting trên menu HuskyLens.
UART_BAUDRATE = 9600

# Kích thước ảnh xử lý của HuskyLens
IMAGE_WIDTH = 320
IMAGE_HEIGHT = 240

# Trục giữa ảnh theo X
IMAGE_CENTER_X = IMAGE_WIDTH // 2   # 160

# Cắt bớt mép trái/phải để giảm nhiễu ở rìa ảnh
X_CROP_LEFT = 20
X_CROP_RIGHT = 20

# Nếu y_tail nằm sát đáy ảnh thì coi là line gần xe
Y_BOTTOM_THRESHOLD = IMAGE_HEIGHT - 20

# Độ dài line tối thiểu theo trục Y để coi là hợp lệ
MIN_LINE_LENGTH_Y = 50

# y_tail phải đủ thấp trong ảnh thì mới coi là line đáng tin
MIN_Y_START_FOR_VALID_LINE = 170

# Nếu tail lệch quá lớn thì bỏ qua
MAX_ABS_TAIL_OFFSET_X = 110

# Thời gian thử kết nối lại HuskyLens khi mất kết nối
RETRY_INTERVAL_MS = 100

# Chu kỳ lặp 100ms — 10Hz, đồng bộ với line/camera.
LOOP_DELAY_MS = 100

# Giá trị direction mặc định
DEFAULT_DIRECTION = 0

# Phân loại line theo vị trí trên ảnh
Y_TYPE_UNKNOWN = 0
Y_TYPE_BOTTOM_TO_MID = 1
Y_TYPE_MID_TO_TOP = 2
# ============================================


def print_json(data):
    """
    In dữ liệu ra dạng JSON để thiết bị khác dễ đọc.
    """
    try:
        print(json.dumps(data))
    except Exception as e:
        print('{"error":"json_dump_failed","detail":"%s"}' % str(e))


def clamp(value, min_value, max_value):
    """
    Giới hạn value trong khoảng [min_value, max_value].
    """
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def no_line_tracking():
    """
    Trạng thái mặc định khi:
    - không thấy line
    - dữ liệu line không hợp lệ
    """
    return {
        "valid": 0,
        "direction": DEFAULT_DIRECTION,
        "tail_offset_x": 0,
        "y_type": Y_TYPE_UNKNOWN,
        "line_length_y": 0,
        "angle_deg": 0.0
    }


def classify_line_y(y_head, y_tail):
    """
    Phân loại line theo vị trí trên ảnh.

    Trả về:
    - Y_TYPE_UNKNOWN: line không hợp lệ
    - Y_TYPE_BOTTOM_TO_MID: line gần đáy ảnh
    - Y_TYPE_MID_TO_TOP: line ở cao hơn
    """
    y_start = int(y_tail)
    y_end = int(y_head)
    line_length_y = abs(y_tail - y_head)

    # Line quá ngắn
    if line_length_y < MIN_LINE_LENGTH_Y:
        return Y_TYPE_UNKNOWN

    # Head phải nằm phía trên tail
    if y_end >= y_start:
        return Y_TYPE_UNKNOWN

    # Tail phải đủ thấp trong ảnh
    if y_start < MIN_Y_START_FOR_VALID_LINE:
        return Y_TYPE_UNKNOWN

    # Nếu tail rất gần đáy ảnh
    if y_start >= Y_BOTTOM_THRESHOLD:
        return Y_TYPE_BOTTOM_TO_MID

    # Còn lại là line ở cao hơn
    return Y_TYPE_MID_TO_TOP


def calc_line_angle_from_tail_to_head(x_tail, y_tail, x_head, y_head):
    """
    Tính góc của vector từ tail -> head so với trục thẳng đứng hướng lên.

    Quy ước:
    - angle = 0   : line thẳng
    - angle < 0   : line nghiêng trái
    - angle > 0   : line nghiêng phải

    Vì hệ trục ảnh có y tăng xuống dưới nên:
    - dx = x_head - x_tail
    - dy_up = y_tail - y_head
    """
    dx = int(x_head) - int(x_tail)
    dy_up = int(y_tail) - int(y_head)

    if dx == 0 and dy_up == 0:
        return 0.0

    angle_rad = math.atan2(dx, dy_up)
    angle_deg = angle_rad * 180.0 / math.pi
    return angle_deg


def arrow_to_line_data(arrow):
    """
    Chuyển dữ liệu arrow từ HuskyLens thành dữ liệu gọn cho thuật toán dò line.

    Output:
    - valid
    - direction
    - tail_offset_x: độ dời của tail theo trục X so với tâm ảnh (160)
    - y_type
    - line_length_y
    - angle_deg: góc của vector tail -> head
    """
    if arrow is None:
        return no_line_tracking()

    # Lấy dữ liệu từ arrow
    x_tail = getattr(arrow, "x_tail", None)
    y_tail = getattr(arrow, "y_tail", None)
    x_head = getattr(arrow, "x_head", None)
    y_head = getattr(arrow, "y_head", None)
    direction = getattr(arrow, "direction", DEFAULT_DIRECTION)

    # Thiếu dữ liệu thì bỏ
    if x_tail is None or x_head is None or y_tail is None or y_head is None:
        return no_line_tracking()

    # Ép kiểu int
    x_tail = int(x_tail)
    y_tail = int(y_tail)
    x_head = int(x_head)
    y_head = int(y_head)

    # Tính độ dài line theo trục Y
    line_length_y = abs(y_tail - y_head)

    # Line quá ngắn
    if line_length_y < MIN_LINE_LENGTH_Y:
        return no_line_tracking()

    # Head phải nằm trên tail
    if y_head >= y_tail:
        return no_line_tracking()

    # Tail phải nằm đủ gần đáy ảnh
    if y_tail < MIN_Y_START_FOR_VALID_LINE:
        return no_line_tracking()

    # Giới hạn x_tail trong vùng ảnh hợp lệ
    x_min_limit = X_CROP_LEFT
    x_max_limit = IMAGE_WIDTH - X_CROP_RIGHT
    if x_max_limit <= x_min_limit:
        return no_line_tracking()

    x_tail = clamp(x_tail, x_min_limit, x_max_limit)
    x_head = clamp(x_head, x_min_limit, x_max_limit)

    # Độ dời của tail theo trục X so với tâm ảnh 160
    # tail_offset_x < 0: tail lệch trái
    # tail_offset_x > 0: tail lệch phải
    tail_offset_x = x_tail - IMAGE_CENTER_X

    # Tail lệch quá lớn thì loại
    if abs(tail_offset_x) > MAX_ABS_TAIL_OFFSET_X:
        return no_line_tracking()

    # Phân loại line theo trục Y
    y_type = classify_line_y(y_head, y_tail)
    if y_type == Y_TYPE_UNKNOWN:
        return no_line_tracking()

    # Tính góc line từ tail -> head
    angle_deg = calc_line_angle_from_tail_to_head(x_tail, y_tail, x_head, y_head)

    return {
        "valid": 1,
        "direction": int(direction if direction is not None else DEFAULT_DIRECTION),
        "tail_offset_x": int(tail_offset_x),
        "y_type": int(y_type),
        "line_length_y": int(line_length_y),
        "angle_deg": float(angle_deg)
    }


DEV_ID = "hrbot_huskylens"
FW_NAME = "huskylens"
FW_VER = 1


def emit(event, payload):
    """Gửi 1 envelope JSON theo schema HospitalRobot Device Protocol v1."""
    print(json.dumps({
        "dev_id":  DEV_ID,
        "event":   event,
        "payload": payload,
    }))


def build_data_payload(line_tracking, connected, algorithm_set):
    return {
        "connected":     1 if connected else 0,
        "algorithm_set": 1 if algorithm_set else 0,
        "valid":         int(line_tracking["valid"]),
        "tail_offset_x": int(line_tracking["tail_offset_x"]),
        "y_type":        int(line_tracking["y_type"]),
        "line_length_y": int(line_tracking["line_length_y"]),
        "direction":     int(line_tracking["direction"]),
        "angle_deg":     float(line_tracking["angle_deg"]),
    }


def make_uart_and_hl():
    """
    Tạo UART và khởi tạo HuskyLens.
    """
    uart = UART(
        UART_ID,
        baudrate=UART_BAUDRATE,
        tx=Pin(UART_TX_PIN),
        rx=Pin(UART_RX_PIN)
    )
    time.sleep(0.2)
    hl = HuskyLens(uart, debug=False)
    return uart, hl


# ================== INIT ==================
uart = None
hl = None
connected = False
algorithm_set = False
last_retry_ms = 0
# ==========================================

# Boot banner — 10 lần để probe (3s window) match.
for _ in range(10):
    emit("boot", {"fw": FW_NAME, "ver": FW_VER})

while True:
    now = time.ticks_ms()

    # Nếu chưa kết nối, thử kết nối lại theo chu kỳ
    if (not connected) and time.ticks_diff(now, last_retry_ms) >= RETRY_INTERVAL_MS:
        last_retry_ms = now
        try:
            uart, hl = make_uart_and_hl()
            connected = bool(hl.knock())
            algorithm_set = False
        except Exception:
            connected = False
            algorithm_set = False
            emit("error", {"code": "uart_init_failed"})

    # Nếu đã kết nối nhưng chưa set thuật toán line tracking
    if connected and (not algorithm_set):
        try:
            connected = bool(hl.set_alg(ALGORITHM_LINE_TRACKING))
            algorithm_set = connected
        except Exception:
            connected = False
            algorithm_set = False

    # Mặc định là chưa có line
    line_tracking = no_line_tracking()

    # Chỉ đọc line khi đã kết nối và set thuật toán thành công
    if connected and algorithm_set:
        try:
            try:
                # Một số thư viện yêu cầu truyền algorithm
                arrows = hl.get_arrows(ALGORITHM_LINE_TRACKING)
            except TypeError:
                # Một số thư viện không cần truyền tham số
                arrows = hl.get_arrows()

            # Nếu có line thì lấy arrow đầu tiên
            if arrows and len(arrows) > 0:
                candidate_arrow = arrows[0]
                line_tracking = arrow_to_line_data(candidate_arrow)

        except Exception:
            connected = False
            algorithm_set = False
            line_tracking = no_line_tracking()

    # Xuất dữ liệu ra ngoài
    emit("data", build_data_payload(line_tracking, connected, algorithm_set))

    # Lặp mỗi 50ms
    time.sleep_ms(LOOP_DELAY_MS)
