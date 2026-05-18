# HospitalRobot Device Protocol v1 — role: camera
# Xem docs/DEVICE_PROTOCOL.md cho spec đầy đủ.

from machine import UART
from yolo_uno import *
from huskylens import HuskyLens, ALGORITHM_FACE_RECOGNITION
import time

try:
    import ujson as json
except ImportError:
    import json

# Delay đầu để board settle sau cold boot (reset cứng). Nếu bỏ delay này,
# neopix/UART có thể chưa ready -> emit() đụng neopix.show() -> crash silent.
time.sleep_ms(500)

DEV_ID = "hrbot_camera"
FW_NAME = "camera"
FW_VER = 1
BOOT_BANNER_COUNT = 10

LOOP_DELAY_MS = 100        # 10 Hz — đồng bộ với line_sensors
RECONNECT_INTERVAL_MS = 3000

# UART hardware giữa ESP32 và HuskyLens IC — giữ 9600 (mặc định HuskyLens,
# không phải USB CDC). KHÔNG đổi trừ khi đã đổi setting trên menu HuskyLens.
#
# LƯU Ý dual-baudrate:
# - USB CDC (kết nối xuống ROS, config trong config.json "serial.baudrate"): 115200
# - UART hardware (kết nối với HuskyLens IC): 9600 (hằng số này)
# Hai baudrate KHÁC NHAU — không nhầm lẫn khi cấu hình.
HUSKYLENS_BAUD = 9600

# KHÔNG khởi tạo UART ở module level — nếu fail sẽ chết im lặng trước khi boot
# banner được emit. Khởi tạo trong try/except sau khi boot banner đã in.
uart = None
hl = None

current_color = (0, 0, 0)

last_reconnect_ms = 0


def now_ms():
    return time.ticks_ms()


def elapsed_ms(start):
    return time.ticks_diff(now_ms(), start)


def set_led(color):
    global current_color
    current_color = color
    try:
        neopix.show(0, color)
    except Exception:
        pass  # neopix chưa ready hoặc lỗi — không phá luồng chính


def blink_tx(duration_ms=30):
    try:
        neopix.show(0, (255, 255, 255))
        time.sleep_ms(duration_ms)
        neopix.show(0, current_color)
    except Exception:
        pass


def emit(event, payload):
    """Gửi 1 envelope JSON theo schema HospitalRobot Device Protocol v1.
    print() phải chạy trước mọi side-effect khác (LED) — đảm bảo luôn có output."""
    print(json.dumps({
        "dev_id":  DEV_ID,
        "event":   event,
        "payload": payload,
    }))
    blink_tx()


def ensure_uart():
    """Khởi tạo UART + HuskyLens nếu chưa có. Trả True nếu OK."""
    global uart, hl
    if hl is not None:
        return True
    try:
        uart = UART(1, baudrate=HUSKYLENS_BAUD, tx=D4_PIN, rx=D3_PIN)
        hl = HuskyLens(uart)
        hl.debug = False
        return True
    except Exception as e:
        emit("error", {"code": "uart_init_failed", "msg": str(e)})
        return False


def connect_huskylens():
    if not ensure_uart():
        return False
    try:
        if not hl.knock():
            return False
        hl.set_alg(ALGORITHM_FACE_RECOGNITION)
        return True
    except Exception:
        return False


# Boot banner.
for _ in range(BOOT_BANNER_COUNT):
    emit("boot", {"fw": FW_NAME, "ver": FW_VER})

connected = connect_huskylens()
if connected:
    set_led((0, 0, 255))
    emit("info", {"msg": "huskylens_connected"})
    emit("info", {"msg": "alg_set", "alg": "face_recognition"})
else:
    set_led((255, 0, 0))
    emit("error", {"code": "no_connection"})


while True:
    if not connected:
        if elapsed_ms(last_reconnect_ms) >= RECONNECT_INTERVAL_MS:
            last_reconnect_ms = now_ms()
            connected = connect_huskylens()
            if connected:
                set_led((0, 0, 255))
                emit("info", {"msg": "huskylens_reconnected"})
            else:
                emit("error", {"code": "no_connection"})
        time.sleep_ms(200)
        continue

    try:
        blocks = hl.get_blocks() or []
    except Exception:
        connected = False
        set_led((255, 0, 0))
        emit("error", {"code": "read_fail"})
        time.sleep_ms(200)
        continue

    ids = []
    unknown_detected = False
    for b in blocks:
        fid = int(getattr(b, "ID", 0))
        if fid > 0:
            if fid not in ids:
                ids.append(fid)
        else:
            unknown_detected = True
    if not ids and unknown_detected:
        ids = [0]

    # Emit data MỖI loop iteration — đồng bộ rate với line/huskylens.
    if ids:
        set_led((0, 255, 0))
        emit("data", {"kind": "face", "ids": ids})
    else:
        set_led((255, 0, 0))
        emit("data", {"kind": "no_object"})

    time.sleep_ms(LOOP_DELAY_MS)
