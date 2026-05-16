from machine import UART
from yolo_uno import *
from huskylens import HuskyLens, ALGORITHM_FACE_RECOGNITION
import time

DEV_ID = "DEV1"

uart = UART(1, baudrate=9600, tx=D4_PIN, rx=D3_PIN)
hl = HuskyLens(uart)
hl.debug = False

current_color = (0, 0, 0)

FACE_SEND_INTERVAL_MS = 5000
NO_OBJECT_INTERVAL_MS = 7000
RECONNECT_INTERVAL_MS = 3000

last_face_sent_ms = 0
last_no_obj_ms = 0
last_reconnect_ms = 0
last_face_payload = ""


def now_ms():
    return time.ticks_ms()


def elapsed_ms(start):
    return time.ticks_diff(now_ms(), start)


def set_led(color):
    global current_color
    current_color = color
    neopix.show(0, color)


def blink_tx(duration=0.03):
    neopix.show(0, (255, 255, 255))
    time.sleep_ms(int(duration * 1000))
    neopix.show(0, current_color)


def usb_send_frame(*fields):
    payload = ",".join([DEV_ID] + [str(f) for f in fields])
    print("<{}>".format(payload))
    blink_tx()


def connect_huskylens():
    try:
        if not hl.knock():
            return False
        hl.set_alg(ALGORITHM_FACE_RECOGNITION)
        return True
    except Exception:
        return False


connected = connect_huskylens()
if connected:
    set_led((0, 0, 255))
    usb_send_frame("INFO", "HUSKYLENS_CONNECTED")
    usb_send_frame("INFO", "ALG_SET", "FACE_RECOGNITION")
else:
    set_led((255, 0, 0))
    usb_send_frame("ERR", "NO_CONNECTION")


while True:
    if not connected:
        if elapsed_ms(last_reconnect_ms) >= RECONNECT_INTERVAL_MS:
            last_reconnect_ms = now_ms()
            connected = connect_huskylens()
            if connected:
                set_led((0, 0, 255))
                usb_send_frame("INFO", "HUSKYLENS_RECONNECTED")
            else:
                usb_send_frame("ERR", "NO_CONNECTION")
        time.sleep_ms(200)
        continue

    try:
        blocks = hl.get_blocks() or []
    except Exception:
        connected = False
        set_led((255, 0, 0))
        usb_send_frame("ERR", "READ_FAIL")
        time.sleep_ms(200)
        continue

    # =========================
    # ID LOGIC:
    # - Nếu có face nhưng ID chưa có (ID=0) -> gửi 0
    # - Nếu có ID hợp lệ (>0) -> gửi danh sách ID
    # =========================
    ids = []
    unknown_detected = False

    for b in blocks:
        fid = int(getattr(b, "ID", 0))
        if fid > 0:
            if fid not in ids:
                ids.append(fid)
        else:
            unknown_detected = True

    # Nếu không có ID hợp lệ nhưng có phát hiện face -> gán 0
    if not ids and unknown_detected:
        ids = [0]

    if ids:
        set_led((0, 255, 0))
        # Gửi IDs dạng "1;2;3"
        face_payload = ";".join([str(x) for x in ids])

        if face_payload != last_face_payload or elapsed_ms(last_face_sent_ms) >= FACE_SEND_INTERVAL_MS:
            usb_send_frame("FACE", face_payload)
            last_face_payload = face_payload
            last_face_sent_ms = now_ms()
    else:
        set_led((255, 0, 0))
        last_face_payload = ""
        if elapsed_ms(last_no_obj_ms) >= NO_OBJECT_INTERVAL_MS:
            usb_send_frame("NO_OBJECT")
            last_no_obj_ms = now_ms()

    time.sleep_ms(100)
