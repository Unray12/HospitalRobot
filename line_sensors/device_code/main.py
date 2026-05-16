# HospitalRobot Device Protocol v1 — role: line
# Xem docs/DEVICE_PROTOCOL.md cho spec đầy đủ.

from machine import I2C
import time
import ujson

DEV_ID = "hrbot_line"
FW_NAME = "line"
FW_VER = 1
BOOT_BANNER_COUNT = 10
LOOP_INTERVAL_MS = 100

i2c = I2C(0, freq=100000)


def emit(event, payload):
    """Gửi 1 envelope JSON theo schema HospitalRobot Device Protocol v1."""
    print(ujson.dumps({
        "dev_id":  DEV_ID,
        "event":   event,
        "payload": payload,
    }))


def read_s4(addr):
    try:
        b = i2c.readfrom(addr, 1)[0]
        return {
            "s1": (b >> 0) & 1,
            "s2": (b >> 1) & 1,
            "s3": (b >> 2) & 1,
            "s4": (b >> 3) & 1,
        }
    except:
        return {"s1": -1, "s2": -1, "s3": -1, "s4": -1}


def build_data_payload():
    try:
        addrs = i2c.scan()
    except:
        addrs = []
    sensors = {}
    for addr in addrs:
        sensors["0x%02X" % addr] = read_s4(addr)
    return {"sensors": sensors}


# Boot banner.
for _ in range(BOOT_BANNER_COUNT):
    emit("boot", {"fw": FW_NAME, "ver": FW_VER})

while True:
    emit("data", build_data_payload())
    time.sleep_ms(LOOP_INTERVAL_MS)
