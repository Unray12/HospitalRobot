from machine import I2C
import time
import ujson

# Boot banner cho host probe (init_serial_symlinks.py) nhận dạng role.
ROLE_BANNER = "<HRBOT:LINE>"
HEARTBEAT_INTERVAL_MS = 2000

i2c = I2C(0, freq=100000)

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

def build_linesensor_json():
    try:
        addrs = i2c.scan()
    except:
        addrs = []

    sensors = {}
    for addr in addrs:
        sensors["0x%02X" % addr] = read_s4(addr)

    payload = {"LineSensor": sensors}
    return ujson.dumps(payload)

# Banner ngay khi boot — emit sớm để probe latch được.
print(ROLE_BANNER)
print(ROLE_BANNER)
print("App started")

last_banner_ms = time.ticks_ms()

while True:
    now = time.ticks_ms()
    if time.ticks_diff(now, last_banner_ms) >= HEARTBEAT_INTERVAL_MS:
        print(ROLE_BANNER)
        last_banner_ms = now
    print(build_linesensor_json())
    time.sleep_ms(100)
