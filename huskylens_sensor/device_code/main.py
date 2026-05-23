"""
HuskyLens line-tracking firmware for the HospitalRobot sensor node.

Reads the ARROW output of the HuskyLens line-tracking algorithm over UART,
filters it into a compact decision payload, and streams the result as JSON
envelopes at 10 Hz on stdout for the upstream host (MQTT bridge / line follower).

Protocol: HospitalRobot Device Protocol v1 (see Note.txt).
Target:   MicroPython on YoloUNO (ESP32-class).
"""

from machine import UART, WDT
from yolo_uno import *
from huskylens import HuskyLens, ALGORITHM_LINE_TRACKING
import time
import math

# ujson is the MicroPython-native, lower-footprint encoder; fall back to json
# on CPython for off-device unit testing.
try:
    import ujson as json
except ImportError:
    import json

# Settle delay sau cold boot — neopix/UART có thể chưa ready ngay khi reset
# cứng. Giống pattern trong camera_sensor/device_code/main.py.
time.sleep_ms(500)


# ---------------------------------------------------------------------------
# Hardware configuration
# ---------------------------------------------------------------------------

UART_ID       = 1
UART_TX_PIN   = D4_PIN           # board TX -> HuskyLens RX
UART_RX_PIN   = D3_PIN           # board RX <- HuskyLens TX

# HuskyLens IC defaults to 9600 baud on its hardware UART (independent of the
# USB-CDC link). Only change if the rate was reconfigured from the device menu.
UART_BAUDRATE = 9600


# ---------------------------------------------------------------------------
# Image geometry (HuskyLens default frame 320x240)
# ---------------------------------------------------------------------------

IMAGE_WIDTH    = 320
IMAGE_HEIGHT   = 240
IMAGE_CENTER_X = IMAGE_WIDTH // 2   # 160

# Horizontal dead zone: drop edge artifacts before computing tail offset.
X_CROP_LEFT  = 20
X_CROP_RIGHT = 20

# Vertical zone thresholds. The frame is partitioned into three bands:
#   TOP : y < MID_Y_THRESHOLD                     (far away from the robot)
#   MID : MID_Y_THRESHOLD <= y < Y_BOTTOM_THRESHOLD
#   BOT : y >= Y_BOTTOM_THRESHOLD                 (closest to the robot)
MID_Y_THRESHOLD    = 80
Y_BOTTOM_THRESHOLD = IMAGE_HEIGHT - 20   # 220

# Maximum tail deviation (in pixels) from the image center that we will still
# steer on. Beyond this the arrow is treated as spurious.
MAX_ABS_TAIL_OFFSET_X = 110


# ---------------------------------------------------------------------------
# Runtime configuration
# ---------------------------------------------------------------------------

RETRY_INTERVAL_MS = 100   # backoff between HuskyLens reconnect attempts
LOOP_DELAY_MS     = 100   # main loop period; 10 Hz matches the line follower
DEFAULT_DIRECTION = 0

# Hardware watchdog. Sized for the worst-case stage time:
#   - boot banner: 10 × 150 ms = 1.5 s (fed every iteration)
#   - HuskyLens detect_version on cold start: up to ~2 s (V2 then V1 probe)
#   - Stage C read_arrows has its own 500 ms ceiling (see main loop)
# 8 s leaves comfortable headroom while still recovering from a true freeze
# (e.g. driver internal hang) within ~8 s.
WDT_TIMEOUT_MS = 8000


# ---------------------------------------------------------------------------
# Line classification codes (y_type)
#
# Naming convention X_TO_Y reads as "tail in X band, head in Y band"; the
# arrow always points from tail (bottom) to head (top) in image space.
# ---------------------------------------------------------------------------

Y_TYPE_NO_LINE       = 0   # no arrow / rejected as invalid
Y_TYPE_BOTTOM_TO_MID = 1   # tail BOT, head MID -- short line near robot (cross)
Y_TYPE_MID_TO_TOP    = 2   # tail MID, head TOP -- long line ahead (follow)
Y_TYPE_BOTTOM_TO_TOP = 3   # tail BOT, head TOP -- full-frame line


# ---------------------------------------------------------------------------
# LED feedback (onboard NeoPixel)
#
# Colour by visible state — operator can diagnose firmware health from across
# the room. Updated once per main-loop iteration so the LED never flickers.
# ---------------------------------------------------------------------------

LED_DISCONNECTED   = (255, 0,   0  )   # red    -- no HuskyLens / set_alg failed
LED_NO_LINE        = (255, 80,  0  )   # orange -- connected but no valid arrow
LED_CROSS          = (255, 255, 0  )   # yellow -- BOTTOM_TO_MID  (short, cross)
LED_FOLLOW         = (0,   255, 0  )   # green  -- MID_TO_TOP     (good follow)
LED_FULL_LINE      = (0,   180, 255)   # cyan   -- BOTTOM_TO_TOP  (full frame)

_current_led = (0, 0, 0)


def set_led(color):
    """Update the onboard NeoPixel; swallow errors so LED issues never abort the loop."""
    global _current_led
    if color == _current_led:
        return
    _current_led = color
    try:
        neopix.show(0, color)
    except Exception:
        pass


def led_for_state(connected, algorithm_set, y_type):
    """Pick the LED colour for the current sample."""
    if not connected or not algorithm_set:
        return LED_DISCONNECTED
    if y_type == Y_TYPE_BOTTOM_TO_MID:
        return LED_CROSS
    if y_type == Y_TYPE_MID_TO_TOP:
        return LED_FOLLOW
    if y_type == Y_TYPE_BOTTOM_TO_TOP:
        return LED_FULL_LINE
    return LED_NO_LINE


# ---------------------------------------------------------------------------
# Device protocol identity
# ---------------------------------------------------------------------------

DEV_ID  = "hrbot_huskylens"
FW_NAME = "huskylens"
FW_VER  = 1


# ---------------------------------------------------------------------------
# Pure helpers
# ---------------------------------------------------------------------------

def clamp(value, min_value, max_value):
    """Clamp `value` into the closed interval [min_value, max_value]."""
    if value < min_value:
        return min_value
    if value > max_value:
        return max_value
    return value


def no_line_tracking():
    """Neutral payload emitted when no valid arrow is available."""
    return {
        "valid":         0,
        "direction":     DEFAULT_DIRECTION,
        "tail_offset_x": 0,
        "y_type":        Y_TYPE_NO_LINE,
        "line_length_y": 0,
        "angle_deg":     0.0,
        "y_head":        0,
        "y_tail":        0,
    }


def classify_line_y(y_head, y_tail):
    """
    Map a (head, tail) pair to one of the Y_TYPE_* classes.

    Lines with both endpoints inside the MID band fold into MID_TO_TOP
    (line still points upward).

    Precondition: `y_head < y_tail` (enforced by the caller).
    """
    head_in_top = (int(y_head) < MID_Y_THRESHOLD)
    tail_in_bot = (int(y_tail) >= Y_BOTTOM_THRESHOLD)

    if head_in_top and tail_in_bot:
        return Y_TYPE_BOTTOM_TO_TOP
    if tail_in_bot:
        return Y_TYPE_BOTTOM_TO_MID
    return Y_TYPE_MID_TO_TOP


def calc_line_angle_from_tail_to_head(x_tail, y_tail, x_head, y_head):
    """
    Angle (degrees) of the tail->head vector relative to image-up.

    Negative = leaning left, positive = leaning right, zero = vertical.
    Image y grows downward, so the "up" component is `y_tail - y_head`.
    """
    dx    = int(x_head) - int(x_tail)
    dy_up = int(y_tail) - int(y_head)

    if dx == 0 and dy_up == 0:
        return 0.0

    return math.atan2(dx, dy_up) * 180.0 / math.pi


def arrow_to_line_data(arrow):
    """
    Convert a raw HuskyLens arrow into the firmware's line-tracking payload.

    Rejection rules (any failure returns `no_line_tracking()`):
      - arrow is None or missing any of x_tail/y_tail/x_head/y_head
      - head is not above tail in image coordinates
      - either endpoint sits fully outside the image bounds
      - tail deviates from center by more than MAX_ABS_TAIL_OFFSET_X

    Otherwise endpoints are clamped into the cropped X band before deriving
    the steering features.
    """
    if arrow is None:
        return no_line_tracking()

    x_tail    = getattr(arrow, "x_tail", None)
    y_tail    = getattr(arrow, "y_tail", None)
    x_head    = getattr(arrow, "x_head", None)
    y_head    = getattr(arrow, "y_head", None)
    direction = getattr(arrow, "direction", DEFAULT_DIRECTION)

    if x_tail is None or x_head is None or y_tail is None or y_head is None:
        return no_line_tracking()

    x_tail = int(x_tail)
    y_tail = int(y_tail)
    x_head = int(x_head)
    y_head = int(y_head)

    if y_head >= y_tail:
        return no_line_tracking()

    line_length_y = y_tail - y_head

    x_min_limit = X_CROP_LEFT
    x_max_limit = IMAGE_WIDTH - X_CROP_RIGHT
    if x_max_limit <= x_min_limit:
        return no_line_tracking()

    if x_tail < 0 or x_tail > IMAGE_WIDTH or x_head < 0 or x_head > IMAGE_WIDTH:
        return no_line_tracking()

    x_tail = clamp(x_tail, x_min_limit, x_max_limit)
    x_head = clamp(x_head, x_min_limit, x_max_limit)

    # Signed pixel deviation from the optical axis. Negative -> left, positive -> right.
    tail_offset_x = x_tail - IMAGE_CENTER_X
    if abs(tail_offset_x) > MAX_ABS_TAIL_OFFSET_X:
        return no_line_tracking()

    y_type    = classify_line_y(y_head, y_tail)
    angle_deg = calc_line_angle_from_tail_to_head(x_tail, y_tail, x_head, y_head)

    return {
        "valid":         1,
        "direction":     int(direction or DEFAULT_DIRECTION),
        "tail_offset_x": int(tail_offset_x),
        "y_type":        int(y_type),
        "line_length_y": int(line_length_y),
        "angle_deg":     float(angle_deg),
        "y_head":        int(y_head),
        "y_tail":        int(y_tail),
    }


# ---------------------------------------------------------------------------
# Device protocol I/O
# ---------------------------------------------------------------------------

def emit(event, payload):
    """Serialize one envelope per HospitalRobot Device Protocol v1."""
    print(json.dumps({
        "dev_id":  DEV_ID,
        "event":   event,
        "payload": payload,
    }))


def build_data_payload(line_tracking, connected, algorithm_set):
    """Assemble a `data` envelope payload from the latest line-tracking sample."""
    return {
        "connected":     1 if connected else 0,
        "algorithm_set": 1 if algorithm_set else 0,
        "valid":         int(line_tracking["valid"]),
        "tail_offset_x": int(line_tracking["tail_offset_x"]),
        "y_type":        int(line_tracking["y_type"]),
        "line_length_y": int(line_tracking["line_length_y"]),
        "direction":     int(line_tracking["direction"]),
        "angle_deg":     float(line_tracking["angle_deg"]),
        "y_head":        int(line_tracking["y_head"]),
        "y_tail":        int(line_tracking["y_tail"]),
    }


def make_uart_and_hl(prev_uart=None):
    """
    Allocate the UART peripheral and a HuskyLens client.

    Any prior UART handle is deinitialized first so re-allocating the same
    UART_ID on retry does not leak the driver instance.
    """
    if prev_uart is not None:
        try:
            prev_uart.deinit()
        except Exception:
            pass

    uart = UART(
        UART_ID,
        baudrate=UART_BAUDRATE,
        tx=UART_TX_PIN,
        rx=UART_RX_PIN,
        timeout=100,        # ms — _transport_read polls in chunks; cap each read so
                            # a missing HuskyLens cannot hang the loop > 1 frame.
        timeout_char=20,
    )
    time.sleep_ms(200)   # let the line settle before issuing the first request
    hl = HuskyLens(uart)
    try:
        hl.debug = False
    except Exception:
        pass
    return uart, hl


# ---------------------------------------------------------------------------
# Runtime state
# ---------------------------------------------------------------------------

uart                  = None
hl                    = None
connected             = False
algorithm_set         = False
last_retry_ms         = 0
get_arrows_takes_alg  = True   # probed lazily, then cached for the session


def read_arrows(client):
    """
    Call hl.get_arrows with or without the algorithm id depending on the
    library variant linked into the firmware. The signature is probed once
    and the result cached in `get_arrows_takes_alg`.

    Before each call we drain any leftover bytes from the UART RX buffer.
    The V1 _read_v1 protocol scans up to 50 * 150ms = 7.5s looking for the
    header sequence, which hangs the main loop if stray bytes from a partial
    or noisy previous response shift the scan out of sync.
    """
    global get_arrows_takes_alg
    try:
        u = getattr(client, "uart", None)
        if u is not None:
            pending = u.any() if hasattr(u, "any") else 0
            if pending:
                u.read(pending)
    except Exception:
        pass

    if get_arrows_takes_alg:
        try:
            return client.get_arrows(ALGORITHM_LINE_TRACKING)
        except TypeError:
            get_arrows_takes_alg = False
    return client.get_arrows()


# ---------------------------------------------------------------------------
# Boot banner
#
# Emitted ten times at 150 ms spacing (1.5 s total) so the upstream probe
# (which scans for device identity for ~3 s after power-on) is guaranteed
# to see at least one. The watchdog is fed during this phase so the boot
# delay does not trip it.
# ---------------------------------------------------------------------------

wdt = WDT(timeout=WDT_TIMEOUT_MS)

for _ in range(10):
    emit("boot", {"fw": FW_NAME, "ver": FW_VER})
    wdt.feed()
    time.sleep_ms(150)


# ---------------------------------------------------------------------------
# Main loop -- 10 Hz, deadline-driven so processing jitter does not slow the
# emitted sample rate.
# ---------------------------------------------------------------------------

while True:
    now           = time.ticks_ms()
    next_deadline = time.ticks_add(now, LOOP_DELAY_MS)
    wdt.feed()

    # Stage A: (re)open the UART link to the HuskyLens.
    if (not connected) and time.ticks_diff(now, last_retry_ms) >= RETRY_INTERVAL_MS:
        last_retry_ms = now
        try:
            uart, hl      = make_uart_and_hl(uart)
            connected     = bool(hl.knock())
            algorithm_set = False
        except Exception:
            connected     = False
            algorithm_set = False
            emit("error", {"code": "uart_init_failed"})
        wdt.feed()   # Stage A may take ~2 s on cold detect_version probe.

    # Stage B: ensure the device is in line-tracking mode.
    if connected and (not algorithm_set):
        try:
            connected     = bool(hl.set_alg(ALGORITHM_LINE_TRACKING))
            algorithm_set = connected
            if not algorithm_set:
                emit("error", {"code": "set_alg_failed"})
        except Exception:
            connected     = False
            algorithm_set = False
            emit("error", {"code": "set_alg_failed"})
        wdt.feed()   # set_alg + internal knock can take ~1 s.

    # Stage C: sample one arrow and reduce it to the line-tracking payload.
    # Bounded by a 500 ms ceiling so a hung _read_v1 header scan (worst case
    # ~7.5 s in huskylens.py V2.2.0 V1 path) cannot stall the 10 Hz loop.
    line_tracking = no_line_tracking()
    if connected and algorithm_set:
        stage_c_start = time.ticks_ms()
        try:
            arrows = read_arrows(hl)
            if time.ticks_diff(time.ticks_ms(), stage_c_start) > 500:
                # Protocol scanner is hunting a header that will never arrive
                # — drop the connection so Stage A retries on the next tick.
                raise RuntimeError("read_arrows_timeout")
            if arrows:
                # Prefer arrow whose tail is closest to the robot (largest y_tail).
                best = arrows[0]
                best_y = int(getattr(best, "y_tail", 0) or 0)
                for arrow in arrows[1:]:
                    y = int(getattr(arrow, "y_tail", 0) or 0)
                    if y > best_y:
                        best = arrow
                        best_y = y
                line_tracking = arrow_to_line_data(best)
        except Exception:
            connected     = False
            algorithm_set = False
            line_tracking = no_line_tracking()
            emit("error", {"code": "get_arrows_failed"})

    # Stage D: publish the sample, update LED, and sleep until the next deadline.
    emit("data", build_data_payload(line_tracking, connected, algorithm_set))
    set_led(led_for_state(connected, algorithm_set, int(line_tracking["y_type"])))

    remaining = time.ticks_diff(next_deadline, time.ticks_ms())
    if remaining > 0:
        time.sleep_ms(remaining)
