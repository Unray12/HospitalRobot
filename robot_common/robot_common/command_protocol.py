VALID_DIRECTIONS = (
    "Forward",
    "Backward",
    "Left",
    "Right",
    "RotateLeft",
    "RotateRight",
    "Stop",
)


def parse_command(text):
    raw = (text or "").strip()
    if not raw:
        return None, None

    direction = raw
    speed = 0
    for sep in (":", ",", " "):
        if sep in raw:
            parts = [p.strip() for p in raw.split(sep) if p.strip()]
            if parts:
                direction = parts[0]
            if len(parts) > 1:
                try:
                    speed = int(parts[1])
                except ValueError:
                    speed = 0
            break

    if direction not in VALID_DIRECTIONS:
        return None, None
    if direction == "Stop":
        return "Stop", 0
    if speed < 0:
        speed = 0
    return direction, int(speed)


def format_command(direction, speed=0):
    if direction not in VALID_DIRECTIONS:
        return None
    if direction == "Stop":
        return "Stop:0"
    try:
        speed = int(round(speed))
    except Exception:
        speed = 0
    if speed < 0:
        speed = 0
    return f"{direction}:{speed}"
