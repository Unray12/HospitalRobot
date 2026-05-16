"""Shared envelope parser cho HospitalRobot Device Serial Protocol v1.

Schema:
    {"dev_id":"hrbot_<role>", "event":"<type>", "payload":{...}, "ts"?:int}

Xem `docs/DEVICE_PROTOCOL.md` cho spec đầy đủ. Mọi ROS-side reader nên dùng
`parse_envelope()` để decode + validate, không tự parse JSON.
"""

from __future__ import annotations

import json
from typing import Any, Optional, Tuple

DEV_ID_PREFIX = "hrbot_"

EVENT_BOOT = "boot"
EVENT_DATA = "data"
EVENT_INFO = "info"
EVENT_ERROR = "error"
EVENT_ACK = "ack"

EVENT_TYPES = frozenset({EVENT_BOOT, EVENT_DATA, EVENT_INFO, EVENT_ERROR, EVENT_ACK})
# Events mà ROS reader nên bỏ qua thầm lặng (không log warning).
NON_DATA_EVENTS = frozenset({EVENT_BOOT, EVENT_INFO, EVENT_ERROR, EVENT_ACK})


class Envelope:
    """Envelope đã parse + validate. payload luôn là dict (có thể rỗng)."""
    __slots__ = ("dev_id", "event", "payload", "ts")

    def __init__(self, dev_id: str, event: str, payload: dict, ts: Optional[int] = None):
        self.dev_id = dev_id
        self.event = event
        self.payload = payload
        self.ts = ts

    @property
    def role(self) -> str:
        """`hrbot_line` -> `line`."""
        return self.dev_id[len(DEV_ID_PREFIX):] if self.dev_id.startswith(DEV_ID_PREFIX) else self.dev_id

    def is_data(self) -> bool:
        return self.event == EVENT_DATA

    def is_control(self) -> bool:
        return self.event in NON_DATA_EVENTS

    def __repr__(self) -> str:  # pragma: no cover
        return f"Envelope(dev_id={self.dev_id!r}, event={self.event!r}, payload={self.payload!r})"


def parse_envelope(
    raw: Any,
    expected_dev_id: Optional[str] = None,
) -> Tuple[Optional[Envelope], Optional[str]]:
    """Parse 1 dòng raw thành Envelope.

    Returns:
        (envelope, None)         nếu hợp lệ
        (None, "<error_code>")   nếu fail. Error codes:
            - "empty"               raw rỗng / None
            - "not_json"            không phải JSON object
            - "missing_dev_id"      thiếu field `dev_id`
            - "wrong_dev_id"        `dev_id` không khớp `expected_dev_id`
            - "missing_event"       thiếu field `event`
            - "unknown_event"       `event` không thuộc tập hợp lệ
            - "invalid_payload"     `payload` không phải object (cho phép vắng = {})

    Caller convention: error code `"empty"` / startswith `"wrong_"` thường có thể
    bỏ qua thầm lặng; các code khác nên log warning.
    """
    if raw is None:
        return None, "empty"
    text = raw.strip() if isinstance(raw, str) else None
    obj: Any
    if text is not None:
        if not text:
            return None, "empty"
        if not text.startswith("{"):
            return None, "not_json"
        try:
            obj = json.loads(text)
        except (ValueError, TypeError):
            return None, "not_json"
    else:
        obj = raw  # cho phép truyền dict trực tiếp (tiện cho test)

    if not isinstance(obj, dict):
        return None, "not_json"

    dev_id = obj.get("dev_id")
    if not isinstance(dev_id, str) or not dev_id:
        return None, "missing_dev_id"
    if expected_dev_id is not None and dev_id != expected_dev_id:
        return None, "wrong_dev_id"

    event = obj.get("event")
    if not isinstance(event, str) or not event:
        return None, "missing_event"
    if event not in EVENT_TYPES:
        return None, "unknown_event"

    payload = obj.get("payload", {})
    if not isinstance(payload, dict):
        return None, "invalid_payload"

    ts = obj.get("ts")
    if ts is not None and not isinstance(ts, (int, float)):
        ts = None

    return Envelope(dev_id=dev_id, event=event, payload=payload, ts=ts), None


def is_silent_error(error_code: Optional[str]) -> bool:
    """True nếu error code thuộc loại nên bỏ qua thầm lặng (không log warning).

    Dùng để reader filter: control frames của thiết bị khác lọt vào (vd boot banner)
    không phải lỗi từ phía mình.
    """
    if error_code is None:
        return False
    return error_code in {"empty", "wrong_dev_id"}
