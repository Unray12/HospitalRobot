"""Pure-Python helpers for resolving inbound MQTT plan commands.

Extracted from ``MQTTBridgeROS.MQTTBridgeNode`` so the routing logic can be
unit-tested without rclpy/paho.
"""

import re

_ROOM_RE = re.compile(r"^(room|phong|plan)\s*[:/_-]?\s*([a-z0-9_-]+)$")
_FACE_RE = re.compile(r"FACE\s*,\s*([0-9,;|/ ]+)")


class PlanCommandResolver:
    """Translate free-form MQTT plan commands into canonical plan names.

    ``plan_key_map`` typically maps single-digit keys to plan names
    (``"1" -> "a19"``). ``room_plan_map`` maps lowercase room ids
    (``"a19" -> "a19"``, ``"1" -> "a19"``) and is used by the ``room:X``
    fallback. ``known_plans`` is the union used for the pass-through branch.

    ``resolve(payload)`` returns:
      - ``"clear"`` for clear/zero variants
      - the canonical plan name when the input matches a known mapping
      - the raw token from ``plan:X`` / ``plan/X`` pass-through
      - ``None`` when the payload is empty or a ``room:X`` lookup misses
      - the original ``payload`` otherwise (caller decides whether to ship it)
    """

    def __init__(self, plan_key_map, room_plan_map, known_plans=None):
        self.plan_key_map = dict(plan_key_map or {})
        self.room_plan_map = {
            str(k).strip().lower(): str(v).strip()
            for k, v in (room_plan_map or {}).items()
        }
        if not self.room_plan_map:
            self.room_plan_map = {
                str(k).strip().lower(): str(v).strip()
                for k, v in self.plan_key_map.items()
                if str(k).strip().isdigit()
            }
        if known_plans is None:
            known_plans = set(self.room_plan_map.values()) | set(self.plan_key_map.values())
        self.known_plans = set(known_plans)

    def resolve(self, payload):
        text = (payload or "").strip()
        if not text:
            return None

        lower = text.lower()
        if lower in {"0", "clear", "room:0", "room/0"}:
            return "clear"

        if text in self.plan_key_map:
            return self.plan_key_map[text]
        if lower in self.room_plan_map:
            return self.room_plan_map[lower]

        if lower.startswith("plan:") or lower.startswith("plan/"):
            plan_name = text.split(":", 1)[1].strip() if ":" in text else text.split("/", 1)[1].strip()
            return plan_name or None

        m = _ROOM_RE.match(lower)
        if m:
            prefix = m.group(1).strip()
            room_id = m.group(2).strip()
            if room_id in {"0", "clear"}:
                return "clear"
            if room_id in self.room_plan_map:
                return self.room_plan_map[room_id]
            return None

        if text in self.known_plans:
            return text

        return text


def extract_face_ids(payload):
    """Pull face ids out of a ``<DEV1,FACE,1,2>`` payload string."""
    text = (payload or "").strip().upper()
    if "FACE" not in text:
        return []
    m = _FACE_RE.search(text)
    if not m:
        return []
    ids = []
    for token in re.findall(r"\d+", m.group(1)):
        try:
            ids.append(int(token))
        except ValueError:
            continue
    return ids


__all__ = ["PlanCommandResolver", "extract_face_ids"]
