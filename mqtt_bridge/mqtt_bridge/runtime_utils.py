import json
import re


def build_room_plan_state(plan_key_map, room_plans):
    room_plan_map = {str(k).strip().lower(): str(v).strip() for k, v in room_plans.items()}
    if not room_plan_map:
        room_plan_map = {
            str(k).strip().lower(): str(v).strip()
            for k, v in plan_key_map.items()
            if str(k).strip().isdigit()
        }
    known_plans = set(room_plan_map.values()) | set(plan_key_map.values())
    return room_plan_map, known_plans


def update_plan_runtime_state(payload, active_plan_name, plan_autoline_active):
    try:
        data = json.loads(payload)
    except Exception:
        return active_plan_name, plan_autoline_active
    if not isinstance(data, dict):
        return active_plan_name, plan_autoline_active

    event_name = str(data.get("event") or "").strip().lower()
    plan_name = data.get("plan")
    autoline = data.get("autoline")

    if plan_name:
        active_plan_name = str(plan_name).strip()

    if autoline is not None:
        plan_autoline_active = bool(autoline)

    if event_name in {"cleared", "completed_reset"}:
        active_plan_name = None
        plan_autoline_active = False

    return active_plan_name, plan_autoline_active


def extract_face_id(payload):
    text = (payload or "").strip().upper()
    if "FACE" not in text:
        return None
    match = re.search(r"FACE\s*,\s*(\d+)", text)
    if not match:
        return None
    try:
        return int(match.group(1))
    except ValueError:
        return None


def resolve_plan_command(payload, plan_key_map, room_plan_map, known_plans):
    text = (payload or "").strip()
    if not text:
        return None, None

    lower = text.lower()
    if lower in {"0", "clear", "room:0", "room/0"}:
        return "clear", None

    if text in plan_key_map:
        return plan_key_map[text], None
    if lower in room_plan_map:
        return room_plan_map[lower], None

    match = re.match(r"^(room|phong|plan)\s*[:/_-]?\s*([a-z0-9_-]+)$", lower)
    if match:
        prefix = match.group(1).strip()
        room_id = match.group(2).strip()
        if room_id in {"0", "clear"}:
            return "clear", None
        if room_id in room_plan_map:
            return room_plan_map[room_id], None
        if prefix == "plan" and room_id.startswith("plan_"):
            return room_id, None
        return None, room_id

    if lower.startswith("plan:") or lower.startswith("plan/"):
        plan_name = text.split(":", 1)[1].strip() if ":" in text else text.split("/", 1)[1].strip()
        return plan_name or None, None

    if text in known_plans:
        return text, None

    return text, None
