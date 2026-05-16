#!/usr/bin/env python3
"""Auto-detect serial devices và map sang role (motor / line / huskylens / camera).

Hai cách dùng:
  python3 scripts/detect_serial_devices.py            # in bảng
  python3 scripts/detect_serial_devices.py --json     # in JSON
  python3 scripts/detect_serial_devices.py --apply    # ghi đè robot_common/.../config.json

Hoạt động chéo nền tảng (Linux + Windows COM) nhờ pyserial.
Yêu cầu: pip install pyserial
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

try:
    from serial.tools import list_ports
except ImportError:
    sys.stderr.write("pyserial chưa cài. Chạy: pip install pyserial\n")
    sys.exit(1)

# (role, vid, pid) — sửa cho khớp phần cứng thực tế (xem `lsusb` hoặc `--json`).
KNOWN: list[tuple[str, int, int]] = [
    ("motor",     0x1A86, 0x7523),  # CH340
    ("motor",     0x10C4, 0xEA60),  # CP210x
    ("motor",     0x0403, 0x6001),  # FTDI
    ("line",      0x2341, 0x0043),  # Arduino Uno
    ("line",      0x2341, 0x0001),
    ("huskylens", 0x1A86, 0x55D3),  # CH9102 (HuskyLens hay gặp)
    ("huskylens", 0x303A, 0x1001),  # ESP32-S3
    ("camera",    0x2341, 0x8036),  # Arduino Leonardo
    ("camera",    0x239A, 0x800B),  # Adafruit
]

# Ưu tiên khi nhiều thiết bị cùng role: device có serial number nhỏ nhất (đầu cắm cố định).
ROLE_TO_CONFIG_KEY = {
    "motor":     ("motor_driver", "serial", "port"),
    "line":      ("line_sensors", "serial", "port"),
    "huskylens": ("huskylens_sensor", "serial", "port"),
    "camera":    ("camera_sensor", "serial", "port"),
}


def classify(vid: int | None, pid: int | None) -> str:
    if vid is None or pid is None:
        return "unknown"
    for role, v, p in KNOWN:
        if v == vid and p == pid:
            return role
    return "unknown"


def scan() -> list[dict]:
    out = []
    for p in list_ports.comports():
        role = classify(p.vid, p.pid)
        out.append({
            "device":  p.device,
            "vid":     f"{p.vid:04x}" if p.vid is not None else None,
            "pid":     f"{p.pid:04x}" if p.pid is not None else None,
            "serial":  p.serial_number,
            "product": p.product,
            "manufacturer": p.manufacturer,
            "role":    role,
        })
    return out


def pick_by_role(entries: list[dict]) -> dict[str, str]:
    """Mỗi role chọn 1 device. Nếu trùng role thì sắp theo serial rồi device name."""
    by_role: dict[str, list[dict]] = {}
    for e in entries:
        if e["role"] != "unknown":
            by_role.setdefault(e["role"], []).append(e)
    chosen: dict[str, str] = {}
    for role, lst in by_role.items():
        lst.sort(key=lambda x: (x["serial"] or "", x["device"]))
        chosen[role] = lst[0]["device"]
    return chosen


def print_table(entries: list[dict]) -> None:
    fmt = "{:<18} {:<6} {:<6} {:<22} {:<10} {}"
    print(fmt.format("DEVICE", "VID", "PID", "SERIAL", "ROLE", "PRODUCT"))
    print("-" * 90)
    for e in entries:
        print(fmt.format(
            e["device"] or "-",
            e["vid"] or "-",
            e["pid"] or "-",
            (e["serial"] or "-")[:22],
            e["role"],
            e["product"] or "-",
        ))


def apply_to_config(chosen: dict[str, str], config_path: Path) -> None:
    cfg = json.loads(config_path.read_text(encoding="utf-8"))
    changes = []
    for role, device in chosen.items():
        section, group, key = ROLE_TO_CONFIG_KEY[role]
        old = cfg.get(section, {}).get(group, {}).get(key)
        if old != device:
            cfg.setdefault(section, {}).setdefault(group, {})[key] = device
            changes.append((role, old, device))
    if not changes:
        print("config.json đã đúng — không thay đổi.")
        return
    config_path.write_text(json.dumps(cfg, indent=4, ensure_ascii=False) + "\n",
                           encoding="utf-8")
    for role, old, new in changes:
        print(f"  {role:<10} {old} -> {new}")


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--json", action="store_true", help="xuất JSON")
    ap.add_argument("--apply", action="store_true",
                    help="ghi cổng đã detect vào robot_common/robot_common/config.json")
    ap.add_argument("--config", type=Path,
                    default=Path(__file__).resolve().parent.parent
                    / "robot_common" / "robot_common" / "config.json")
    ap.add_argument("--role", choices=list(ROLE_TO_CONFIG_KEY),
                    help="chỉ in device path cho 1 role (dùng trong shell scripts)")
    args = ap.parse_args()

    entries = scan()
    if not entries:
        print("Không tìm thấy serial port.", file=sys.stderr)
        return 1

    if args.role:
        chosen = pick_by_role(entries)
        if args.role not in chosen:
            print(f"Không tìm thấy device cho role '{args.role}'.", file=sys.stderr)
            return 1
        print(chosen[args.role])
        return 0

    if args.json:
        print(json.dumps(entries, indent=2, ensure_ascii=False))
    else:
        print_table(entries)
        chosen = pick_by_role(entries)
        if chosen:
            print("\nMapping được chọn:")
            for role, dev in chosen.items():
                print(f"  {role:<10} -> {dev}")

    if args.apply:
        chosen = pick_by_role(entries)
        if not chosen:
            print("Không có thiết bị nào được nhận dạng — bỏ qua --apply.",
                  file=sys.stderr)
            return 1
        apply_to_config(chosen, args.config)
    return 0


if __name__ == "__main__":
    sys.exit(main())
