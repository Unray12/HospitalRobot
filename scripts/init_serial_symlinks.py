#!/usr/bin/env python3
"""Probe nội dung mỗi /dev/ttyUSB*/ttyACM*, tạo symlink ổn định trong /dev/hospitalrobot/.

Cách hoạt động:
  - Với mỗi cổng tty, thử lần lượt các baudrate đã biết.
  - Đọc tối đa N bytes hoặc T giây.
  - Match với CONTENT_SIGNATURES; cổng đầu tiên khớp 1 role sẽ chiếm role đó.
  - Tạo symlink /dev/hospitalrobot/<role> -> /dev/ttyXXX.

Phải chạy TRƯỚC khi ROS launch (vì khi đang dùng port thì không probe được).
Cần sudo để ghi vào /dev/.

Usage:
  sudo python3 scripts/init_serial_symlinks.py
  sudo python3 scripts/init_serial_symlinks.py --dry-run
  sudo python3 scripts/init_serial_symlinks.py --required motor line camera
"""
from __future__ import annotations

import argparse
import glob
import os
import re
import sys
import time
from pathlib import Path

try:
    import serial
except ImportError:
    sys.stderr.write("Thiếu pyserial. Chạy: sudo apt install python3-serial\n")
    sys.exit(1)

LINK_DIR = Path("/dev/hospitalrobot")

# role -> (regex pattern, list baudrate cần thử, read_timeout_sec, max_bytes)
PROBES: dict[str, dict] = {
    "motor": {
        "pattern": re.compile(rb"<MOTOR|<CAN|MOTOR_STATUS"),
        "bauds":   [115200],
        "timeout": 1.5,
        "max":     2048,
    },
    "line": {
        "pattern": re.compile(rb'"LineSensor"'),
        "bauds":   [115200],
        "timeout": 2.0,
        "max":     4096,
    },
    "camera": {
        "pattern": re.compile(rb"<DEV\d"),
        "bauds":   [9600, 115200],
        "timeout": 2.5,
        "max":     4096,
    },
    "huskylens": {
        # HuskyLens dùng protocol nhị phân với preamble 0x55 0xAA 0x11
        "pattern": re.compile(rb"\x55\xAA\x11"),
        "bauds":   [9600, 115200],
        "timeout": 2.5,
        "max":     4096,
    },
}

# Heuristic dựa VID:PID để giảm số probe (không bắt buộc — vẫn fallback content probe).
VID_PID_HINT = {
    ("0403", "6001"): "motor",      # FTDI FT232R
    ("1a86", "7523"): "motor",      # CH340
    ("10c4", "ea60"): "motor",      # CP210x
    ("1a86", "55d3"): "huskylens",  # CH9102
}


def list_tty() -> list[str]:
    return sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*"))


def udev_prop(dev: str, key: str) -> str | None:
    import subprocess
    try:
        out = subprocess.check_output(["udevadm", "info", "-q", "property", "-n", dev],
                                      stderr=subprocess.DEVNULL, text=True)
    except Exception:
        return None
    for line in out.splitlines():
        if line.startswith(key + "="):
            return line.split("=", 1)[1]
    return None


def read_chunk(dev: str, baud: int, timeout: float, max_bytes: int) -> bytes:
    try:
        with serial.Serial(dev, baud, timeout=0.3) as ser:
            ser.reset_input_buffer()
            deadline = time.monotonic() + timeout
            buf = bytearray()
            while time.monotonic() < deadline and len(buf) < max_bytes:
                chunk = ser.read(512)
                if chunk:
                    buf.extend(chunk)
                else:
                    time.sleep(0.05)
            return bytes(buf)
    except (serial.SerialException, OSError) as e:
        print(f"  [warn] không mở được {dev} @ {baud}: {e}", file=sys.stderr)
        return b""


def classify(dev: str, hint_role: str | None) -> tuple[str | None, int | None]:
    """Trả về (role, baud) hoặc (None, None) nếu không nhận diện được."""
    role_order = list(PROBES)
    if hint_role and hint_role in role_order:
        role_order = [hint_role] + [r for r in role_order if r != hint_role]

    for role in role_order:
        cfg = PROBES[role]
        for baud in cfg["bauds"]:
            print(f"  probe {dev} as {role} @ {baud} ...", end="", flush=True)
            data = read_chunk(dev, baud, cfg["timeout"], cfg["max"])
            if data and cfg["pattern"].search(data):
                print(" MATCH")
                return role, baud
            print(f" no ({len(data)}B)")
    return None, None


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dry-run", action="store_true", help="không tạo symlink")
    ap.add_argument("--required", nargs="+", default=["motor", "line", "camera"],
                    help="role bắt buộc — exit non-zero nếu thiếu")
    ap.add_argument("--link-dir", type=Path, default=LINK_DIR)
    args = ap.parse_args()

    devices = list_tty()
    if not devices:
        print("Không tìm thấy /dev/ttyUSB*/ttyACM*", file=sys.stderr)
        return 1

    print(f"Found {len(devices)} tty device(s): {', '.join(devices)}")
    mapping: dict[str, tuple[str, int]] = {}

    for dev in devices:
        vid = (udev_prop(dev, "ID_VENDOR_ID") or "").lower()
        pid = (udev_prop(dev, "ID_MODEL_ID") or "").lower()
        hint = VID_PID_HINT.get((vid, pid))
        print(f"\n[{dev}] vid={vid or '-'} pid={pid or '-'} hint={hint or '-'}")
        if hint and hint in mapping:
            hint = None  # đã có rồi, để probe role khác

        role, baud = classify(dev, hint)
        if role is None:
            print(f"  -> không nhận diện được, bỏ qua")
            continue
        if role in mapping:
            print(f"  -> {role} đã được {mapping[role][0]} chiếm, bỏ qua")
            continue
        mapping[role] = (dev, baud)
        print(f"  => {role}: {dev} @ {baud}")

    print("\n=== Mapping ===")
    for role, (dev, baud) in mapping.items():
        print(f"  {role:<10} -> {dev}  (baud={baud})")

    missing = [r for r in args.required if r not in mapping]
    if missing:
        print(f"\n[ERROR] Thiếu role bắt buộc: {missing}", file=sys.stderr)
        return 2

    if args.dry_run:
        print("\n(dry-run) Không tạo symlink.")
        return 0

    args.link_dir.mkdir(parents=True, exist_ok=True)
    for role, (dev, _) in mapping.items():
        link = args.link_dir / role
        try:
            if link.is_symlink() or link.exists():
                link.unlink()
            link.symlink_to(dev)
            os.chmod(dev, 0o666)
            print(f"  symlink {link} -> {dev}")
        except PermissionError:
            print(f"  [ERROR] cần sudo để tạo {link}", file=sys.stderr)
            return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
