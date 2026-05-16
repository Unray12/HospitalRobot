#!/usr/bin/env python3
"""Probe nội dung mỗi /dev/ttyUSB*/ttyACM*, tạo symlink /dev/hospitalrobot/<role>.

Quy tắc nhận diện (theo thứ tự ưu tiên):
  1. ttyUSB* duy nhất                 -> motor (USB-Serial adapter)
  2. Content signature khớp regex     -> role tương ứng
  3. Không khớp                       -> bỏ qua

Cần sudo để ghi /dev/.
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

# HospitalRobot Device Protocol v1: firmware emit envelope JSON với field "dev_id".
# Probe chỉ cần match chữ ký dev_id — nhất quán, dễ thêm role mới (xem
# docs/DEVICE_PROTOCOL.md).
# Fallback regex cũ giữ lại để tương thích firmware chưa cập nhật.
PROBES: dict[str, dict] = {
    "line": {
        "pattern": re.compile(rb'"dev_id"\s*:\s*"hrbot_line"|"LineSensor"'),
        "bauds":   [115200],
        "timeout": 3.0,
    },
    "camera": {
        "pattern": re.compile(rb'"dev_id"\s*:\s*"hrbot_camera"|<DEV\d'),
        "bauds":   [115200],
        "timeout": 3.0,
    },
    "huskylens": {
        "pattern": re.compile(rb'"dev_id"\s*:\s*"hrbot_huskylens"|"HuskylenSensor"'),
        "bauds":   [115200],
        "timeout": 3.0,
    },
    "motor": {
        "pattern": re.compile(rb'"dev_id"\s*:\s*"hrbot_motor"|<MOTOR|<CAN|MOTOR_STATUS'),
        "bauds":   [115200],
        "timeout": 2.0,
    },
}


def list_tty() -> tuple[list[str], list[str]]:
    usb = sorted(glob.glob("/dev/ttyUSB*"))
    acm = sorted(glob.glob("/dev/ttyACM*"))
    return usb, acm


def read_chunk(dev: str, baud: int, timeout: float) -> bytes:
    try:
        with serial.Serial(dev, baud, timeout=0.3) as ser:
            ser.reset_input_buffer()
            deadline = time.monotonic() + timeout
            buf = bytearray()
            while time.monotonic() < deadline and len(buf) < 4096:
                chunk = ser.read(512)
                if chunk:
                    buf.extend(chunk)
                else:
                    time.sleep(0.05)
            return bytes(buf)
    except (serial.SerialException, OSError) as e:
        print(f"  [warn] mở {dev} @ {baud}: {e}", file=sys.stderr)
        return b""


def classify(dev: str, candidate_roles: list[str]) -> tuple[str | None, int | None, bytes]:
    last_data = b""
    for role in candidate_roles:
        cfg = PROBES[role]
        for baud in cfg["bauds"]:
            print(f"  probe {dev} -> {role} @ {baud}...", end="", flush=True)
            data = read_chunk(dev, baud, cfg["timeout"])
            last_data = data or last_data
            if data and cfg["pattern"].search(data):
                print(f" MATCH ({len(data)}B)")
                return role, baud, data
            print(f" no ({len(data)}B)")
    return None, None, last_data


def safe_symlink(link: Path, target: str) -> None:
    if link.is_symlink() or link.exists():
        link.unlink()
    link.symlink_to(target)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--link-dir", type=Path, default=LINK_DIR)
    ap.add_argument("--require", nargs="*", default=[],
                    help="role bắt buộc, exit non-zero nếu thiếu")
    args = ap.parse_args()

    usb, acm = list_tty()
    devices = usb + acm
    if not devices:
        print("Không tìm thấy /dev/ttyUSB*/ttyACM*", file=sys.stderr)
        return 1

    print(f"USB:  {usb or '(none)'}")
    print(f"ACM:  {acm or '(none)'}")

    mapping: dict[str, tuple[str, int | None]] = {}

    # 1. ttyUSB duy nhất -> motor.
    if len(usb) == 1 and "motor" not in mapping:
        mapping["motor"] = (usb[0], None)
        print(f"\n[auto] {usb[0]} là ttyUSB duy nhất -> motor")

    # 2. Content probe các port còn lại.
    remaining = [d for d in devices if d not in {v[0] for v in mapping.values()}]
    for dev in remaining:
        print(f"\n[{dev}]")
        # role ưu tiên: line/camera/huskylens (đa số ACM), rồi motor.
        cand = [r for r in ["line", "camera", "huskylens", "motor"] if r not in mapping]
        if not cand:
            break
        role, baud, sample = classify(dev, cand)
        if role:
            mapping[role] = (dev, baud)
            print(f"  => {role}")
        else:
            preview = sample[:120].decode("ascii", errors="replace")
            print(f"  => không nhận diện được. Preview: {preview!r}")

    print("\n=== Mapping cuối ===")
    if not mapping:
        print("  (rỗng)")
    for role, (dev, baud) in mapping.items():
        print(f"  {role:<10} -> {dev}" + (f"  (baud={baud})" if baud else ""))

    missing = [r for r in args.require if r not in mapping]
    if missing:
        print(f"\n[ERROR] Thiếu role bắt buộc: {missing}", file=sys.stderr)
        return 2

    if args.dry_run:
        print("\n(dry-run) Không tạo symlink.")
        return 0

    try:
        args.link_dir.mkdir(parents=True, exist_ok=True)
    except PermissionError:
        print(f"[ERROR] Cần sudo để tạo {args.link_dir}", file=sys.stderr)
        return 3

    # Xoá symlink cũ không còn dùng.
    if args.link_dir.exists():
        for p in args.link_dir.iterdir():
            if p.is_symlink() and p.name not in mapping:
                p.unlink()

    for role, (dev, _) in mapping.items():
        link = args.link_dir / role
        try:
            safe_symlink(link, dev)
            try:
                os.chmod(dev, 0o666)
            except PermissionError:
                pass
            print(f"  symlink {link} -> {dev}")
        except PermissionError:
            print(f"[ERROR] Cần sudo để tạo {link}", file=sys.stderr)
            return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
