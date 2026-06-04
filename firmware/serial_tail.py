#!/usr/bin/env python3
"""
Continuous serial tail. One line per stdout event; ANSI color codes
stripped for cleaner display. Does NOT reset the device — so the
streaming starts whatever-the-device-is-doing right now.

Usage:  python serial_tail.py [PORT]   (default PORT = COM16)
"""
import re
import sys
import time

import serial

try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace", line_buffering=True)
except Exception:
    pass

port = sys.argv[1] if len(sys.argv) > 1 else "COM16"
ANSI = re.compile(r"\x1b\[[0-9;]*m")

while True:
    try:
        s = serial.Serial(port, 115200, timeout=0.5)
    except serial.SerialException as e:
        # Device may be re-enumerating after a flash / reset; back off and retry.
        print(f"[tail] open {port}: {e}; retrying in 1s", flush=True)
        time.sleep(1)
        continue

    buf = bytearray()
    while True:
        try:
            chunk = s.read(4096)
        except serial.SerialException as e:
            print(f"[tail] read: {e}; reopening", flush=True)
            break
        if not chunk:
            continue
        buf.extend(chunk)
        while b"\n" in buf:
            line, _, rest = buf.partition(b"\n")
            buf = rest
            text = line.decode("utf-8", "replace").rstrip("\r")
            print(ANSI.sub("", text), flush=True)

    try:
        s.close()
    except Exception:
        pass
