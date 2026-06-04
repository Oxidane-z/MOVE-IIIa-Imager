#!/usr/bin/env python3
"""Analyze the M5Stack llm-camera binary for SC850SL register tables."""
import sys, struct, re
from pathlib import Path

BIN = Path(__file__).parent / "extracted/opt/m5stack/bin/llm_camera-1.9"

def scan_strings(buf, min_len=6):
    """Yield (offset, text) for ASCII runs."""
    cur_start = None
    cur = []
    for i, b in enumerate(buf):
        if 0x20 <= b < 0x7f:
            if cur_start is None:
                cur_start = i
            cur.append(b)
        else:
            if cur_start is not None and len(cur) >= min_len:
                yield cur_start, bytes(cur).decode('ascii', 'replace')
            cur_start = None
            cur = []

def main():
    data = BIN.read_bytes()
    print(f"binary size: {len(data):,} bytes")

    # 1. Filter strings for SC850SL-related markers
    keywords = ['sc850sl', 'SC850SL', 'Sc850sl', 'sensor_', 'mipi_lane',
                'init_setting', 'init_table', '1920x1080', '3840x2160',
                'binning', 'BINNING', '_8M', '_4M', '_2M', '8MP', 'imgsz',
                'axera_single', 'lane_num']
    print("\n=== Interesting strings ===")
    found = []
    for off, s in scan_strings(data, min_len=4):
        if any(k in s for k in keywords):
            found.append((off, s))
    for off, s in found[:200]:
        print(f"  0x{off:08x}  {s}")
    print(f"  ... {len(found)} total")

    # 2. Find the PLL preamble pattern: register 0x36e9 with value 0x80
    # Typical encodings:
    #   struct { u16 reg; u8 val; }       -> e9 36 80
    #   struct { u16 reg; u8 val; u8 pad } -> e9 36 80 00
    #   struct { u16 reg; u16 val; }      -> e9 36 80 00
    print("\n=== Searching for SC850SL PLL preamble (reg 0x36e9, val 0x80) ===")
    patterns = {
        "u16+u8 (3-byte)":       b"\xe9\x36\x80",
        "u16+u8 (4-byte align)": b"\xe9\x36\x80\x00",
        "u16+u16":               b"\xe9\x36\x80\x00",
    }
    seen_offsets = set()
    for label, pat in patterns.items():
        for m in re.finditer(re.escape(pat), data):
            o = m.start()
            if o in seen_offsets: continue
            seen_offsets.add(o)
            print(f"  {label:24s} @ 0x{o:08x}")

    # 3. Try to locate full init table by scanning for the unique sequence:
    #    36e9 80 ... 36f9 80 ... 36ea 08 ...
    # We expect them within ~32 bytes of each other if 3-byte records.
    print("\n=== Adjacent PLL preamble + commit sequence ===")
    # Find runs where these three appear close together:
    e9_80 = [m.start() for m in re.finditer(b"\xe9\x36\x80", data)]
    f9_80 = [m.start() for m in re.finditer(b"\xf9\x36\x80", data)]
    for off in e9_80:
        # Look for f9 36 80 within next 128 bytes
        for off2 in f9_80:
            if off < off2 < off + 128:
                # ea 08 should appear shortly after
                window = data[off:off+200]
                if b"\xea\x36\x08" in window or b"\xea\x36\x08\x00" in window:
                    print(f"  table candidate @ 0x{off:08x}, length ~{off2-off+8} bytes header")
                    print(f"    bytes: {window[:48].hex(' ')}")
                    break

if __name__ == "__main__":
    main()
