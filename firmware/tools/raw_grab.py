#!/usr/bin/env python3
"""Raw binary capture + frame-header / log parser for the SC850SL USB stream.

Unlike capture_serial.py (which forces UTF-8 stdout and mangles binary frame
bytes), this reads the port as raw bytes, so the frame magic + stats header and
the ASCII log lines both survive. Use it to confirm the camera is emitting valid
frames and to pull key boot-log lines out from under the binary frame flood.

    python raw_grab.py COM16 16      # capture COM16 for 16 s, parse
"""
import sys, time, re, struct
import serial

port = sys.argv[1] if len(sys.argv) > 1 else 'COM16'
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

# Open with retry: after a reset the USB-Serial-JTAG port re-enumerates and is
# briefly unavailable.
s = None
for _ in range(48):
    try:
        s = serial.Serial(port, 115200, timeout=0.5)
        break
    except Exception:
        time.sleep(0.25)
if s is None:
    print('could not open', port)
    sys.exit(1)

buf = bytearray()
end = time.time() + secs
while time.time() < end:
    buf += s.read(65536)
s.close()
b = bytes(buf)

MAGIC = b'\x00\x00\xff\xffFRM\x01'   # USB_STREAM_MAGIC (wire v1)
idx = [m.start() for m in re.finditer(re.escape(MAGIC), b)]
print('captured %d bytes ; %d frame header(s)' % (len(b), len(idx)))
for i in idx[:6]:
    if i + 24 <= len(b):
        w, h, mn, mx, me, bl, wr, wb = struct.unpack('<8H', b[i + 8:i + 24])
        print('  frame %dx%d  min=%d max=%d mean=%d bl=%d wbR=%.2f wbB=%.2f'
              % (w, h, mn, mx, me, bl, wr / 256.0, wb / 256.0))

# Pull a printable snippet around each interesting log marker.
for t in (b'emitting MIPI', b'got ip', b'Identified slave', b'Station mode',
          b'clear bus failed', b'reset hardware failed', b'no frame-done',
          b'stream-on failed'):
    hits = list(re.finditer(re.escape(t), b))
    if not hits:
        continue
    seg = b[hits[0].start():hits[0].start() + 60]
    line = re.sub(rb'[^ -~]', b' ', seg).decode().strip()
    print('  log[x%d] %s' % (len(hits), line))
