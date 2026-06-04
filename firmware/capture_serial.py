#!/usr/bin/env python3
"""One-shot serial capture: reset COM13, read for N seconds, print, exit."""
import serial, time, sys

# Force UTF-8 stdout — firmware logs include ✓ etc.
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass

port = sys.argv[1] if len(sys.argv) > 1 else 'COM13'
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 6.0

s = serial.Serial(port, 115200, timeout=0.1)
# USB-Serial-JTAG: try classic UART reset toggle just in case it's mapped
s.setRTS(True);  time.sleep(0.05)
s.setRTS(False); time.sleep(0.1)
s.setRTS(True)
try: s.send_break(duration=0.1)
except Exception: pass

end = time.time() + secs
buf = bytearray()
while time.time() < end:
    chunk = s.read(2048)
    if chunk:
        buf.extend(chunk)
s.close()

text = buf.decode('utf-8', 'replace')
print(f"--- {len(buf)} bytes received ---")
print(text)
