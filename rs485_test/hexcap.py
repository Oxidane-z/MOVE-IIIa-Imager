import serial, sys, time
port = sys.argv[1]
secs = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
s = serial.Serial(port, 115200, timeout=0.2)
end = time.time() + secs
buf = b''
while time.time() < end:
    buf += s.read(512)
s.close()
print("port", port, "got", len(buf), "bytes")
print("hex :", buf[:128].hex(' '))
print("repr:", repr(buf[:128]))
