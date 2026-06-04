#!/usr/bin/env python3
"""
Live host-side viewer for the Stamp-P4 USB camera stream.

The firmware (app_sc850sl.c -> usb_stream_task) pushes one 640x240 RGB565
composite frame (RGB visible | thermal) out the USB-Serial-JTAG port. Each
frame is a SINGLE atomic write so ESP_LOGx text can't splice into it; any
residual misalignment (a dropped byte, or a log that landed in the gap
between frames) is caught by a fixed trailer sentinel and the frame is
dropped + re-synced rather than displayed as a smear.

Wire format (v1, little-endian):
    magic[8]   = 00 00 ff ff 'F' 'R' 'M' 01
    u16 width,  u16 height
    u16 raw_min, u16 raw_max, u16 raw_mean      (exposure stats)
    u16 black_level, u16 wb_r*256, u16 wb_b*256
        ^-- 24-byte header
    pixels[width*height*2]                       (RGB565 LE)
    trailer[4] = DE AD BE EF

The exposure stats are overlaid on the image so you can watch `max` (raw
brightness ceiling) while adjusting lighting/exposure.

Usage:  python tools/usb_preview.py [PORT]   (default COM16)
Press Esc or close the window to exit.
"""
import sys
import struct
import threading
import time
import tkinter as tk

import serial
import numpy as np
from PIL import Image, ImageTk, ImageDraw

MAGIC              = bytes([0x00, 0x00, 0xff, 0xff]) + b"FRM" + bytes([0x01])
TRAILER            = bytes([0xDE, 0xAD, 0xBE, 0xEF])
HEADER_AFTER_MAGIC = 16          # 8 x u16 following the magic
DEFAULT_PORT       = "COM16"
DISPLAY_SCALE      = 1           # 640x240 composite shown 1:1


def find_magic(ser: serial.Serial) -> None:
    """Read bytes until the magic sequence is matched (re-sync point)."""
    idx = 0
    while idx < len(MAGIC):
        b = ser.read(1)
        if not b:
            continue
        if b[0] == MAGIC[idx]:
            idx += 1
        else:
            # Mismatch — restart. If this byte is MAGIC[0], the partial
            # prefix is length 1, otherwise 0.
            idx = 1 if b[0] == MAGIC[0] else 0


def read_exact(ser: serial.Serial, n: int) -> bytes:
    """Read exactly n bytes, blocking until they arrive."""
    out = bytearray()
    while len(out) < n:
        chunk = ser.read(n - len(out))
        if chunk:
            out.extend(chunk)
    return bytes(out)


def annotate(img: Image.Image, st: dict) -> None:
    """Overlay the exposure/WB stats on the top-left of the frame."""
    d = ImageDraw.Draw(img)
    line1 = f"max {st['max']:3d}  mean {st['mean']:3d}  min {st['min']:3d}"
    line2 = f"bl {st['bl']:3d}  wbR {st['wbr']:.2f}  wbB {st['wbb']:.2f}"
    d.rectangle([0, 0, 158, 21], fill=(0, 0, 0))
    d.text((2, 0),  line1, fill=(255, 255, 0))
    d.text((2, 10), line2, fill=(0, 255, 255))


# ----------------------------------------------------------------------
# Tkinter display thread
# ----------------------------------------------------------------------

class Viewer:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("Stamp-P4 SC850SL — USB live preview")
        self.label = tk.Label(self.root, bg="#101820")
        self.label.pack()
        self.root.bind("<Escape>", lambda e: self.root.destroy())
        self._tk_img = None
        self.next_image = None
        self.lock = threading.Lock()
        self.frames_seen = 0
        self.drops = 0
        self.last_status = time.time()
        self.root.after(50, self._poll)

    def update(self, image):
        with self.lock:
            self.next_image = image
            self.frames_seen += 1

    def set_drops(self, n):
        with self.lock:
            self.drops = n

    def _poll(self):
        with self.lock:
            img = self.next_image
            self.next_image = None
            seen = self.frames_seen
            drops = self.drops
        if img is not None:
            disp = img.resize((img.width * DISPLAY_SCALE,
                               img.height * DISPLAY_SCALE),
                              Image.NEAREST)
            self._tk_img = ImageTk.PhotoImage(disp)
            self.label.config(image=self._tk_img)
        now = time.time()
        if now - self.last_status > 1.0:
            print(f"[viewer] frames={seen} drops={drops}", flush=True)
            self.last_status = now
        self.root.after(50, self._poll)

    def mainloop(self):
        self.root.mainloop()


# ----------------------------------------------------------------------
# Serial reader thread
# ----------------------------------------------------------------------

def reader_loop(port: str, viewer: "Viewer"):
    drops = 0
    while True:
        try:
            ser = serial.Serial(port, 115200, timeout=1)
        except Exception as e:
            print(f"[reader] open {port}: {e} — retrying", flush=True)
            time.sleep(1)
            continue
        print(f"[reader] streaming from {port}", flush=True)
        while True:
            try:
                find_magic(ser)
                hdr = read_exact(ser, HEADER_AFTER_MAGIC)
                (w, h, raw_min, raw_max, raw_mean,
                 bl, wbr_q8, wbb_q8) = struct.unpack("<8H", hdr)

                if not (8 <= w <= 4096 and 8 <= h <= 4096):
                    drops += 1
                    viewer.set_drops(drops)
                    continue
                payload_len = w * h * 2
                data = read_exact(ser, payload_len)
                trailer = read_exact(ser, len(TRAILER))
                if trailer != TRAILER:
                    # Misaligned (dropped byte, or a log landed in this frame's
                    # slot). Skip it; find_magic re-syncs on the next frame.
                    drops += 1
                    viewer.set_drops(drops)
                    if drops <= 5 or drops % 25 == 0:
                        print(f"[reader] bad trailer {trailer.hex()} — dropped "
                              f"(total {drops})", flush=True)
                    continue

                # RGB565 LE -> RGB888, vectorized with numpy.
                v = np.frombuffer(data, dtype="<u2")
                r = (v >> 11) & 0x1f
                g = (v >> 5) & 0x3f
                b = v & 0x1f
                r8 = ((r << 3) | (r >> 2)).astype(np.uint8)
                g8 = ((g << 2) | (g >> 4)).astype(np.uint8)
                b8 = ((b << 3) | (b >> 2)).astype(np.uint8)
                rgb = np.dstack((r8, g8, b8)).reshape((h, w, 3))
                img = Image.fromarray(rgb, "RGB")
                annotate(img, {"min": raw_min, "max": raw_max, "mean": raw_mean,
                               "bl": bl, "wbr": wbr_q8 / 256.0,
                               "wbb": wbb_q8 / 256.0})
                viewer.update(img)
            except serial.SerialException as e:
                print(f"[reader] serial: {e}", flush=True)
                break
        try:
            ser.close()
        except Exception:
            pass


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
    viewer = Viewer()
    t = threading.Thread(target=reader_loop, args=(port, viewer), daemon=True)
    t.start()
    viewer.mainloop()


if __name__ == "__main__":
    main()
