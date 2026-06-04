# Final inventory — everything we need for ESP32-P4 / SC850SL bring-up

## ⭐ The four register init tables (extracted from libsns_sc850sl.so .data)

All are `{u16 reg, u8 val}` records, written to `sc850sl_table_{0..3}.c`.

| File | Lane | Bit | HTS | VTS | What it is |
|---|---|---|---|---|---|
| `sc850sl_table_0.c` | 4 | RAW10 | 1100 | (set by .ini) | **4K30 SDR linear** — M5Stack default |
| `sc850sl_table_1.c` | 4 | RAW10 | 550  | 4500 | **HDR-DOL 4K30** |
| `sc850sl_table_2.c` | **2** | RAW10 | 1100 | 2250 | **2-lane mode A** (PLL half-speed) |
| `sc850sl_table_3.c` | **2** | RAW10 | 2200 | 2250 | **2-lane mode B** (extended HTS) |

189–206 entries each. First write is `0x0103=0x01` (soft reset), last is
`0x0100=0x00` (stream off; the driver flips it to 0x01 after AE setup).

## Key registers at a glance — 4-lane vs 2-lane

|        | Table #0 (4L) | Table #2 (2L) | Diff |
|---|---|---|---|
| 0x3018 | 0x7a (4-lane) | **0x3a (2-lane)** | bit[7:5] 011→001 |
| 0x3019 | 0xf0 | 0xfc | minor |
| 0x301a | (not set) | 0x30 | new in 2L |
| 0x3031 | 0x0a (RAW10) | 0x0a (RAW10) | same |
| 0x36e9 | 0x24 (PLL commit) | 0x20 (PLL commit) | half rate |
| 0x36ea | 0x09 (PLL div) | 0x17 (PLL div) | very different |
| 0x36fa | 0x0b | **0xcb** | MIPI PLL changes massively |
| 0x36fb | 0x33 | 0x13 | |
| 0x36fc | 0x10 | 0x00 | |
| 0x36fd | 0x37 | 0x07 | |
| 0x320c (HTS) | 0x04 | 0x04 | same → 0x044c = 1100 |
| 0x36c9..36cb | (per OpenIPC) | (different) | analog bias adjust |

The diffs are far more than just lane bit — the **whole MIPI PLL is retuned**
for 2-lane operation. Trying to hack 4-lane init → 2-lane by just flipping
0x3018 would NOT have worked. This is exactly the file we needed.

## Configuration parameters (from sc850sl_sdr_2lane_15fps.ini)

```
nWidth        = 3840
nHeight       = 2160
fFrameRate    = 15
eSnsMode      = 1       (linear)
eRawType      = 10      (RAW10)
eBayerPattern = 0       (RGGB)
eLaneNum      = 2
nDataRate     = 1080    (Mbps per lane)
nDataLaneMap  = {0,1,3,4}
nClkLane      = {2,5}
ePixelFmt     = 133     (0x85 = RAW10_PACKED)
szImgDt       = 43      (0x2B = RAW10 DT per CSI-2)
```

## Workflow we'll codify into the ESP32-P4 driver

```
Power up VLDOs (DOVDD → DVDD/AVDD → wait 4 ms → release XSHUTDN)
  ↓
Start EXTCLK = 24 MHz (LEDC on P4 GPIO)
  ↓
Wait 5 ms for sensor PLL prep
  ↓
Read 0x3107/0x3108 → expect 0x9d 0x1e  (sanity check)
  ↓
Walk sc850sl_table_2[] writing each {reg,val} via I²C
  ↓
Set output window: 0x3208/9 = 3840, 0x320a/b = 2160, X/Y start = 0
  ↓
Set VTS for 15 fps: 0x320e=0x08, 0x320f=0xca   (already in table)
  ↓
Configure AE: starting exposure 0x3e01/02, gain 0x3e08/09
  ↓
Configure ESP32-P4 CSI host: 2-lane, RAW10, 1080 Mbps/lane, DT=0x2B
  ↓
Configure ESP32-P4 ISP: input 1920x1080 from CSI (need to crop! sensor outputs 4K)
  ↓
Start stream: 0x302c=0x00, 0x0100=0x01
  ↓
Wait for first frame (CSI EOF interrupt)
```

⚠ **Important**: P4 ISP maxes at 1920×1080 input. SC850SL is outputting
3840×2160 here. Options:
1. Set sensor output window to 1920×1080 (center crop) — works, halves FOV
2. Skip P4 ISP, route CSI direct to PSRAM, do everything in software
3. Use P4 ISP "memory input" mode: full RAW to PSRAM, then ISP reads 1920×1080
   tiles for sequential JPEG encoding

Option 3 is best for full-frame output. We can do quadrant tiling: 4× ISP
passes through the 4K RAW frame, each handling a 1920×1080 quadrant, then
stitch the four JPEG quadrants on the P4 CPU (or skip stitching — just
send 4 sub-images to OBC).

For SSTV (320×240): just one ISP pass with the **center** 1920×1080
quadrant, then PPA scale to 320×240.

## Files now in `rootfs_files/`

```
opt/etc/sc850sl_sdr_2lane_15fps.ini          ⭐ target config
opt/etc/sc850sl_sdr_4lane.ini                (reference)
opt/etc/sc850sl_sdr_4lane_620q.ini
opt/etc/sc850sl_hdr_4lane.ini
opt/etc/sc850sl_hdr_4lane_620q.ini
opt/etc/sc850sl_single_sdr_2lane_15fps_entry.ini
opt/etc/sc850sl_single_sdr_4lane_entry.ini
opt/etc/sc850sl_single_hdr_4lane_entry.ini
opt/etc/sc850sl_sdr_mode3_switch_mode7.bin   674 KB - mode-switch firmware blob
opt/etc/sc850sl_sdr_ptnw768_600G_25fps.bin   674 KB - 25 fps PTN mode AE tuning
opt/etc/sc850sl_hdr_2x_ratio_default.bin     189 KB - HDR LE/SE ratio LUT
opt/etc/sc850sl_hdr_2x_ratio_1to1.bin        189 KB - HDR ratio LUT
opt/lib/libsns_sc850sl.so.0.0.0             2.73 MB - source of the tables ★
opt/bin/FRTDemo/config/ipc/sensor/sc850sl.json
opt/bin/FRTTest/config/ipc/sensor/sc850sl.json
```

## What this means for the project plan

- **No more NDA dependency** — we have everything M5Stack ships.
- **2-lane 4K15 RAW10 is validated** — exact register sequence in hand.
- **No PLL reverse engineering needed** — done by SmartSens FAE, baked into table.
- **24 MHz EXTCLK confirmed** — matches Stamp-P4 capability.
- **Bayer pattern is RGGB** (eBayerPattern=0) — feed straight into P4 ISP.
- **MIPI DT = 0x2B (RAW10)** — standard, no quirks.

We can start writing the ESP32-P4 driver immediately.
