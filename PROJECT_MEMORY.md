# CubeSat Imaging Payload — Project Memory

> Comprehensive project state. Reading this file should be enough to pick
> up where any prior session left off. Last updated: 2026-05-13
> (**Phase 1 milestone passed on real hardware**: ESP-IDF v6.0.1 builds
> clean, flashes, boots, runs sc850sl driver init through I²C probe stage,
> degrades gracefully when sensor not yet connected).

---

## 1. Mission overview

CubeSat imaging payload. Two cameras feed an ESP32-P4 that processes images
on-orbit and serves the spacecraft On-Board Computer (OBC) over RS-485.

### Top-level commands (only two, no local storage)

| Command | Action |
|---|---|
| `CMD_CAPTURE_TO_OBC(camera, format)` | Capture one frame, send JPEG to OBC over RS-485 UART |
| `CMD_CAPTURE_TO_SSTV(camera, overlay_ir)` | Capture one frame, downscale, Robot36-encode → I²S → VHF |
| `CMD_PING` | Health/telemetry |
| `CMD_ABORT` | Stop in-progress task |

P4 has **no eMMC, no LittleFS**, no on-board photo library. OBC is sole storage.

### Operating modes

- **Full-res photo path**: 4K (or quad-tile 4K) JPEG → RS-485 → OBC → S-band downlink
- **SSTV path**: capture → 320×240 RGB → Robot36 PCM @ I²S → audio codec → VHF Tx
- IR overlay (optional): MI1602 thermal as picture-in-picture inside RGB

---

## 2. Hardware

### MCU: ESP32-P4 (M5Stamp-P4 module — **target flight version**, currently out of stock)

- Chip: ESP32-P4NRW32 — dual-core RISC-V @ 360 MHz + LP core @ 40 MHz
- **16 MB flash, 32 MB octal PSRAM** (critical for 4K frames)
- 44 GPIO via 1.27 mm / 2.00 mm castellated holes
- **MIPI CSI-2: 2-lane** (AXE516127D BTB connector) — matches sensor 2-lane mode
- SDIO 4-bit (SDIO0–SDIO3) on BTB — not used (no eMMC)
- USB 2.0 HS, RMII Ethernet
- 5 V input, 3.3 V IO, 30 mA active / 360 µA deep-sleep
- **Hardware ISP** (max 1920×1080), Hardware JPEG encoder, PPA (resize/rotate/CSC)

Docs: <https://docs.m5stack.com/en/core/Stamp-P4>

### Available dev boards (Stamp-P4 stand-ins for the bench)

The Stamp-P4 is currently unavailable; firmware bring-up is happening on
multiple larger dev boards that share the same ESP32-P4 silicon:

| Board | Role | Notes |
|---|---|---|
| **M5Stack Tab5** | ⭐ **primary integration platform** | All four pipelines in one box: P4 + SC2356 cam + ES8388 audio codec + 5″ 1280×720 MIPI-DSI display + WiFi 6. Can validate ~70 % of project without flight hardware. |
| Olimex ESP32-P4-DevKit | secondary, for SC850SL-specific bring-up | RPi-camera-compatible 15-pin FFC CSI, built-in USB-C JTAG, microSD, open-hardware schematics, 16 MB flash matches Stamp-P4 |
| Waveshare ESP32-P4-WIFI6 (WAVE-31647) | tertiary | 32 MB flash, ESP32-C6 onboard for Wi-Fi 6/BLE, 40-pin RPi-Pico-HAT-compatible header |

The Olimex/Waveshare boards have **rev v1.0 silicon** (verified by esptool:
`chip revision: v1.0`); must build with
`CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y` since ESP-IDF v6.0's stock default
targets v3.0+. Tab5 silicon revision TBD (probably also v1.0 — confirm
on first flash). See section 8 gotcha #11.

MAC of the active Olimex/Waveshare dev board: `60:55:f9:fa:ff:1d`.

### Why Tab5 changes the development trajectory

Tab5 lets us exercise the **complete signal chain** of the CubeSat
imaging payload on a single battery-powered handheld device:

| Tab5 part | Replaces (during bench dev) | What's verified |
|---|---|---|
| ESP32-P4 (16 MB flash, 32 MB PSRAM) | Stamp-P4 flight version | Same SoC, same firmware |
| **SC202CS** (2 MP MIPI-CSI, SmartSens family — Tab5 marketing says SC2356, M5Tab5-UserDemo names it SC2336 in code; the actual silicon PID readback is `0xeb52` = SC202CS, verified in §8.5) | SC850SL | CSI host, P4 ISP, JPEG encoder, PPA, full sensor-driver framework |
| **ES8388** I²S codec | VHF radio MIC input | I²S DAC path, Robot36 PCM generation, audio waveform correctness |
| MIPI-DSI display (1280×720) | RS-485 → OBC → S-band | Live frame preview, debug visualization, no SD-card or RS-485 dump needed |
| Speaker + mic | (loop-back testing) | "TX SSTV out speaker → mic → PC decoder" round-trip integrity |

**Critical SmartSens-family insight**: SC202CS, SC2336, and SC850SL all
share the same register architecture (16-bit reg + 8-bit val SCCB / I²C,
PLL preamble at 0x36e9/0x36f9, lane select at 0x3018, same DT codes,
same AEC/AGC names). Same I²C address default (`0x30`).

Espressif's official `esp_cam_sensor` library has a **full production
driver for SC202CS** (Apache-2.0, used in our Tab5 build). It also has
a separate **SC2336 driver** that M5Tab5-UserDemo references — but
M5Stack mislabeled the chip; their code path doesn't match the actual
silicon and we don't use it. `esp_cam_sensor` does NOT yet include
SC850SL, so our hand-rolled driver remains necessary for the flight
target. Strategy:

- **Tab5 bench work** uses `esp_cam_sensor` + `esp_video` (V4L2-style API)
  with the upstream SC202CS driver auto-detected on bus.
- **Flight firmware** uses our `firmware/components/sc850sl/` (already
  written, 4 init tables baked in).
- A Kconfig switch in `main/` selects between the two at build time.

See `TAB5_FINDINGS.md` for the complete pinout/wiring/API trail.

### Camera adapter problem (still open)

M5Stack ships the SC850SL CamModule on a **30-pin custom connector**, not
the 15-pin 1 mm FFC standard. Neither dev board accepts it directly.
Three resolution paths:

1. Develop firmware against **OV5647 (RPi Camera v1.3)** in the interim —
   it's the standard "hello world" sensor for ESP32-P4 (Espressif's own
   `examples/peripherals/camera/mipi_isp_dsi` targets it). Olimex + OV5647
   = plug-and-play. ~$10, 5 MP, sensor-side ISP. Decision: **do this now.**
2. Build a 30-pin ↔ 15-pin FFC adapter PCB. Long-pole but the SC850SL
   carrier-PCB net-list will reuse the same routing rules.
3. Replace the M5Stack module with a SmartSens-shipped SC850SL evaluation
   board if one with a standard 15-pin connector exists.

### Visible camera: SmartSens SC850SL

- 8 MP (3840×2160 active, 3856×2176 raw), 1/1.8″, 2.0 µm pixel
- Per-lane MIPI ≤ 1.5 Gbps
- **17.5 mm lens** → HFOV 24.8°, VFOV 14.2° (LEO 400 km → ~90 m/pixel)
- Power rails: AVDD 2.8 V / DOVDD 1.8 V / DVDD 1.3 V
- I²C address 0x30 (SID has internal pull-down; leave NC)
- Chip ID readback: `0x3107=0x9d`, `0x3108=0x1e`
- Active power: 395 mW @ 30 fps / 448 mW @ 60 fps; XSHUTDN-low ≈ 0
- Power-on sequence: DOVDD → DVDD/AVDD → ≥ 4 ms → XSHUTDN release
- Bayer: RGGB
- FPC + power circuits **already built** by the user

### Thermal camera: Meridian Innovation MI1602M5S (narrow FOV)

- 160×120 LWIR microbolometer, **shutterless, wafer-level vacuum-packed**
  (already in vacuum — perfect for space)
- **NFOV 45°/34°/56° H/V/D** (M5S variant; user's choice)
- Pixel pitch 17 µm
- Max 25 fps (NETD scales ~√(1/fps); 1 fps → 35 mK, 25 fps → ~175 mK)
- SPI interface, 3.3 V VDD, 3 MHz SYSCLK
- Radiometric output (16-bit per pixel temperature)

### Field-of-view mismatch ⚠

RGB 24.8°×14.2° is **narrower than IR 45°×34°** (factor 0.55× / 0.42×).
Unusual geometry: RGB looks like a zoomed-in window inside the IR scene.

**SSTV overlay choice**: Picture-in-picture (Method B) — IR fills the full
320×240 frame, RGB sits in the center as a ~176×84 inset corresponding to
its actual FOV inside the IR scene.

Calibration: one-time ground checkerboard shot in both bands → solve 3×3
affine homography → hard-code into firmware.

### Communications

- **OBC ↔ P4**: RS-485 UART (full-duplex differential)
  - Transceiver: **MAX3491E** (12 Mbps, ±15 kV ESD, 0.8 mA Iq).
    Use ADM2587E with isolation if OBC is on different power domain.
  - Speed: start at 921 600 baud → ~32 s per 3 MB JPEG; can push to 2 Mbps
  - Protocol: framed (STX + LEN + CMD + SEQ + PAYLOAD + CRC16 + ETX);
    consider [CubeSat Space Protocol (CSP)](https://github.com/libcsp/libcsp)
    for OBC interop. M5Stack's own StackFlow uses JSON-over-UART — viable
    reference: <https://github.com/m5stack/StackFlow>
- **VHF radio**: not selected yet. PTT polarity, MIC input level, CTCSS
  requirement TBD.
- **Audio codec**: not selected yet. PCM5102 / ES8311 / equivalent.
- **S-band downlink**: P4 doesn't touch it. P4 streams JPEG to OBC over
  RS-485; OBC handles S-band path.

---

## 3. Critical sensor configuration (extracted from M5Stack LLM630 BSP)

### Target operating point: 4K15 RAW10 2-lane

From `rootfs_files/opt/etc/sc850sl_sdr_2lane_15fps.ini`:

```
Resolution  : 3840 × 2160 (full sensor, NO crop)
Frame rate  : 15 fps  (single-shot capture: 67 ms per frame)
Bit depth   : RAW10
MIPI lanes  : 2
Per-lane    : 1080 Mbps  ← well under P4's 1.5 Gbps ceiling
EXTCLK      : 24 MHz  ← generated by P4 LEDC (no external XO needed)
Bayer       : RGGB
MIPI DT     : 0x2B (RAW10)
Pixel format: 0x85 (RAW10_PACKED)
Lane map    : data[0,1,3,4], clock[2,5]  (M0CN/M0CP for clock-lane 0)
```

**This mode is production-validated by M5Stack on LLM630.**

### Four register init tables (in `sc850sl_table_{0..3}.c`)

Extracted byte-for-byte from `libsns_sc850sl.so` `.data` segment, stored as
`{u16 reg, u8 val}` arrays. Each ~190 entries.

| File | Lane | Bits | HTS | VTS | Purpose |
|---|---|---|---|---|---|
| `sc850sl_table_0.c` | 4 | RAW10 | 1100 | set@runtime | 4K30 SDR (M5Stack default) |
| `sc850sl_table_1.c` | 4 | RAW10 | 550  | 4500 | HDR-DOL 4K30 |
| **`sc850sl_table_2.c`** | **2** | **RAW10** | **1100** | **2250** | **★ TARGET MODE** |
| `sc850sl_table_3.c` | 2 | RAW10 | 2200 | 2250 | Alt 2-lane (extended HTS) |

### Key register diffs: 4-lane → 2-lane (table_0 vs table_2)

| Reg | 4L | 2L | Meaning |
|---|---|---|---|
| 0x3018 | 0x7a | **0x3a** | lane bits[7:5]: 011 → 001 |
| 0x3019 | 0xf0 | 0xfc | aux |
| 0x301a | — | 0x30 | added in 2L |
| 0x36e9 | 0x24 | 0x20 | PLL commit |
| 0x36ea | 0x09 | 0x17 | PLL N0 — completely different |
| 0x36fa | 0x0b | **0xcb** | MIPI PLL — massive change |
| 0x36fb | 0x33 | 0x13 | MIPI PLL |
| 0x36fc | 0x10 | 0x00 | MIPI PLL |
| 0x36fd | 0x37 | 0x07 | MIPI PLL |

**Conclusion**: 2-lane is NOT just `0x3018` flip. SmartSens FAE retuned the
entire MIPI PLL block. Trying to derive 2-lane from 4-lane by hand would have
failed. We were lucky M5Stack ships both tables.

### Other useful base registers (all confirmed)

```
0x0103 = soft reset              (write 0x01, hold ≥ 150 ns)
0x0100 = stream on/off           (bit0)
0x302c = sleep aux               (0x0f enable, 0x00 disable)
0x3107/0x3108 = chip ID (0x9d/0x1e)
0x3208/9 = output width hi/lo
0x320a/b = output height hi/lo
0x3210/1 = X start hi/lo
0x3212/3 = Y start hi/lo
0x320c/d = HTS hi/lo
0x320e/f = VTS hi/lo (bit[6:0] high)
0x3221   = mirror[2:1] / flip[6:5]
0x3018   = MIPI lane count bit[7:5]: 0=1L 1=2L 3=4L 7=2x4L
0x3031   = MIPI DT (data mode): 0x08=RAW8 0x0a=RAW10 0x0c=RAW12
0x3037   = leave at 0x00 (datasheet bit[6:5] decode is misleading)
0x4501[3]= test pattern enable
0x3e00/01/02 = exposure (24-bit)
0x3e06   = digital gain
0x3e08/09 = analog gain coarse/fine
0x5000/0x5002 = DPC enable
```

### Sleep / wake state machine (CubeSat duty cycle)

```
Between passes:    XSHUTDN=0, rails OFF                       (~0 mW)
30 s before pass:  rails ON → XSHUTDN=1 → wait 4 ms
                   → 0x0103=0x01 (soft reset, wait 1 ms)
                   → walk init_table → 0x302c=0x0f, 0x0100=0x00  (idle, ~10 mW)
On capture cmd:    0x302c=0x00 → 0x0100=0x01 → wait 1 frame → 0x0100=0
                   → 0x302c=0x0f                                  (idle)
End of pass:       XSHUTDN=0, rails OFF
```

**Sleep enter/exit order is mandatory** (datasheet 1.4.3 注:顺序不可变更).

---

## 4. ESP32-P4 pipeline architecture

### Hardware pipeline (target)

```
SC850SL ─[MIPI 2L RAW10, 1080 Mbps/L]→ P4 CSI Host
                                             │
                                             ▼ (internal, no PSRAM hop)
                                       P4 ISP (input ≤ 1920×1080!)
                                             ▼  BLC → DPC → BF → Demosaic
                                                → WBG → CCM → Gamma
                                                → Edge → CSC → YUV420
                                             ▼ DMA
                                       PSRAM YUV420 buffer
                                             ▼
                                       P4 JPEG HW encoder
                                             ▼
                                       PSRAM JPEG ── RS-485 ──► OBC
```

### The ISP-size problem & solution

ESP32-P4 ISP max input = **1920×1080**, but SC850SL outputs 3840×2160.
Two operating modes in firmware:

1. **Full-res mode** (for S-band downlink):
   - Stream 4K RAW10 from CSI directly to PSRAM (skip ISP on first pass)
   - Then feed PSRAM RAW into ISP in 4 quadrant tiles (1920×1080 each)
   - Output 4 JPEG quadrants (or 1 stitched JPEG via SW seam blend)

2. **SSTV mode** (for VHF):
   - Center-crop sensor output to 1920×1080 via `0x3208/0x320a/0x3210/0x3212`
   - Or take a downscaled view via PPA after ISP
   - Then PPA scale to 320×240 → Robot36 encode

### Memory budget (32 MB PSRAM)

| Buffer | Size | Lifetime |
|---|---|---|
| RAW10 (full 4K) | 10.5 MB | transient capture |
| YUV420 (working) | 12 MB | ISP output |
| JPEG output | ~3–4 MB | until RS-485 finished |
| MI1602 frame | 38 KB | per IR shot |
| SSTV PCM buffer | 1.2 MB | 36 s × 16 kHz × 16-bit |
| RS-485 DMA buffers | 2 × 4 KB | continuous |
| Stack / heap | few MB | continuous |

Plenty of headroom on the Stamp-P4 32 MB part. The 16 MB variants would be
tight.

### Power budget per phase

| Phase | Duration | Total power |
|---|---|---|
| Idle (P4 LS, sensors off) | 99 % | ~55 mW |
| 4K capture + JPEG | 2 s | ~700 mW peak |
| RS-485 transfer of 3 MB | 30 s @ 921k | ~250 mW |
| SSTV frame TX (VHF active) | 36 s | ~2 W (VHF dominates) |

CubeSat bus likely budgets ≤500 mA @ 5 V (2.5 W) → SSTV mode hits the
ceiling. **Don't capture and SSTV simultaneously**; serialize.

---

## 5. Decisions log

1. **No eMMC, no local storage** — P4 is stateless processor; OBC is store
2. **No video, no timelapse** — single-shot stills only (deferred)
3. **No sensor binning** — full 4K capture + software/hardware downscale
4. **Target mode: 4K15 RAW10 2-lane** — exactly what M5Stack validates
5. **EXTCLK 24 MHz generated by P4 LEDC** — no external XO on carrier;
   uses `LEDC_TIMER_1_BIT` resolution (only one that fits — see gotcha #13)
6. **17.5 mm lens kept** — accept FOV mismatch with IR; do PiP overlay
7. **RS-485 framed protocol** — likely CSP or StackFlow-style JSON
8. **MI1602 narrow FOV (M5S)** — already procured
9. **No FAE / SmartSens NDA needed** — full register tables found in M5Stack public BSP
10. **ESP-IDF v6.0.1** — installed at `C:\esp\esp-idf-v6.0.1\`; verified end-to-end
11. **Olimex ESP32-P4-DevKit** — bench platform during Stamp-P4 shortage;
    Waveshare 31647 as secondary
12. **OV5647 (RPi Camera v1.3)** — bridge sensor for early CSI/ISP/JPEG
    pipeline development while the SC850SL 30-pin adapter is being designed
13. **MI1602 driver is vendored in-tree at `firmware/components/mi1602/`** —
    moved June 2026 from the old `../MI1602 Dev/` sibling so a clone builds
    standalone; ESP-IDF auto-discovers `components/` (see §10)
14. **Project memory file (this one) is authoritative** —
    `PROJECT_MEMORY.md` is the single resume-point doc; auxiliary docs
    (BREAKTHROUGH_2LANE.md, FINAL_INVENTORY.md, etc.) are historical
    breadcrumbs from earlier sessions
15. **M5Stack Tab5 becomes the primary integration platform** —
    SC202CS (Tab5 marketing labels it SC2356 and M5Tab5-UserDemo uses
    the SC2336 driver; both are wrong — see §8.5 for the PID-readback
    verification) + ES8388 + DSI display + speaker let us validate
    sensor / ISP / I²S / SSTV / audio loop without waiting for SC850SL
    hardware. Olimex remains the SC850SL-specific bring-up board once
    the 30-pin adapter exists.
16. **OV5647 deprioritized** — Tab5's SC202CS covers the same role
    (validate CSI/ISP pipeline) with closer SmartSens-family
    architecture to SC850SL, so no OV5647 detour needed.
17. **For SC202CS on Tab5 we use Espressif's official driver, not a
    hand-rolled one** — `esp_cam_sensor` ships a production-grade
    SC202CS driver (Apache-2.0). Our `sc850sl/` component is exclusive
    to the flight target.
18. **Tab5 cam pinout confirmed from M5Stack source**: MCLK=GPIO36
    (LEDC 24 MHz, 1-bit duty — same as our `sc850sl.c` recipe),
    SDA=GPIO31, SCL=GPIO32 on i2c port 0, RESET/PWDN tied/expander
    (use -1 in cfg). I²S: BCLK=27, MCLK=30, LRCLK=29, DOUT=26, DIN=28.

---

## 5.8 Phase 2 Tab5 demo (2026-05-22) — LIVE END-TO-END on Tab5 hardware

**Status:** working on Tab5 (chip rev v1.3). Display + touch + LVGL +
live SC202CS preview + Robot36 SSTV encoder + ES8388 speaker output all
verified booting cleanly. Pending visual confirmation of preview image
and SSTV audio decode.

### Real boot log (good run)

```
I (2196) sc202cs: Detected Camera sensor PID=0xeb52
I (2198) app/tab5: bsp_camera_start OK
I (2215) app/tab5: cam: driver=MIPI-CSI card=MIPI-CSI
I (2216) app/tab5: cam default: 1280x720 pixfmt=RGBP
I (2217) app/tab5: ✓ camera streaming 1280x720 RGB565 from /dev/video0
I (2245) app/tab5: UI built
I (2246) mi1602_probe: MI1602 aux probe (shared bus from BSP)
I (2247) mi1602: ready: i2c_addr=0x40 spi_host=2 cs=15 rst=-1 dr=-1
```

### Patches required to make this work (and how to recreate them)

All of these are IDF-level or registry-component patches, not project
code. Without them, the Tab5 build will not produce a working firmware.

**Patch 1 — LEDC gamma RAM no-op for P4 v1.x silicon**

File: `C:\esp\esp-idf-v6.0.1\components\esp_hal_ledc\esp32p4\include\hal\ledc_ll.h`

Replace the body of `ledc_ll_set_fade_param_range()` with a no-op that
discards all arguments. The original writes to `LEDC_GAMMA_RAM`, which
only exists on P4 silicon rev v3.0+. On v1.0/v1.3 silicon the write
faults with `Store access fault @ 0x500d3440`.

**Patch 2 — esp_ipa CMakeLists.txt path-quoting**

File: `firmware\managed_components\espressif__esp_ipa\CMakeLists.txt`

Replace the space-join of IPA JSON paths with a per-path `-i` arg loop,
and quote the `-o ${ipa_config_source}` / `${ipa_config_py_script}` /
`${python}` macros. Without this, project paths containing spaces
(we have "SC850SL Dev") get split at the space.

**Patch 3 — esp_ipa_config.py single-path semantics**

File: `firmware\managed_components\espressif__esp_ipa\tools\config\esp_ipa_config.py`

Replace `files = input.split()` at line ~1302 with `files = [input]`.
The original script expected the caller to space-join multiple JSON
paths into one `-i` arg; combined with patch 2 we now pass each path
as its own `-i`, so `input` is already a single path.

**Patch 4 — disable PM + tickless idle**

In `firmware/sdkconfig.defaults`:
```
# CONFIG_FREERTOS_USE_TICKLESS_IDLE is not set
# CONFIG_PM_ENABLE is not set
```

Tab5 BSP and esp_codec_dev are unstable under dynamic CPU clock scaling
on v6.0.1. M5Tab5-UserDemo's own sdkconfig also has PM off, so this is
consistent with what M5Stack ships.

**Patch 5 — disable Werror=default-warnings**

In `firmware/sdkconfig.defaults`:
```
CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS=y
```

`esp_io_expander` v1.2.1 emits `-Werror=discarded-qualifiers` on IDF v6.
Will be obsolete when the registry component ships a v6.0-clean release.

**Patch 6 — explicit SC202CS Kconfig**

In `firmware/sdkconfig.defaults`:
```
CONFIG_CAMERA_SC202CS=y
CONFIG_CAMERA_SC202CS_AUTO_DETECT=y
CONFIG_CAMERA_SC202CS_AUTO_DETECT_MIPI_INTERFACE_SENSOR=y
CONFIG_CAMERA_SC202CS_MIPI_RAW8_1280X720_30FPS=y
CONFIG_CAMERA_SC202CS_MIPI_DEFAULT_FMT_RAW8_1280X720_30FPS=y
```

`esp_cam_sensor` defaults ALL sensor drivers to OFF — without explicit
opt-in, no V4L2 device gets created and `/dev/video0` doesn't exist.

### LVGL UI knobs

In `firmware/sdkconfig.defaults`:
```
CONFIG_LV_FONT_MONTSERRAT_14=y    # default
CONFIG_LV_FONT_MONTSERRAT_24=y    # title + button
CONFIG_LV_FONT_MONTSERRAT_28=y    # spare
```

### Tab5-specific hardware facts (verified on rev v1.3 silicon)

- **Panel = ST7123** (not the GT911/ILI9881C that some docs claim) —
  display *and* touch are both on ST7123, BSP auto-detects "board v2"
- **Display res = 720×1280 portrait** (rotated at LVGL layer if needed)
- **SC202CS auto-detect PID = 0xeb52** at I²C addr 0x36 on bus port 1
- **Native CSI default = 1280×720 RGBP (RGB565 packed)**, ISP-converted
  from RAW8 sensor output
- **EXT_I2C (port 1, GPIO53/54)** is shared by touch, sensor, IO
  expander, MI1602 if attached. BSP grabs it first; we must use
  `bsp_i2c_get_handle()` rather than open our own port-1 bus.
- **Audio path**: `bsp_audio_codec_speaker_init()` →
  `esp_codec_dev_open(16k mono 16b)` → `esp_codec_dev_write()` →
  ES8388 → on-board 1 W speaker. No manual I²S setup needed.

### Boot sequence in the demo

1. `app_run()` allocates PSRAM bufs (PCM 1.2 MB, snapshot 154 KB,
   preview 460 KB), creates preview mutex
2. `bsp_display_start()` → LVGL + ST7123 panel + ST7123 touch
3. `cam_init()`:
   - `bsp_camera_start()` calls `esp_video_init()` → enumerates SC202CS
   - `open("/dev/video0")`
   - `VIDIOC_G_FMT` → 1280×720 RGBP, S_FMT same with explicit RGB565
   - `VIDIOC_REQBUFS` × 2, mmap, QBUF×2, STREAMON
4. `xTaskCreate(cam_task)` — pumps DQBUF → scale → QBUF at ~20 fps
5. `build_ui()` creates LVGL widgets + 33 ms preview-repaint timer
6. `mi1602_try_probe(bsp_i2c_get_handle())` — shared bus, non-fatal
7. Idle 10 s sleeps; LVGL has its own task; button event fires
   `demo_task()` which snapshot/encode/play

### Path-with-spaces lesson

Three of the patches above (CMake quoting, Python single-path semantics,
LEDC gamma) only became visible because our project folder is named
`SC850SL Dev` with a space. ESP-IDF supports spaces in `PROJECT_PATH`
in principle, but third-party registry components (esp_ipa here) often
don't. If we ever onboard another developer, **strongly recommend
their project folder be space-free** (e.g. `C:\dev\sc850sl\`).



End-to-end Tab5 SSTV demo with **REAL camera capture** (not a test
pattern). **Compiles clean (337 KB), not yet flashed for visual
verification** — Tab5 disconnected from USB at the moment of attempted
flash (user reconnects in a fresh session).

**Demo flow on real hardware:**
1. Init LVGL + 1280×720 MIPI-DSI display + GT911 touch (`bsp_display_start()`)
2. Init SC202CS via `bsp_camera_start()` → esp_video brings up CSI + ISP
3. Open `/dev/video0`, set V4L2_PIX_FMT_RGB565 at the sensor's native size,
   REQBUFS 2 buffers, mmap, STREAMON
4. Background `cam_task` (prio 6) DQBUFs → nearest-neighbour scales to a
   640×360 preview buffer in PSRAM → QBUFs (≈ 20 fps)
5. LVGL `lv_image_t` widget shows the live preview, repainted by a
   33 ms `lv_timer` callback
6. Button press → `demo_task`:
   - downscales 640×360 preview → 320×240 snapshot
   - `sstv_robot36_encode()` → ~580 k mono int16 PCM in 1.2 MB PSRAM
   - `bsp_audio_codec_speaker_init()` → `esp_codec_dev_write()` → onboard 1 W speaker

### What was built

```
firmware/
├── components/sstv_robot36/        ★ NEW — real Robot36 encoder
│   ├── CMakeLists.txt
│   ├── include/sstv_robot36.h      (API: sample_count, encode)
│   └── sstv_robot36.c              (~300 lines: VIS, sync, Y/R-Y/B-Y scanlines)
├── main/
│   ├── app_sc202cs_tab5.c           ★ REWRITTEN — full demo (LVGL UI + button + audio)
│   ├── idf_component.yml           ★ UPDATED — pulls full Tab5 BSP stack
│   └── CMakeLists.txt              ★ UPDATED — Tab5 deps in REQUIRES (gated)
└── sdkconfig.defaults              ★ UPDATED — CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS=y
```

### Registry components pulled (managed_components/, 22 total)

```
espressif/m5stack_tab5             1.2.0~1   (Tab5 BSP: display+touch+audio+camera+IO)
espressif/esp_video                ~2.0      (V4L2 layer, v6.0-compatible release)
espressif/esp_cam_sensor                     (with SC202CS driver — real Tab5 sensor)
espressif/esp_lvgl_port            ^2.5      (LVGL ↔ esp_lcd glue)
espressif/esp_codec_dev                      (ES8388 + I²S abstraction)
espressif/esp_lcd_ili9881c                   (MIPI-DSI panel driver)
espressif/esp_lcd_touch_gt911                (multi-touch)
espressif/esp_lcd_st7123                     (alt panel — auto-detect)
espressif/esp_sccb_intf, esp_ipa, esp_h264, esp_io_expander*, bmi270,
i2c_bus, sensor_hub, usb*, usb_host_uvc, cmake_utilities
lvgl/lvgl                          ~9.2.0
```

### Where it stands in the code

`app_sc202cs_tab5.c` (`CONFIG_CAMERA_TARGET_SC202CS_TAB5=y` selects this path):

- `app_run()` allocates PSRAM buffers (PCM 1.2 MB, snapshot 154 KB,
  preview 460 KB), creates preview mutex, brings up display + camera +
  background `cam_task`, builds UI under LVGL lock, launches
  `mi1602_try_probe()` (non-fatal), then sleeps forever.
- `cam_init()` does the full V4L2 dance: BSP camera, open device,
  G_FMT/S_FMT to RGB565, REQBUFS 2, mmap, QBUF×2, STREAMON.
- `cam_task()` runs forever: DQBUF → mutex-guarded `scale_rgb565_nn()`
  into `g_preview_buf` → QBUF; ~50 ms throttle for LVGL headroom.
- `preview_tick()` (LVGL 33 ms timer) just invalidates `g_preview_img`
  so the panel repaints with whatever's in `g_preview_buf`.
- `on_capture_press()` (LVGL CLICKED event) spawns `demo_task`.
- `demo_task` takes the mutex, downscales preview → 320×240 snapshot,
  encodes Robot36, opens codec, writes ~36 s of PCM, closes codec.

### Audio path (confirmed to onboard 1W speaker)

`bsp_audio_codec_speaker_init()` from Tab5 BSP returns a codec_dev handle
for the speaker output. Internally:

```
[esp_codec_dev_write PCM]
  → ES8388 codec (I²S RX from P4)
  → onboard PAM-class 1 W amplifier (BSP enables it via IO expander)
  → onboard mono speaker
```

No external GPIO / amp_enable wiring needed at the application layer.

### The compiler-errors-disabled workaround (gotcha #19)

ESP-IDF v6.0 promoted several previously-warning checks to errors
(`-Werror=discarded-qualifiers`, etc.). Some registry components
(notably `esp_io_expander` v1.2.1) still emit these warnings on v6.0.1.
We set `CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS=y` to keep them as
warnings only. Espressif's release notes call this option transitional
and intend to remove it; we'll revisit once the offending components ship
v6.0-clean releases.

### What's NOT done yet — pickup points for next session

- [ ] **Flash the built firmware to Tab5.** Binary is at
      `firmware/build/cubesat_imager.bin` (337 KB). Connect Tab5 USB,
      re-detect COM port (was COM15, may have changed), then
      `idf.py -p COMxx flash monitor`.
- [ ] **Visual verification**:
      - LVGL UI renders on 1280×720 panel?
      - Live SC202CS frames show up in the preview area (auto-detected,
        no Kconfig switch needed — `esp_cam_sensor` includes the driver)?
      - Button press encodes Robot36 in ~3 s and plays through speaker?
      - Decode with a phone running RX-SSTV / Robot36 to confirm the
        transmitted image is a faithful copy of what was on the preview.
- [ ] **First-frame race**: if `bsp_camera_start()` succeeds but ISP
      takes >1 s to deliver a frame, the user might see "no frame yet"
      momentarily on first button press. Currently we just gate the
      demo_task on `g_have_frame`. Could be improved with a blocking
      semaphore wait.
- [ ] **RGB565 byte order verification**: P4 ISP may emit big-endian
      RGB565 while LVGL expects little-endian (or vice versa). If the
      preview shows weird colors, swap bytes in `scale_rgb565_nn()`.
- [ ] **MI1602 IR overlay** (PiP per FOV math in §2)
- [ ] **RS-485 protocol** (no OBC link yet)
- [ ] **Full SC850SL path** validated on the Olimex board with real FPC

### Files modified or created this session (snapshot)

```
firmware/main/app.h                      (+mi1602_try_probe proto)
firmware/main/app.c → main.c             (refactored to dispatch)
firmware/main/app_sc850sl.c              (NEW — SC850SL probe app)
firmware/main/app_sc202cs_tab5.c          (REWRITTEN — full LVGL+SSTV demo)
firmware/main/mi1602_probe.c             (NEW — shared MI1602 probe)
firmware/main/Kconfig.projbuild          (NEW — CAMERA_TARGET + Tab5 + MI1602 pins)
firmware/main/CMakeLists.txt             (updated REQUIRES)
firmware/main/idf_component.yml          (pulls Tab5 BSP + LVGL stack)
firmware/components/sstv_robot36/...     (NEW — real encoder, 300 lines)
firmware/sdkconfig.defaults              (+CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS=y)
PROJECT_MEMORY.md                        (this update)
```

---

## 5.6 Phase 1+ extension: MI1602 parallel probe (2026-05-13 evening)

After Phase 1 (SC850SL probe) passed on real hardware, the MI1602 driver
written in the sibling folder (`../MI1602 Dev/components/mi1602/`) was
wired into the same firmware via a shared `main/mi1602_probe.c` helper.
Both probe paths run sequentially in `app_sc850sl.c` and (when selected)
`app_sc202cs_tab5.c`.

Real-hardware boot log on Olimex with no carriers attached:

```
I (1188) main: CubeSat imager — built for SC850SL (flight target)
I (1252) app/sc850sl: probe failed: ESP_ERR_INVALID_RESPONSE
I (1252) mi1602_probe: MI1602 aux probe: i2c port=1 SDA=53 SCL=54
I (1253) mi1602: ready: i2c_addr=0x40 spi_host=2 cs=15 rst=-1 dr=-1
E (1254) mi1602: probe: I²C read FW_VERSION_1 failed: ESP_ERR_INVALID_RESPONSE
W (1254) mi1602_probe: MI1602 probe NACK at 0x40 — IR sensor offline
```

Both NACKs are graceful. Firmware continues into heartbeat.

Verified:

- ✅ Sibling-folder MI1602 driver (5 C files, 331 KB static lib) builds
  zero-warning under ESP-IDF v6.0.1
- ✅ EXTRA_COMPONENT_DIRS auto-detection works across drive paths with spaces
- ✅ `mi1602_init()` succeeds end-to-end without IR module connected
  (creates SPI bus, configures host, allocates resources)
- ✅ I²C 0x40 NACK is handled as a recoverable warning, not a crash
- ✅ Binary size: 278 KB (+35 KB over Phase 1; 93% partition free)
- ✅ MI1602 default I²C pins (port 1, GPIO53/54) match Tab5 EXT_I2C — IR
  module plugs into Tab5 back header directly

Open items:

- [ ] Pick real GPIO assignments per actual carrier (Olimex + IR dev
      board OR Tab5 + IR dev board); override via `idf.py menuconfig` →
      "MI1602 thermal camera"
- [ ] Phase 4: replace probe-only with frame capture + SSTV pipeline
- [ ] After MI1602 driver is polished by the user, run the full test
      sequence with IR module physically connected (chip ID, bootup,
      single-frame capture, header parse)

## 5.5 Phase 1 bring-up status (as of 2026-05-13)

End-to-end verification complete on a v1.0-silicon ESP32-P4 dev board
(Olimex/Waveshare). The boot log captured on real hardware:

```
I (1170) cpu_start: cpu freq: 360000000 Hz
I (1171) app_init: ESP-IDF:          v6.0.1
I (1174) efuse_init: Chip rev:         v1.0
I (1176) esp_psram: Adding pool of 32768K of PSRAM memory to heap allocator
I (1182) main: CubeSat imager booting (Phase 1)
I (1182) main: I²C master open: SDA=8 SCL=9
I (1184) sc850sl: EXTCLK 24000000 Hz on GPIO11 (LEDC ch0)
I (1196) sc850sl: powered, addr=0x30, freq=100000 Hz, mode=2
E (1246) sc850sl: sc850sl_probe(216): id hi
W (1246) main: probe failed: ESP_ERR_INVALID_RESPONSE — running without sensor
```

Verified components:

- ✅ Toolchain at `C:\esp\esp-idf-v6.0.1\` (sourced via `export.ps1`)
- ✅ Python 3.13.9 venv at `C:\Users\zeyu.zhu\.espressif\python_env\idf6.0_py3.13_env\`
- ✅ All four firmware components compile (`sc850sl`, `isp_pipeline` stub,
  `rs422_protocol` stub, `sstv_robot36` stub) plus `main`
- ✅ Top-level `EXTRA_COMPONENT_DIRS` auto-detects MI1602 sibling folder
  when present (currently absent → build continues without it)
- ✅ Bootloader 0x6030 / 0x8000 (57 % free with partition-table offset
  pushed to 0x10000)
- ✅ App image 0x3c460 / 0x400000 (94 % free in 4 MB factory partition)
- ✅ Flash speed 867 kbit/s over USB-Serial-JTAG
- ✅ CPU 360 MHz, PSRAM 32 MB @ 200 MHz, all heaps initialized
- ✅ I²C master peripheral opens cleanly on SDA=GPIO8, SCL=GPIO9
- ✅ LEDC channel 0 outputs **24 MHz** on GPIO11 with 1-bit duty
- ✅ SC850SL driver state machine reaches the I²C-probe step
- ✅ Probe NACK handled by `ESP_ERR_INVALID_RESPONSE` path; firmware
  enters heartbeat loop instead of crashing

What's NOT verified yet (needs hardware):

- ❌ Actual SC850SL chip-ID readback (need carrier or adapter)
- ❌ MIPI CSI lane integrity (need scope or working stream)
- ❌ Full register init table walk (190 writes / mode)
- ❌ ISP / JPEG / PPA pipelines (not yet wired in `isp_pipeline` component)
- ❌ MI1602 driver (not yet implemented in sibling folder)
- ❌ RS-485 protocol layer (not yet implemented)
- ❌ SSTV Robot36 encoder (not yet implemented)
- ❌ Power-rail sequencing on the actual carrier (current main.c sets
  rail_en_* = -1 because dev boards have always-on rails)

## 6. File inventory (kept in this folder)

```
PROJECT_MEMORY.md            ⭐ THIS FILE — read first to resume work

firmware/                    ⭐⭐ ESP-IDF v6.0.1 project — already builds clean & runs Phase 1
  ├── CMakeLists.txt           top-level; auto-detects MI1602 sibling folder
  ├── sdkconfig.defaults       locked-in settings (P4 silicon rev fix, PSRAM, partitions)
  ├── partitions.csv           16 MB flash layout, table at 0x10000
  ├── README.md / HOW_TO_BUILD.md / MI1602_INTEGRATION.md
  ├── capture_serial.py        one-shot serial reader for boot-log capture
  ├── main/main.c              Phase 1 milestone code (I²C probe + heartbeat)
  └── components/
      ├── sc850sl/             SC850SL driver (init tables 0-3 baked in)
      ├── isp_pipeline/        stub for Phase 2 (CSI host + ISP + JPEG)
      ├── rs422_protocol/      stub for Phase 5 (OBC comms)
      └── sstv_robot36/        stub for Phase 4 (audio encoder)

datasheets / references:
  SC850SL_数据手册_V1.10.pdf    Original SC850SL datasheet (Chinese)
  datasheet_utf8.txt           Extracted text (UTF-8) of above
  mi1602.pdf                   MI1602 datasheet
  mi1602.txt                   Extracted text of above
  sensor_sc850sl_mipi.c        OpenIPC SigmaStar reference driver (1951 lines)

⭐ register tables (THE GOLD):
  sc850sl_table_0.c            4K30 4-lane SDR (M5Stack default)
  sc850sl_table_1.c            4K30 4-lane HDR-DOL
  sc850sl_table_2.c            ★ 4K15 2-lane SDR (TARGET MODE)
  sc850sl_table_3.c            4K15 2-lane SDR (alt HTS)

M5Stack BSP artifacts (rootfs_files/):
  opt/etc/sc850sl_sdr_2lane_15fps.ini      ⭐ target config
  opt/etc/sc850sl_sdr_4lane.ini            (4L reference)
  opt/etc/sc850sl_hdr_4lane.ini            (HDR reference)
  opt/etc/sc850sl_sdr_ptnw768_600G_25fps.bin   AX620E ISP param blob, NOT sensor fw (674 KB) — see §12.1
  opt/etc/sc850sl_sdr_mode3_switch_mode7.bin    AX620E ISP param blob, NOT sensor fw (674 KB) — see §12.1
  opt/etc/sc850sl_hdr_2x_ratio_*.bin       AX620E ISP HDR param blobs (hdr "AX620E_ISP_V4.0.31") — see §12.1
  opt/lib/libsns_sc850sl.so.0.0.0          source of the tables (2.7 MB)
  opt/bin/FRTDemo|FRTTest/.../sc850sl.json (app configs)

custom headers / docs:
  sc850sl_registers.h          C header with register defines
  FINAL_INVENTORY.md           Detailed file/finding inventory
  BREAKTHROUGH_2LANE.md        2-lane mode discovery write-up
  M5STACK_CONFIG.md            llm-camera binary attribute decode
  TWO_LANE_DERIVATION.md       Original (pre-breakthrough) derivation plan

DEB packages (reference, can re-extract):
  llm-camera_1.9-m5stack1_arm64.deb    M5Stack app binary
  llm-sys_1.6-m5stack1_arm64.deb       UART/ZMQ bridge daemon

Python tooling:
  extract_deb.py               unpack Debian packages (no `ar` needed)
  parse_sparse.py              Android sparse-ext4 → ext4
  extract_rootfs_files.py      Walk ext4, follow symlinks, extract
  dump_init_tables.py          ⭐ extract register tables from .so
  dump_sc850sl_attrs.py        decode AX_SNS_*_ATTR_T structs from ELF
  analyze_camera.py            string-scan & byte-pattern search
```

DELETED (no longer needed — extraction complete):
- `M5_LLM_ubuntu22.04_*.axp` (3.3 GB image)
- `rootfs.ext4` (6.2 GB physical, 30 GB sparse)
- `lib-llm_*.deb` + `extracted_lib_llm/` (AI/speech runtime, irrelevant)
- `axp_extracted/` (bootloaders, kernel, optee — for AX630C not P4)
- `extracted*/` (already analyzed; relevant facts captured in .md files)
- Duplicate datasheet PDF

---

## 7. Open items / next steps

### Phase 0: Hardware confirmations (need user input)

- [ ] VHF transceiver model — drives PTT polarity, MIC input level, CTCSS
- [ ] Audio codec selection — PCM5102 / ES8311 / other?
- [ ] OBC ↔ P4 baud rate target — start 921 600
- [ ] Will OBC speak CSP, JSON-over-UART, or custom protocol?
- [ ] Carrier PCB: verify 27 MHz vs 24 MHz EXTCLK source (we go 24) **[NOT NEEDED on dev boards — LEDC GPIO11 confirmed working]**
- [ ] FPC lane mapping: confirm physical SC850SL pins M0D0/M0D1 + M0CN/CP are
      routed to Stamp-P4 BTB MIPI data 0/1 + clock
- [ ] 30-pin → 15-pin FFC adapter for M5Stack SC850SL CamModule (or commit
      to one of the other adapter paths in section 2)

### Phase 1: Bench bring-up

- [x] **Install ESP-IDF v6.0.1, verify toolchain end-to-end**
- [x] **Build firmware clean, flash to Olimex/Waveshare dev board**
- [x] **Verify CPU/PSRAM/Flash/I²C/LEDC peripherals via main.c milestone code**

### Phase 2 (Tab5): end-to-end pipeline on integrated platform

- [x] Add Kconfig switch `CAMERA_TARGET_*` (SC850SL flight vs SC202CS Tab5)
- [x] Tab5 stub builds cleanly; MCLK 24 MHz on GPIO36 ready for scope check
- [x] **Unblock Tab5 capture path** (resolved in §8.5):
      `esp_cam_sensor` SC202CS driver works on IDF v6.0.1 once we enable
      `CONFIG_CAMERA_SC202CS=y` + auto-detect. (The earlier hand-roll
      plan via `esp_cam_ctlr_csi` is no longer needed.)
- [x] Wire SC202CS init via `esp_video_init_csi_config_t` with Tab5 pins
      (SDA=31, SCL=32, MCLK=GPIO36 via LEDC, reset/pwdn=-1)
- [ ] Capture first frame via V4L2 `ioctl(VIDIOC_DQBUF)` → PSRAM buffer
- [ ] HW JPEG encode → dump over USB CDC OR display on Tab5 DSI
- [ ] Wire up ES8388 driver via Espressif's `audio_codec` component
- [ ] Generate 1 kHz sine on I²S → ES8388 → speaker (audio sanity)
- [ ] Implement Robot36 SSTV encoder, play out speaker, decode with
      `qsstv` or `MMSSTV` on PC mic → verify image round-trips
- [ ] Implement RS-485 protocol over USB-CDC (treat USB CDC as the
      RS-485 link during dev) — frames go to a PC test app

### Phase 3 (Olimex / SC850SL real hardware)

- [ ] 30-pin → 15-pin FFC adapter PCB designed, fabbed, populated
- [ ] Power up rails per sequence; verify ≤4 ms timing on scope
- [ ] I²C probe at 0x30; read chip ID 0x9d 0x1e
- [ ] Apply `sc850sl_table_2[]` writes; enable test pattern `0x4501[3]=1`
- [ ] Configure P4 CSI host for 2L RAW10 @ 1080 Mbps/L
- [ ] Capture one test-pattern frame to PSRAM; dump over USB CDC
- [ ] Disable test pattern; capture real scene; verify Bayer RGGB

### Phase 4 (parallel track): MI1602 thermal driver

Developed standalone, now vendored in-tree at `firmware/components/mi1602/`
(June 2026; auto-discovered, no EXTRA_COMPONENT_DIRS — see §10).

- [ ] SPI master setup (3 MHz SYSCLK gen via LEDC or clock-output, RSTN handling)
- [ ] Frame capture loop in radiometric (16-bit/pixel) mode
- [ ] Iron palette LUT + min/max temperature stretch
- [ ] PPA helper: scale 160×120 → 320×240 for SSTV
- [ ] Picture-in-picture composer (RGB inset inside IR background)
- [ ] One-time IR↔RGB homography calibration on ground

### Phase 5: Carrier PCB + Stamp-P4 flight version

- [ ] Order Stamp-P4 once back in stock
- [ ] Custom CubeSat carrier: SC850SL FPC, MI1602 FPC, ES8388-class codec,
      RS-485 transceiver (TI THVD1424RGTR, full-duplex), VHF radio interface, power rails
- [ ] Bring up firmware on Stamp-P4 (mostly a config-pin re-map)

### Phase 6: Flight hardening

- [ ] Sleep state machine (sensor / P4 / peripherals all gated)
- [ ] Watchdog around every I²C transaction
- [ ] I²C unstick recipe (manual 9-clock pulse on stuck bus)
- [ ] SEU recovery: known-good state on uncommanded reset
- [ ] PSRAM ECC + LittleFS scrub for stored config
- [ ] Thermal vac chamber test plan (-30 °C to +60 °C operating)
- [ ] Radiation total-dose budget review

---

## 8. Notable gotchas discovered

1. **`0x3037` PHY bit-mode field**: datasheet says bit[6:5] selects PHY bit
   mode, but the M5Stack driver writes 0x00 in all RAW10/RAW12 modes.
   Don't trust the datasheet on this register; mirror the driver.
2. **`nSettingIndex` in the .ini is a driver lookup key**, not an opaque ID.
   It selects which of the four `.data`-segment tables to load.
3. **2-lane mode retunes the WHOLE MIPI PLL**, not just the lane-count bit.
   Cannot derive from 4-lane table by inspection.
4. **VTS = 0 at table end** means the driver writes VTS later (from the
   .ini). Our driver must set `0x320e/0x320f` after the table walk.
5. **Sleep enter/exit ordering is enforced**: `0x0100=0` THEN `0x302c=0x0f`
   to sleep; reverse to wake. Datasheet 1.4.3 注: 顺序不可变更.
6. **SID pin internal pull-down** = 0x30 default. Leave NC on the FPC.
7. **MI1602 NETD is at 1 fps** (best case). At 25 fps it's ~5× worse. For
   stills, run at 1 fps.
8. **ESP32-P4 ISP max input 1920×1080** — cannot directly process 4K from
   the sensor; must tile or center-crop.
9. **AX630C BSP packaged the .axp as ZIP** containing Android-sparse ext4.
   `parse_sparse.py` handles it.
10. **The four register tables in libsns_sc850sl.so .data** are at byte
    offsets 0x230, 0x818, 0xe88, 0x1480 (8-byte `{u32 reg, u32 val}` records).
    `dump_init_tables.py` decodes them.

11. **ESP-IDF v6.0 defaults to ESP32-P4 silicon rev ≥ v3.0** — early
    Olimex/Waveshare boards (and our dev one) are rev v1.0. The
    bootloader will refuse to flash with: *"requires chip revision in
    range [v3.1 - v3.99] (this chip is revision v1.0)"*. Fix:
    `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y` in `sdkconfig.defaults`. We
    have this set.

12. **`idf.py fullclean` does NOT delete `sdkconfig`** — only `build/`.
    Kconfig "choice" symbols (like CPU frequency) are written once into
    `sdkconfig` and not re-derived from `sdkconfig.defaults` on
    subsequent builds. Symptom: assert in `clk.c:104` at boot because
    the cached `sdkconfig` still says `CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ_400=y`
    even though defaults now point at 360. **Workaround**: `rm sdkconfig`
    then re-run `idf.py set-target esp32p4 && idf.py build`. Or use
    `idf.py menuconfig` to override the choice explicitly.

13. **LEDC cannot output 24 MHz with 2-bit duty resolution** on
    ESP32-P4. The error reads: *"requested frequency 24000000 and duty
    resolution 2 can not be achieved … div_param=0"*. Math: 2-bit
    needs source ≥ 24 MHz × 4 = 96 MHz, but P4's LEDC sources cap at
    80 MHz. Fix: use `LEDC_TIMER_1_BIT` and `duty=1` (still 50 %
    square wave). Already applied in `sc850sl.c`. Long-term: switch to
    `esp_clock_output_start()` for a glitch-free dedicated clock pin.

14. **IDF v6.0 bootloader is 0x6030 bytes** — pushes past the legacy
    partition-table offset 0x8000 by ~48 bytes. Set
    `CONFIG_PARTITION_TABLE_OFFSET=0x10000` and use auto offsets in
    `partitions.csv`. Already applied.

15. **ESP-IDF v6.x does NOT run from Git Bash / MSYS** on Windows
    (`export.sh` aborts with "MSys/Mingw is not supported"). Use the
    ESP-IDF PowerShell shortcut or call `. C:\esp\esp-idf-v6.0.1\export.ps1`
    in a regular PowerShell. PowerShell execution policy on this box is
    "Restricted" by default, so prepend
    `Set-ExecutionPolicy -Scope Process -ExecutionPolicy Bypass -Force;`.

16. **The board's USB-Serial-JTAG does not respond to classic DTR/RTS reset.**
    pyserial's `setRTS()` toggle does not reboot the chip. To force a
    fresh boot log capture, use `python -m esptool --before usb-reset
    --after hard-reset chip-id` then immediately open the serial port.
    There's a helper `firmware/tools/capture_serial.py` for this.

17. **Firmware logs only at boot**, then enters silent heartbeat. If you
    open the serial port mid-loop, you see nothing. This is by design
    (Phase 1 is a passive milestone). To re-trigger boot output: reset
    the chip (see #16) or hold its reset button if the carrier exposes it.

18. **`espressif/esp_video` v1.x and `espressif/esp_cam_sensor` v1.7.0
    are broken against ESP-IDF v6.0.1.** Compile errors:
    - `'CAM_CTLR_COLOR_YUV422' undeclared` in
      `esp_video_csi_device.c`, `esp_video_dvp_device.c`,
      `esp_video_isp_device.c` (enum removed in v6.0)
    - `-Werror=discarded-qualifiers` in `esp_video_init.c:148/151`
    - `spicommon_dma_chan_alloc / _bus_initialize_io / _bus_free_io_cfg`
      called with wrong arg counts inside `esp_cam_sensor` SPI sub-driver
    These are all "upstream component hasn't been ported to v6.0 yet"
    issues, not user-fixable in our code. Workarounds tried:
    - Setting `CONFIG_CAM_CTRL_SPI_ENABLE=n` removed the SPI driver but
      didn't fix the YUV422 enum issue
    - Pinning to older versions (`<1.0`) made the component manager
      refuse (no compatible version found for ESP-IDF v6.0)
    Resolution: **temporarily removed both deps**; `app_sc202cs_tab5.c` is
    a stub that only starts MCLK for scope verification. Re-add once
    Espressif publishes a v6.0-compatible release (probably with
    `esp_cam_sensor >= 2.0`).

---

## 8.5  Phase 2 — Tab5 bench demo, complete

This section documents the **fully-working Tab5 demo** as of the May 2026
work block. The demo on its own is a non-flight bench tool, but every
patch + Kconfig + algorithm in it is also what the flight target will
need. Treat this as the canonical "how the Tab5 path is wired up" record.

### 8.5.1  Top-of-tree feature set

End-to-end pipeline on M5Stack Tab5 (ESP32-P4 rev v1.3, 32 MB octal
PSRAM, 16 MB flash, ST7123 720×1280 portrait MIPI-DSI display+touch,
ES8388 audio codec out, ES7210 dual-mic in, SC202CS MIPI-CSI camera):

```
SC202CS 1280×720 RGB565 @ 30 fps
        │
        ▼  PPA hardware rotation (CCW 270° = visual CW 90°) + 4:3 crop + scale
        │  src crop = 540×720 (3:4 portrait inside the 16:9 sensor frame,
        │  centered at x=370). Output 640×480 landscape RGB565.
        ▼
   ┌─ g_preview_buf (640×480, DMA-aligned PSRAM) ─ LVGL image, top of screen
   │
   │   ┌── Capture button: scale 2× into g_image_buf (320×240) ──┐
   │   │                                                          ▼
   │   │                                                  g_image_buf (320×240)
   │   │                                                          │
   │   │   ┌── Send button: sstv_robot36_encode() ────────────────┘
   │   │   │   600 K samples (37 s incl. 1.0 s trailing silence)
   │   │   ▼
   │   │  g_pcm_buf (PSRAM, int16 mono @ 16 kHz)
   │   │   │
   │   │   ▼  audio_play() — chunked 200 ms writes so the
   │   │   │   progress bar can tick mid-TX
   │   │  ES8388 DAC → PAM amp → 1 W speaker (out @ 30% volume)
   │   │
   └───┘
                       ▲
                       │
                   I²S full-duplex
                       │
                       ▼
       ES7210 dual-mic → 16 kHz mono int16
                       │
       ┌───────────────┼───────────────┐
       │               │               │
       ▼ FFT 1024-pt   ▼  ZCR per-sample freq estimator (16-sample window)
       │               │
       ▼ waterfall    ▼  Robot36 RX state:
       │ column        │   • per-sample sync detector (1200 Hz ±150 Hz, ≥5 ms)
       │               │   • on lock → snap samples_in_row to 80
       │               │   • Y samples (192..1600) → g_dec.y_curr[320]
       │               │   • chroma samples (1696..2400) →
       │               │       even row: g_dec.cr[160]
       │               │       odd  row: g_dec.cb[160]
       │               │   • pair complete (odd row finished) →
       │               │       memmove image ↑ 2 rows, paint y_prev/y_curr +
       │               │       cr/cb as RGB565 at bottom 2 rows
       │               │
       ▼ LVGL waterfall (640×130 widget, slim spectrum strip)
                       ▼ LVGL decoded image (640×440 widget displayed via
                         LV_IMAGE_ALIGN_STRETCH from 320×240 buffer)
```

### 8.5.2  Patches applied (in addition to the 6 from §5.8)

Beyond the original 6 IDF-v6.0.1 / hardware-specific patches:

7. **PPA hardware rotation** (`main/app_sc202cs_tab5.c`).
   Replaced ~85 ms/frame CPU column-wise PSRAM rotation with the
   ESP32-P4 PPA SRM peripheral (~1 ms/frame). `ppa_register_client` +
   `ppa_do_scale_rotate_mirror` with `rotation_angle =
   PPA_SRM_ROTATION_ANGLE_270` (the IDF enum is CCW, so 270° == visual
   CW 90°). Source block is the 540×720 portrait sub-rectangle of the
   sensor frame; output block fills the whole 640×480 preview at
   `scale_x = scale_y = 8/9 ≈ 0.889`. Output buffer must be
   `MALLOC_CAP_DMA | MALLOC_CAP_SPIRAM` and 64-byte aligned (L2 cache
   line) — used `heap_caps_aligned_calloc(64, …)`.

8. **`pad_all = 0` on lv_image widgets**.
   LVGL's default style includes a few pixels of content padding which
   leaves the right + bottom edges of the image cropped (background
   bleeds through). Setting `lv_obj_set_style_pad_all(img, 0, 0)` makes
   the image touch its border on every side.

9. **No mic pause during TX**.
   The Tab5 BSP shares one I²S peripheral between ES8388 (out) and
   ES7210 (in), but they use independent TX/RX channels — full-duplex
   operation works fine. Removed the `g_tx_active` flag that previously
   paused `mic_task` during `audio_play()`. This is what lets the
   decoder hear (and decode!) the Tab5's own SSTV TX.

10. **LVGL cross-thread invalidate fix**.
    `mic_task` (pinned to CPU 1) was calling `lv_obj_invalidate()`
    without holding the LVGL mutex, which raced against the LVGL task
    on CPU 0 and deadlocked IDLE1 — tripping the task watchdog every
    5 s. Fix: all LVGL invalidate calls live inside the `preview_tick`
    `lv_timer` (which runs in the LVGL task context, so it already
    holds the mutex implicitly). Other threads just write the pixel
    buffer; the LVGL timer picks up the new data at the next 30 Hz
    refresh.

11. **Internal-RAM staging buffer for waterfall scroll**.
    Naïve per-row `memmove` inside the PSRAM waterfall buffer was
    100 + ms/frame (overlapping PSRAM-to-PSRAM with tiny strides).
    Replaced with a 1.3 KB internal-RAM `row_tmp[WF_W]` scratch: read
    each row's "kept" portion into RAM with one `memcpy`, then bulk-
    write back to PSRAM. Drops the cost to ~25 ms/frame.

12. **Robot36 encoder trailing silence**.
    Appended 1.0 s of zero-PCM after the last scanline in
    `sstv_robot36_encode()`. Robot36 has no protocol-defined frame end,
    so mobile decoders use a VOX-style trailing-silence trigger to
    finalize / auto-save. Without the silence the phone keeps the
    frame in "still receiving" state and never saves. Bumped
    `sstv_robot36_sample_count()` total_ms by `MS_TRAILING_SILENCE`
    so the capacity check stays correct.

### 8.5.3  Kconfig gotchas discovered

- **Pinning the camera target in `sdkconfig.defaults`** is mandatory.
  `firmware/main/Kconfig.projbuild` defines a `choice CAMERA_TARGET`
  whose default is `CAMERA_TARGET_SC850SL` (flight). Deleting
  `sdkconfig` to re-derive defaults *silently reverts* to that choice
  unless you also include `CONFIG_CAMERA_TARGET_SC202CS_TAB5=y` and
  `# CONFIG_CAMERA_TARGET_SC850SL is not set` in `sdkconfig.defaults`.
  We hit this once after a `rm sdkconfig`: build produced a 320 KB
  binary that ran the `app_sc850sl.c` body, leaving the display dark
  and the user staring at a blank Tab5. Now pinned permanently.

- **`CONFIG_ESP_VIDEO_ENABLE_ISP_PIPELINE_CONTROLLER=y`** is required for
  esp_ipa to actually close the AE/AWB feedback loop. With it off
  (the v2.0.x default), the IPA collects statistics but never writes
  them back to the sensor / ISP, so the picture stays at boot-time
  defaults (greenish, underexposed). Off-by-default for power, on for
  image quality.

- **`CONFIG_COMPILER_DISABLE_DEFAULT_ERRORS=y`** is still needed because
  esp_io_expander v1.2.1 still emits `-Wdiscarded-qualifiers`. The
  espressif/m5stack_tab5 BSP transitively pulls it; nothing we can do
  upstream.

### 8.5.4  Phase 2 file inventory (`firmware/`)

```
firmware/
├── sdkconfig.defaults     ← target choice, ISP loop, SC202CS Kconfig, etc.
├── _build.bat             ← dev helper: unsets MSYSTEM, sources export.bat,
│                            `idf.py reconfigure && idf.py build`
├── _flash.bat             ← same, then `idf.py -p COM15 flash`
├── _reset_capture.bat     ← esptool usb-reset → capture_serial.py 14 s
├── capture_serial.py      ← one-shot serial reader with UTF-8 stdout
├── main/
│   ├── app_sc202cs_tab5.c  ← THE Tab5 demo (preview + UI + waterfall +
│   │                          Robot36 RX decoder, ≈1300 lines)
│   ├── app_sc850sl.c      ← flight-target stub (Phase 1 milestone)
│   ├── main.c             ← target dispatch
│   ├── mi1602_probe.c     ← MI1602 thermal aux probe
│   ├── Kconfig.projbuild  ← CAMERA_TARGET_* choice + Tab5/MI1602 pin maps
│   ├── CMakeLists.txt     ← REQUIRES: BSP, LVGL, codec_dev. PRIV_REQUIRES:
│   │                          esp_driver_ppa, espressif__esp-dsp
│   └── idf_component.yml  ← m5stack_tab5 ^1.0, esp_lvgl_port ^2.5,
│                              espressif/esp-dsp ^1.5
├── components/
│   ├── sc850sl/           ← flight driver (Phase 1, hand-rolled)
│   └── sstv_robot36/      ← Robot36 ENCODER (compute-side TX path)
└── managed_components/    ← (gitignored) esp_video, esp_cam_sensor,
                              esp_ipa, esp_lvgl_port, m5stack_tab5,
                              espressif__esp-dsp, lvgl, etc.
```

### 8.5.5  IDF v6 build env quirks (Windows + MSYS bash)

- `export.bat` refuses to run when `MSYSTEM` is set (which leaks into
  every `cmd /c` child spawned from MSYS bash). The `_build.bat` /
  `_flash.bat` wrappers `set "MSYSTEM="` before sourcing.
- PowerShell `export.ps1` is blocked by the default execution policy.
  Switched to the `.bat` flavor invoked via `cmd /c`.
- MSYS bash mangles paths in `cmd /c "..."` args (the `&&` chain gets
  reinterpreted). Use `cmd //c "wrapper.bat"` (double slash → MSYS
  doesn't re-quote) calling a `.bat` file that does all the work
  internally.
- `idf.py` output through a pipe (`| tail`) often returns exit 0 even
  when the underlying compile failed, because the pipe's exit is from
  `tail`. Always check `=== BUILD_OK ===` (printed by `_build.bat` on
  success) or grep for `error:` in the full log.

### 8.5.6  Open work — Phase 2 next steps

- **Robot36 RX sync lock — currently always-snap on 5 ms of 1200 Hz**.
  Works for self-decode but may misalign on the first VIS-leader tones
  (1900 Hz, not 1200 Hz). Adding a VIS-code detector would let the
  decoder know exactly which line is even vs odd and pick up the very
  first row of an incoming frame. Today's heuristic resets
  `row_in_pair = 0` on every snap, so the first decoded pair has a
  50/50 chance of swapping Cr/Cb.

- **ISP color tuning**. The `esp_ipa` default SC202CS JSON file emits
  CCM matrix elements outside the ESP32-P4 ISP hardware's ±4.0 clamp
  (`ISP_CCM Matrix[2][2] value 4.87 is out of range`). The hardware
  rejects the bad update every frame and the matrix stays at its
  boot-time identity. Picture is "fine but a little green" because of
  this. Either ship a custom IPA JSON with tighter clamps, or apply a
  fixed manual CCM via `V4L2_CID_USER_ESP_ISP_CCM` ioctl on top.

- **Robot36 audio scaling**. The encoder's max amplitude doesn't quite
  hit ±32767 (uses a sine-wave oscillator without normalization), so
  the speaker output has ~6 dB of headroom. Could tighten but not
  critical.

---

## 9. SC850SL flight bring-up + dual visible/thermal stream (2026-05/06)

Worked the Stamp-P4 flight target end to end: from a dead MIPI bus to a
live USB composite of **visible (SC850SL) | thermal (MI1602)** side by
side. All in `firmware/main/app_sc850sl.c`, `components/sc850sl/`, and
`main/mi1602_probe.c`. Default target is **SC850SL** (pinned in
`sdkconfig.defaults`, not the SC202CS_TAB5 that CLAUDE.md's intro still
names — that intro is stale; the flight target is what's active).

### 9.1 The MIPI/CSI breakthrough (why `isr_count` was stuck at 0)

Two independent root causes, both non-obvious:

1. **Sensor crop malforms the MIPI frame.** We were writing the output-
   window regs (0x3208/0x320a + 0x3210/0x3212) to crop 3840×2160 → 1920×
   1080, but left HTS/VTS (table_2's 4K15 timing) untouched. The sensor
   then emits a frame the CSI bridge never sees a clean frame-end on →
   `on_trans_finished` never fires. **Fix: don't crop. Capture native
   3840×2160 RAW10** (g_capture_buf = 16.6 MB RGB565-sized, holds the
   10.4 MB RAW10 frame). The "ISP max input = 1920×1080" limit in §5
   does NOT bind because we run the ISP in bypass (see below).

2. **IDF-9706: the ISP must be told the frame geometry even in bypass.**
   On ESP32-P4 the ISP sits between the CSI host and the GDMA. Even for
   pure RAW pass-through you MUST, *after* `esp_cam_ctlr_start()`:
   - `esp_isp_new_processor()` (placeholder RAW8→RGB565; don't enable)
   - poke `ISP.frame_cfg.hadr_num = ceil(h_res*out_bpp/32)-1`,
     `ISP.frame_cfg.vadr_num = v_res-1`, `ISP.cntl.isp_en = 0`
   (cribbed from esp_video `esp_video_isp_device.c`
   `esp_video_isp_start_by_csi`, the `state->bypass_isp` branch).
   Order matters: pokes go *after* `esp_cam_ctlr_start`, matching
   esp_video's `csi_video_start`. `#include "soc/isp_struct.h"`.

   Also: CSI config = `input=output=CAM_CTLR_COLOR_RAW10`, `bk_buffer_dis
   = true` (we always supply our own buffer via on_get_new_trans).

   **ISP memory-input (DWGDMA) is `ESP_ERR_NOT_SUPPORTED` in IDF v6.0.1**
   (`isp_core.c:81`), so "tile the RAW10 through the HW ISP offline" is
   not possible without raw-register hacking. The HW ISP only processes a
   live CSI stream at ≤1080p — which would need a 2×2-binning sensor mode
   we don't have a table for.

### 9.2 Software "ISP-lite" (we do color in SW, ISP is bypassed)

`downscale_raw10_bggr_to_rgb565()` does it all in one pass:
- **Bayer order is RGGB, not BGGR.** The classic "blue skin / blue mask
  shows yellow" R/B-swap. (even,even)=R, (odd,odd)=B.
- **Box-filter downscale** (average the whole source footprint per output
  pixel, per Bayer color) — nearest-neighbor subsampling of a Bayer image
  aliased into rainbow-confetti stripes on detailed rows. Box filter
  low-passes it away.
- **Gray-world AWB**: measure frame R/G/B means, set per-channel gains to
  equalize (green=ref), 1-frame lag + IIR smooth. Converges ~r1.44/b1.20
  in the cleanroom. No hand-tuned constants. Globals `g_wb_r/g_wb_b`.
- **sRGB gamma (1/2.2)** LUT — raw is linear and looks dark/wrong; gamma
  lifts mid-tones.
- Takes a `dst_stride` arg so it can render into a sub-rect of the wider
  composite buffer.

### 9.3 Exposure/gain control — STILL BLOCKED (datasheet §2.3.1)

**The SC850SL has NO internal AEC/AGC** ("需要通过后端平台实现"). Host must
drive it. BUT our writes to the exposure regs (0x3e00/01/02) produce
**zero brightness change** even with the group-hold latch (0x3800 = 0x00
pack / 0x10 end / 0x60 commit) AND AGC-enable (0x3e03[3:0]=0xb), proven by
a clean single-boot sweep: exp 256→2246 lines + analog gain 1×→4× → raw
mean pinned at ~65. So host AE can't work yet. `ae_step()` is `#if 0`'d.
The sensor runs at a fixed (table_2 default) exposure; brightness ~mean
60-68 raw, which gamma lifts to a usable image. **Pickup: recover the
real exposure/gain write sequence from M5Stack `libsns_sc850sl.so`** —
that one piece unblocks AE, anti-flicker, AND the 2×2-binning table for
the HW-ISP path.

### 9.4 I²C robustness (this carrier's I²C is genuinely flaky)

- `i2c_master_bus_reset()` at startup (clears a slave stuck mid-xfer).
- **Scan AFTER `sc850sl_init` (XSHUTDN high), not before** — this is why
  the bus scan used to show "ghost ACKs at every-N" (task #9, now closed):
  the sensor was floating during the scan. Powered, the scan shows only
  0x30.
- `write_table()`: per-write retry + bus reset, +5–20 ms settle after PLL
  regs (0x36e9/0x36f9), + full XSHUTDN power-cycle on table-level failure.
- **`sc850sl_write_reg`/`read_reg` now retry 4× with bus reset** — plus an
  app-level 5× retry around `sc850sl_stream_on`. Stream-on intermittently
  NACK'd its first write (0x302c) and would lose the camera for the boot.
- I²C @ 100 kHz (50 kHz was *less* reliable here).

### 9.5 MI1602 thermal — dual stream over USB

- **MODE strap → GND in hardware** (sampled at the module's power-on; the
  firmware GPIO10 drive at ~1.2 s is too late to matter — needs the HW
  pull-down, or a RESET line we don't have). MODE=GND = I²C+SPI mode.
- **I²C address 0x41** (ADDR strap high) — `CONFIG_MI1602_I2C_ADDR_HIGH=y`.
- I²C on **port 1** (SDA=12, SCL=15), fully separate controller from the
  SC850SL's port 0 (SDA=11, SCL=9). DataReady=GPIO8. SPI2: SCLK28/MISO29/
  MOSI30/CS31. The 5V rail to the module had a soldering fault (user
  fixed) — an unpowered module NACKs identically to a wrong address.
- **Chunked SPI read.** P4 GP-SPI caps one transaction at
  `SPI_MS_DATA_BITLEN` = 2^18-1 bits ≈ 32 KB; the 38.4 KB frame exceeds
  it → `txdata transfer > hardware max supported len`. `mi1602_spi.c`
  now reads in 16 KB chunks under continuous **manual CS** (MI48 just
  pauses on the SCLK gaps).
- `mi1602_aux_capture_rgb565()` in `mi1602_probe.c`: capture_single →
  auto-range → **twilight** colormap (17-anchor LUT, cyclic: light cold &
  hot, dark mid) → nearest-neighbor scale, **horizontally mirrored**
  (module orientation). Handle kept in `g_mi1602`.

### 9.6 USB composite stream

640×240 = `[ RGB 320×240 | thermal 320×240 ]`, ~1.5-2 fps (USB-Serial-
JTAG Full-Speed bound). Host `tools/usb_preview.py` rewritten with numpy
(per-pixel Python loop was the bottleneck) and reads w/h from the header
so it adapts. Magic prefix `\x00\x00\xff\xffFRM\x00`. The thermal capture
is a *blocking* single-shot per loop — moving it to a background MI48
streaming task would decouple it and raise the RGB fps.

### 9.7 Build/flash gotchas (this session)

- **Component `REQUIRES` can't be `if(CONFIG_*)`-gated.** Resolved in an
  early-expansion pass before Kconfig loads → the dep silently drops →
  "mi1602.h … mi1602 component is not in the requirements list". List
  `mi1602` unconditionally; the source self-guards with `#if
  CONFIG_MI1602_ENABLED`.
- **Tab5-only managed deps gated** in `main/idf_component.yml` via
  `rules: - if: "$CONFIG{CAMERA_TARGET_SC202CS_TAB5} == True"` (note the
  `$CONFIG{...}` curly syntax and `== True`, not bare `$CONFIG_...`).
  Cuts the SC850SL clean build from ~225 MB managed_components to ~1.
- **Bootloader subbuild lost `REV_LESS_V3`** once and demanded chip rev
  v3.1 (chip is v1.3). `rm -rf build/` (full clean) fixed it.
- **USB-Serial-JTAG port hops COM16↔COM21** on every reset; flashes
  intermittently fail right after a reset. Check
  `Get-CimInstance Win32_PnPEntity | ? {$_.Name -match 'USB Serial'}` and
  retry. Scripts currently target **COM16**.

### 9.8 Current firmware state & next pickups

State: SC850SL native-4K RAW10 → SW demosaic(RGGB)+box+grayworld-AWB+gamma
→ left pane; MI1602 → twilight thermal → right pane; 640×240 USB composite.
SSTV TX **disabled** (`#if 0`, live-preview mode). Host AE **disabled**.

Next, in rough priority:
1. **Reverse-engineer SC850SL exposure/gain seq from `libsns_sc850sl.so`**
   → unblocks AE, 50 Hz anti-flicker, and a 1080p binning mode.
2. Re-enable SSTV TX with the RGB+thermal pipeline.
3. HW ISP via 2×2 binning (real CCM/AWB/gamma) once the binning table is
   recovered.
4. MI48 background streaming task → higher RGB fps.

---

## 10. Repo reorganization + MI1602 vendored in-tree (2026-06)

The project root was a dumping ground (datasheets, `.deb` unpacks, rootfs
extraction, scratch scripts all mixed with source). Cleaned up and put under
version control.

**git + GitHub.** `SC850SL Dev/` is now a git repo, pushed to
<https://github.com/Oxidane-z/MOVE-IIIa-Imager> (public). The first commit is
the pre-cleanup working snapshot, so every later move is diffable/revertible.

**New top-level layout** (`SC850SL Dev/`):
- `firmware/` — the build (internals unchanged)
- `docs/` — R&D notes (BREAKTHROUGH_2LANE, FINAL_INVENTORY, M5STACK_CONFIG,
  TAB5_FINDINGS, TWO_LANE_DERIVATION)
- `reference/` — vendor driver + register-table *source copies* (the build
  uses the copies under `firmware/components/sc850sl/init_tables/`), extracted
  datasheet text, SSTV test-image source
- `datasheets/` — PDFs (gitignored)
- `_archive/` — bulky scratch: `*.deb`, `m5_research/`, `rootfs_files/`, the
  old `tools/rnd/` scripts, duplicate `_*_stamp.bat` (gitignored)
- root keeps only `CLAUDE.md`, `PROJECT_MEMORY.md`, `LICENSE`
- `firmware/`: `capture_serial.py` + `serial_tail.py` moved into
  `firmware/tools/` (alongside `usb_preview.py`); `_reset_capture.bat` updated
  to call `tools/capture_serial.py`.

**MI1602 driver vendored in-tree.** It used to be a sibling folder pulled via
`EXTRA_COMPONENT_DIRS`; it now lives at `firmware/components/mi1602/` (ESP-IDF
auto-discovers `components/`), so a fresh clone builds without the sibling.
`firmware/CMakeLists.txt` no longer sets `EXTRA_COMPONENT_DIRS`. The old
`MI1602 Dev/` sibling now holds only datasheets/reference + the standalone
`test/` app (its CMakeLists repointed to the new component path). `MI1602 Dev/`
itself is NOT under git — track it separately or as a submodule if ever needed.

> **git submodule (concept, for the record):** a submodule nests one git repo
> inside another *by reference* — the parent stores only the child's URL plus a
> pinned commit SHA, not the child's files. It fits when the child is a
> separately versioned library shared across multiple projects. We chose plain
> **vendoring** (copying the driver into this repo) instead, because the MI1602
> driver is single-purpose and edited in lockstep with this firmware; a
> submodule would add `git submodule update --init` ceremony to every clone for
> no real benefit here.

---

## 11. Flight-SW session: exposure unblocked, transmission fixed, OTA foundation (2026-06)

Big session. End state — what WORKS, what's BROKEN, where things are.

### What now works
- **SC850SL exposure/gain UNBLOCKED** (host AE converges to ~mean 110). Recipe
  recovered from the OpenIPC C reference (`reference/sensor_sc850sl_mipi.c`:
  `pCus_SetAEUSecs`/`pCus_SetAEGain`), NOT the `.so`. The old dead code's bugs:
  it poked `0x3800` as a "group-hold" (SC850SL has none there) and wrote
  `0x3e03=0x0b` "AGC enable" at boot (forced a mode that ignored manual writes)
  — *those two* were why exposure did nothing. New: gain = piecewise DCG+analog
  code (`0x3e08`∈{03,07,23,27,2f,3f}, `0x3e09` fine, `0x3e06`=0); exposure =
  `lines<<4` across 3e00/01/02; `ae_step` re-enabled (exposure-priority +
  anti-flicker snap to mains period, throttled 1/4 frames).
- **Watchdog reset loop fixed.** Per-frame AE I²C writes occasionally hit a
  stuck I2C0 bus; the normal write's retry+bus-reset storm blocked the stream
  task for seconds → task-WDT → reboot loop. Added `sc850sl_write_reg_fast()`
  (single-shot, no bus-reset) for the AE hot path.
- **USB preview smearing FIXED.** Root cause: the USB-Serial-JTAG console did
  **LF→CRLF translation**, expanding every `0x0A` byte in the binary frame →
  content-dependent length drift → desync (bright frames worse; all-black
  covered frame fine). Fix: `usb_serial_jtag_vfs_set_tx_line_endings(LF)`. Also
  hardened framing: single atomic `fwrite` (header+payload+trailer), `DEADBEEF`
  trailer (host drops+resyncs), stats in a 24-byte header. Viewer rewritten
  (`tools/usb_preview.py`, wire format v1) — overlays max/mean/bl/wb.
- **Black-level subtract + 3×3 CCM** added to software ISP-lite (good under
  adequate light).
- **Repo on GitHub (public):** <https://github.com/Oxidane-z/MOVE-IIIa-Imager>.
  Folders reorganized (`docs/ reference/ datasheets/ _archive/`); MI1602 driver
  **vendored in-tree** at `firmware/components/mi1602/` (a fresh clone now builds
  standalone). HEAD = `ef3b73c`. `capture_serial.py` got an open-retry.
- **OTA foundation (flight roadmap step 1):** dual-OTA partition table (`ota_0`/
  `ota_1` @2 MB + `otadata`), `CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y`,
  `app_update` dep, `ota_confirm_pending()` with a ~5 s burn-in in `app_run`'s
  idle loop. Built clean; RUNTIME not yet hardware-verified.

### Broken / pending
- **MI1602 thermal pane BLACK (task #27).** Probe + addr auto-detect work (ADDR
  strap floats 0x40/0x41 across power-ons; `mi1602_probe` now scans and uses
  whichever ACKs), BUT `mi1602_bootup` times out: `status=0x20` =
  MI48 BOOTING_UP never clears (used to clear in ~0 ms). Likely the **5 V solder
  joint** to the MI48 (digital core ACKs I²C but full boot stalls). Next: measure
  / reflow 5 V; optional boot-timeout bump 500→3000 ms; optional wire RESET_N
  (currently rst=−1).
- **SC850SL 2×2 binning / 1080p HW-ISP path: shelved** (task #22 closed). Reg
  table not in any source (all 4K). **Now a CONFIRMED public negative — see §12**:
  three independent SoC-vendor drivers (Axera, SigmaStar, Sophgo) are all
  4K-only, so a sub-4K table must be center-crop-windowed or hand-derived, not
  found. Decompiling the `.so`/`.bin` will not yield it.

### Flight architecture (NEW) — see `docs/FLIGHT_ARCHITECTURE.md`
Key principle: **AUTONOMOUS SSTV-beacon is the default** (capture + SSTV, RGB⇄LWIR
alternating, low duty) so the payload works even if RS485 fails. The OBC owns
the P4's **power switch**, so "OBC dead" = unpowered = not a software case; the
only fault handled is *powered + RS485 silent > T_link → AUTONOMOUS* (no dead-man
logic). Multi-core: HP0 = camera/ISP, HP1 = `rs422_link`(high-prio lifeline) +
`sstv_i2s`, LP = supervisor (heartbeat watchdog + reads board temp + low-power).
Roadmap: (1) OTA ✅; (2) rs422_link + dispatch; (3) autonomous beacon; (4)
multi-core; (5) bulk paths; (6) hardening.

### Confirmed from the latest schematic
- **RS485 = UART0**: TX=GPIO37, RX=GPIO38, **DE=GPIO39** (full-duplex, RX
  always-on, DE gates TX). Console on USB → U0 free. The `SSTV0_TX/RX±` nets are
  the *same* RS485 pair after 10 Ω series filters — NOT a second channel.
- **Board temp = Microchip AT30TS74**, I²C **addr 0x48**, on the **LP-I²C bus
  (I2C2): SDA=LP_GPIO7, SCL=LP_GPIO8** → the LP supervisor reads it directly.
- 3 I²C buses (I2C0/1/2); DSI display **unused** (MIPI pins = camera CSI).
- C6 WiFi addon = **SDIO** (ESP-Hosted/`esp_wifi_remote`), ground-only; exact
  `SDIO2_*` P4 GPIOs still TBD (Stamp-P4 pinmap) — confirm no clash.

### Next pickups (priority)
1. **At hardware:** `idf.py erase-flash` then flash (layout changed) → verify OTA
   boots `ota_0` + logs "OTA … confirmed valid" after ~5 s; re-confirm exposure/AE
   and check thermal (the 5 V fix).
2. Fix MI1602 5 V / bootup (task #27).
3. Flight roadmap **step 2**: `rs422_link` + `cmd_dispatch` skeleton (UART0,
   DE=GPIO39; `PING`/`GET_TLM`/`CAPTURE` first).
4. Resolve TBDs: C6 SDIO pins, RS485 transceiver part/baud, `T_beacon`/`T_link`.

---

## 12. RE data-source audit + Sophgo CV183x cross-reference (2026-06-05)

Re-audited every obtained file for remaining reverse-engineering value, then ran
a full web search (deep-research, 99 agents) for SC850SL data sources outside
M5Stack. Net: the RE well is dry for anything that ports to the P4; one new
*independent* source was found (still 4K-only); the binning/1080p table is now a
**confirmed public negative**.

### 12.1 Correction to §6 — the four `sc850sl_*.bin` are NOT sensor firmware
Hexdump shows all four carry the header `AX620E_ISP_V4.0.31`. They are **Axera
AX620E ISP parameter/tuning blobs**, not SC850SL sensor register data. The old
§6 labels ("AE/AGC tuning blob", "mode-switch fw", "HDR ratio LUTs") were
misidentifications — zero transfer value to the P4 (different ISP; we run
software ISP-lite). Same verdict for `rootfs_files/opt/etc/models/aiisp/*.axmodel`
(Axera NPU AI-ISP models). `libsns_sc850sl.so.0.0.0` is stripped aarch64; only
its `.data` register tables were ever useful and those are fully extracted —
no binning table inside (M5Stack only ever shipped 4K: all 4 tables + all `.ini`
= 3840×2160).

### 12.2 NEW independent source: Sophgo/CVITEK CV183x driver (4K-only)
Third fully-independent SC850SL driver (distinct from M5Stack/Axera and
OpenIPC/SigmaStar):
- Repo `sipeed/LicheeRV-Nano-Build`, path
  `middleware/v2/component/isp/sensor/cv183x/sms_sc850sl/` (5 files).
- **Plain-C hex literals** (not a binary blob) — `sc850sl_write_register(ViPipe,
  0xRRRR, 0xVV)`, copy straight into bare-metal I²C. Chip-ID defines match our
  silicon (0x3107/0x3108 = 0x9d1e).
- Vendored locally at `reference/sophgo_cv183x_sc850sl/` (+ `diff_vs_m5stack.py`).
- TWO modes only, both 4K: `sc850sl_linear_2160P30_init` (4-lane **RAW12** 4K30,
  147 writes) + `sc850sl_wdr_2160P30_2to1_init` (HDR-DOL 4K30, 200 writes). No
  1080p/binning. Exists ONLY under `cv183x` (not cv181x/cv182x, the SG2002's
  family) → it's a reference, not a drop-in for that board.

### 12.3 Three-way register diff (`reference/.../diff_vs_m5stack.py`)
Sophgo-linear (4L RAW12) vs M5Stack table_0 (4L RAW10 4K30) vs table_2 (2L RAW10
4K15, our flight mode), last-write-wins:
- **96 registers identical across all three vendors** → SENSOR-CORE INVARIANTS
  (analog/pixel/calib: 0x33xx/0x36xx/0x39xx/0x59xx, plus HTS 0x320c/d = 0x044c).
  Keep untouched when deriving a new mode; also independently corroborates our
  extracted M5Stack tables.
- Volatile per-mode block (all three differ): PLL/MIPI 0x36e9/ea/f9/fa/fb/fc/fd +
  lane(0x3018)/bitdepth(0x3031). **Not hand-derivable** — this is the binning
  blind spot.
- **Sophgo's only genuinely-new data**: a baked-in exposure/gain default block
  (~24 regs: 0x3e04-09, 0x3e51/52/58/59, 0x3e66/67/6a/6b, 0x3e71/72, 0x3e82/83)
  the M5Stack tables leave to the runtime AE loop — a useful THIRD reference to
  cross-check our OpenIPC-derived gain layout (0x3e08 coarse / 0x3e09 fine /
  0x3e06 digital) and for future HDR dual-exposure work.
- **None of the three** writes any output-window (0x3208/0a/10/12) or binning
  register — all run full-res on power-on defaults. A crop/bin mode must ADD
  registers none of them documents.

### 12.4 CONFIRMED public negative — no sub-4K / binning / 1080p table exists
Verified across three independent SoC-vendor drivers + official + community:
- Axera/M5Stack (4 tables), SigmaStar/OpenIPC (`sensor_sc850sl_mipi.c`, 3 modes),
  Sophgo/CV183x (2 modes) — **all 3840×2160 only**.
- Sophgo tree has exactly one SC850SL variant (cv183x); no cv181x/182x version.
- Sipeed MaixCAM2 (Axera AX630) lists SC850SL but at 4K/60fps/4-lane — same Axera
  family as M5Stack, expected to overlap; not chased deeper.
- Espressif `esp_cam_sensor` has **NO** SC850SL driver (only sc02xx/sc03xx/sc101/
  sc2336) — confirms our hand-rolled flight driver fills a real gap.
- SmartSens datasheet V1.10 names "2×2 binning" as a feature but ships no binning
  register sequence; the V4.0 web flyer has zero register data.
- NOT exhaustively searched (so NOT confirmed negatives): `scpcom/sophgo-middleware`
  submodules beyond the mirrored cv183x; MaixPy/MaixCDK SDK source tree; Rockchip
  rk_aiq / Ingenic T31-T40 / Hisilicon Hi35xx trees (architecturally unlikely —
  those SoCs are ≤5MP-class; the 8MP/4K families are exactly the 3 already found).

### 12.5 Consequence for the 1080p HW-ISP path (task #22)
A sub-4K sensor mode will NOT be found ready-made; obtain it one of three ways:
1. **1080p center-crop window** — set 0x3208/0a output size + 0x3210/12 start +
   recompute VTS; uses only documented registers, works today, costs FOV.
2. **Hand-derived 2×2 binning** — now lower-risk (keep the 96 invariants, change
   only lane/bitdepth/window/timing), BUT the MIPI-PLL retune for the halved
   data rate is still a blind spot — same class of thing only a SmartSens FAE
   gave us reliably for the original 2-lane PLL.
3. Ask SmartSens FAE for the binning init table.
Stop decompiling the `.so`/`.bin` — they hold no binning data.

### 12.6 AE recipe cross-checked + daytime highlight protection added (2026-06-05)
Used the new CVITEK cv183x driver to verify our host-AE register recipe
(`app_sc850sl.c`), then added three improvements.

**Cross-check verdict — our recipe is correct.** Exposure field
`{0x3e00[3:0],0x3e01,0x3e02[7:4]}` = lines; all six analog-gain breakpoints +
`0x3e08` codes (`0x03/07/23/27/2f/3f` at gain x1024 = 1024/2048/3200/6400/
12800/25600); the `0x3e09` fine mantissa; `EXP_MAX = VTS-4`; and `GAIN_MAX` =
49.6x all match CVITEK `AgainInfo[]` / `cmos_gains_update` register-for-register.
CVITEK's baked-in init `0x3e08=0x03 / 0x3e09=0x40` == our `SC850SL_GAIN_UNITY`
(1.0x) AE start. (The `mean=90` setpoint is our control choice, not in any driver.)

**Three changes in `app_sc850sl.c`:**
1. **Daytime highlight protection** (the priority — large-DR daytime scenes were
   the concern). The raw-stats scan counts near-saturated pixels
   (>= `AE_SAT_LEVEL`=250 in the 8-bit sample) into `raw_sat_ppm`. `ae_step()`
   gained a highlight guard: if `> AE_SAT_BUDGET_PPM` (8000 = 0.8%) of pixels
   clip, pull brightness DOWN (gain first, then exposure, -20%/step) regardless
   of the mean. AE mean target lowered 110->90 for headroom. New `sat=NNNppm`
   field in the `usb_stream[...]` log to tune by.
2. **DPC vs gain** (night): `0x363c` 0x07->0x04 below 2.0x analog gain (mirrors
   CVITEK `cmos_gains_update`), change-detected to spare the I2C bus.
3. **Digital-gain relay** (night, UNTESTED): past 49.6x analog, rail analog and
   step `0x3e06` digital 2x/4x/8x (CVITEK `cmos_dgain_calc_table`) with
   `0x3e07=0x80`; `SC850SL_GAIN_MAX` raised to ~397x. Coarse 6 dB steps, so
   brightness stepping at the very top; daytime never reaches it.

**Pending:** on-hardware verification (Olimex + FPC). Tuning constants
(250 / 8000 ppm / target 90) are conservative starting points; the highlight
guard may hunt slightly around the clip boundary on extreme-DR scenes.

---

## 13. Ground WiFi + web station via C6 addon (2026-06-05)

Ground-test convenience layer so the flight board can be driven + updated over
WiFi without USB. **Full operator guide: `docs/GROUND_WIFI.md`.** Everything is
gated behind `CONFIG_GROUND_WIFI_ENABLE` (default OFF) — the flight build is
unaffected (verified byte-identical when off).

**Architecture.** P4 has no radio; the M5 Stamp-AddOn C6 is a WiFi co-processor
over SDIO via `esp_wifi_remote` + `esp_hosted`. P4 host Slot-1 flexible GPIOs,
addon-wired to **G42-48** (CLK43/CMD44/D0-3=45-48/RST42). The addon's
pre-flashed ESP-Hosted slave fw was compatible first try — **no C6 reflash**.

**Status.** _(Web UI later reworked to a 6-tab layout + poll-based preview; the
MJPEG `/stream` was removed — see §13.1. The `/stream` / on-demand-ISP notes
below are the original P3 design, kept for history.)_
- **HW-verified:** P1 link+STA (`got ip 10.200.0.121`) and P2 telemetry+control
  (`/`, `/api/tlm`, `/api/cmd`). Live timing telemetry confirmed software-ISP
  ~0.66 s, USB ~0.36 s per frame.
- **HW-VERIFIED (2026-06-06)** — the rest
  of the ground station: P3 JPEG preview (`/snapshot.jpg`, MJPEG `/stream`),
  **OTA over WiFi** (`/api/ota`), mDNS+DHCP hostname (**move-imager.local**),
  `/api/log` log ring; on-demand ISP (renders only when a `/stream` viewer is up
  or USB push on) + USB binary push default OFF; system-health telemetry
  (heap/psram/reset) + **reboot** + **NVS-persisted settings** (`save=1`); live
  colour tuning (AWB/WB/black-level/CCM) + focus aid; **HD still** `/capture.jpg`
  (1280×720; `ground_render_hd` reuses the SW ISP); **web-triggered SSTV**
  (`sstv=1`; sstv_tx_task now trigger-driven — task #23 closed). Full
  endpoint/command reference: `docs/GROUND_WIFI.md`.

**The bug we hit + fixed (P1):** starting WiFi before `app_run()` collided with
the SC850SL stream-on — the WiFi-connect transient wedged the sensor's port-0
I²C bus mid-stream-on (port-1 MI1602 survived). Fix: `ground_wifi_start()` moved
to the `idle:` label in `app_sc850sl.c`, after the camera is streaming.

**Files:** `main/ground_wifi.c` (STA + mDNS), `main/ground_http.c` (server +
all endpoints + JPEG + OTA + log ring), `main/ground_station.h` (app<->server
snapshot/mailbox interface), `main/ground_index.html` (embedded page),
`sdkconfig.defaults.ground` (committed non-secret overlay) +
`sdkconfig.ground.secret` (gitignored creds), `_build_ground.bat`,
`tools/raw_grab.py` (raw byte capture + frame/log parser — capture_serial.py
UTF-8-mangles the binary stream). Deps: `esp_wifi_remote`/`esp_hosted`/`mdns`
gated in `idf_component.yml`; `esp_wifi`/`esp_http_server`/`esp_driver_jpeg`/
`nvs_flash` are unconditional CMake REQUIRES (IDF built-ins can't be
Kconfig-gated; flight dead-strips them).

**RESUME (board must be connected):**
1. One **USB flash** of the ground build (`_build_ground.bat` → `_flash.bat`)
   to put the OTA-capable image on the board. _(Done 2026-06-06 — this whole
   RESUME list is complete; the board runs the full ground image. Kept for
   history.)_ `_erase_flash.bat` first only if the partition layout changed.
2. Then verify the untested pile: open `http://move-imager.local/` → live
   preview (`/stream`), telemetry, a `/api/ota` upload round-trip, `/api/log`,
   and confirm idle ISP≈0 (on-demand) when no viewer.
3. Remaining P4: full-res HD download, SSTV trigger from web (task #23), and
   re-test SDIO at 40 MHz (C6 reports the PCB supports it; we run 20 MHz).

Commits: `82b2102` P2 · `1ba5120` P3+OTA+mDNS+on-demand · `e3de44f` /api/log +
guide · `1c8de7a` health+reboot+NVS · `a2e4d0e` tuning+focus · `5661117` HD
capture · `e101b0f` SSTV trigger · `c7f4bf5` guide refresh.

**HW verification (2026-06-06): PASS.** Flashed + exercised the whole stack at
`move-imager.local`: boot, WiFi + mDNS, camera stream, `/api/tlm`, `/api/log`,
`/stream` (MJPEG ~1.3 fps, on-demand ISP), `/capture.jpg` (HD 1280×720, repeatable),
and a full **OTA round-trip** (upload → reboot into the inactive slot → back
online, up≈22 s). Two HW-only bugs found + fixed during bring-up:
1. esp_hosted SDIO mempool OOM reboot-loop → `CONFIG_ESP_HOSTED_MEMPOOL_PREFER_SPIRAM=y`
   (the grown app squeezed internal RAM) — commit `848dbd5`.
2. `/capture.jpg` per-request `jpeg_alloc_encoder_mem` hang (pool didn't free →
   2nd request blocked → wedged httpd) → reuse persistent JPEG buffers — `f8bf22d`.

**NEXT (still held by user):** RS485 OBC link + command dispatch (task #28
step 2) — new `rs422` module on UART0 (TX37/RX38/DE39),
`SYNC|LEN|TYPE|SEQ|payload|CRC16` parser, `PING/GET_TLM/CAPTURE` dispatch.

### 13.1 Web UI rework — tabs + poll-based per-view preview (2026-06-06)

Reworked the single page into **six tabs** (RGB / LWIR / image-tuning /
auto-exposure / OTA / console) over a persistent telemetry dashboard + status bar.

**Why MJPEG `/stream` is gone.** esp_http_server runs handlers in a **single
worker task**; a `/stream` MJPEG response loops forever, so once any browser
opened the preview it starved `/api/tlm` + `/api/log` — that was the "console
shows nothing" bug. Fix: drop `/stream`, make previews **poll-based**
`/snapshot.jpg?view=rgb|lwir` — each request renders one view in the worker,
sends, returns, freeing the worker between frames. Verified: under a continuous
snapshot flood, `/api/tlm` + `/api/log` still answer in ≤2.5 s (bounded by one
render), never hang. The console tab auto-tails `/api/log` every 1.5 s.

**Per-view + full-res.** `/snapshot.jpg?view=rgb` (default 960×540) and
`?view=lwir` (480×360) share the render path; LWIR via new `ground_render_lwir`
→ `mi1602_aux_capture_rgb565`. `/capture.jpg?res=hd|fhd` downloads 1280×720 or
1920×1080; **FHD is the ceiling** — native 4K RGB565 (16.6 MB) won't fit beside
the capture buffer. Shared JPEG input buffer bumped to FHD (~4 MB; PSRAM has
room — ~10 MB free after). Render moved entirely off the camera task into the
httpd worker, so `want_preview` stays off (nothing increments `s_stream_clients`)
and the camera task's `isp_us`≈0.

**LWIR reality:** MI1602 I²C+SPI readout is live (`device found at 0x41`,
`bootup OK status=0x00`), but the MI48 still isn't producing a calibrated frame
(task #27) — the LWIR tab shows live *uncalibrated* FPA noise, labelled as such,
not "offline".

**HW-verified (2026-06-06):** tabs, poll-based RGB preview, FHD/HD download,
auto-tailing console, and console responsiveness under preview load — all pass
at move-imager.local. Touches `ground_http.c` (handlers), `app_sc850sl.c`
(`ground_render_lwir`), `ground_station.h`, `ground_index.html`. Field note: at
low RSSI (≲ -80 dBm) the C6 STA can flap connect/reconnect, and a marginal USB
rail can brown out under camera+radio load — keep the board near the AP on solid
power.

**Temps + LWIR switch (2026-06-06, follow-on).** Added a board-temperature
readout (AT30TS74 @0x48 on **LP-I2C, SCL=GPIO7 / SDA=GPIO1**) driven from the
**HP core** via the standard `i2c_master` driver — LP_I2C master from the HP core
works on P4 (`i2c_port = LP_I2C_NUM_0`, `lp_source_clk = LP_I2C_SCLK_DEFAULT`).
This is the ground stand-in for the flight LP-core path (task #28): same sensor +
pins, HP-driven for now. `board_temp_task` reads it every 2 s; fail-soft (-300
sentinel → JSON null → "n/a"). Also surfaced the MI1602 **SenXor die temp**
(latched from the thermal frame header in `mi1602_aux_capture_rgb565`; gated on
`hdr.crc_ok`, so it shows n/a while the MI48 is unhealthy — task #27). SC850SL has
no readable die-temp register → omitted (checked the reference driver). Added an
**LWIR streaming on/off** switch (`lwir=0|1`, gates `ground_render_lwir`) mirroring
the SC850SL one. Deployed + verified over OTA: `board_temp` ~37 C live, LWIR toggle
503/200. CSI/power note: the armed CSI controller is P4-side only; `sc850sl_sleep`
powers down the sensor itself (the hot part), so pausing genuinely cuts sensor
power/heat — the live board temp is the feedback.

---

- ESP-IDF v6.0.1 release: <https://github.com/espressif/esp-idf/releases/tag/v6.0.1>
- ESP-IDF v6.0 breaking changes: <https://github.com/espressif/esp-idf/issues/17052>
- ESP32-P4 ISP API (v6.0.1): <https://docs.espressif.com/projects/esp-idf/en/v6.0.1/esp32p4/api-reference/peripherals/isp.html>
- ESP-IDF camera sensor (auto-detect): <https://components.espressif.com/components/espressif/esp_cam_sensor>
- ESP-IDF MIPI CSI example (target for Phase 2): `examples/peripherals/camera/mipi_isp_dsi` inside `C:\esp\esp-idf-v6.0.1\`
- Olimex ESP32-P4-DevKit OSHW: <https://www.olimex.com/Products/IoT/ESP32-P4/ESP32-P4-DevKit/open-source-hardware>
- Waveshare ESP32-P4-WIFI6 (WAVE-31647): <https://www.waveshare.com/wiki/ESP32-P4-WIFI6>
- ESP32-P4 CSI host: same docs site, `cam_csi.html`
- OpenIPC SigmaStar sensors: <https://github.com/OpenIPC/sensors>
- M5Stack StackFlow OBC bus: <https://github.com/m5stack/StackFlow>
- M5Stack LLM-camera demo: <https://docs.m5stack.com/en/stackflow/llm630_compute_kit/stackflow_camera_demo>
- M5Stamp-P4 board: <https://docs.m5stack.com/en/core/Stamp-P4>
- CubeSat Space Protocol: <https://github.com/libcsp/libcsp>
- Robot36 SSTV spec: <https://www.classicsstv.com/specs.php>
