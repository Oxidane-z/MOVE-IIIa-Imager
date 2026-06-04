# MOVE-IIIa Imager

Firmware for the **MOVE-IIIa** CubeSat imaging payload, running on an
**ESP32-P4** (M5Stamp-P4 + custom carrier). It drives two cameras over MIPI
and SPI, does the image processing on-chip, and streams a live preview over
USB.

## Overview

| | |
|---|---|
| **Visible camera** | SmartSens **SC850SL** (4K), MIPI CSI-2 2-lane RAW10 — the flight sensor. Hand-rolled driver in `firmware/components/sc850sl/`. |
| **Thermal camera** | Meridian Innovation **MI1602** (MI48 core), 160x120 radiometric (deci-Kelvin), I2C control + SPI frame data. Driver in `firmware/components/mi1602/`. |
| **Live preview** | Visible + thermal scaled into one 640x240 RGB565 composite, streamed binary-clean over USB-Serial-JTAG to a host viewer (`firmware/tools/usb_preview.py`). |
| **SSTV** | In-tree Robot36 encoder (`firmware/components/sstv_robot36/`) for an audio downlink path. |

Two build targets select the camera via `CONFIG_CAMERA_TARGET_*`:

- **SC850SL** — the flight target (visible 4K).
- **SC202CS on M5Stack Tab5** — a bench/demo target using upstream
  `esp_video` + `esp_cam_sensor`, with a Robot36 SSTV TX/RX demo.

### Image pipeline (SC850SL, software "ISP-lite")

`RAW10 -> black-level subtract -> box-filter demosaic (RGGB) -> gray-world
AWB -> 3x3 CCM -> sRGB gamma -> RGB565`. The ESP32-P4 hardware ISP is bypassed
on the native-4K path (see `PROJECT_MEMORY.md` section 9 for why).

## Repository layout

```
firmware/            ESP-IDF v6.0 project (the build)
  main/              app entry + per-target app_*.c + USB stream
  components/
    sc850sl/         SC850SL visible-camera driver + register init tables
    mi1602/          MI1602 thermal-camera driver  (see Credits)
    sstv_robot36/    Robot36 SSTV encoder
    isp_pipeline/    rs422_protocol/   (stubs for later phases)
  tools/             host scripts: usb_preview.py, capture_serial.py, ...
docs/                R&D notes (bring-up, derivations, findings)
reference/           vendor driver + register-table source copies, datasheet text
datasheets/          PDFs (gitignored)
CLAUDE.md            per-session guide
PROJECT_MEMORY.md    full project history / context
```

## Build & flash

Requires ESP-IDF **v6.0.1** with the RISC-V toolchain.

```sh
cd firmware
idf.py set-target esp32p4         # one-time
idf.py menuconfig                 # pick CONFIG_CAMERA_TARGET_* if needed
idf.py build
idf.py -p <PORT> flash monitor
```

See `firmware/HOW_TO_BUILD.md` for details and the Windows wrapper scripts.

## Status

- **Working:** SC850SL 2-lane 4K RAW10 capture; software ISP-lite; MI1602
  thermal capture (twilight false-colour); dual visible+thermal USB preview
  with binary-clean transmission.
- **Blocked:** SC850SL exposure/gain control — the sensor has no internal AEC
  and the vendor register write-sequence has not yet been recovered
  (`PROJECT_MEMORY.md` section 9.3).
- **Planned:** SSTV downlink re-enable, RS422 OBC link, thermal/visible
  registration, hardware-ISP via 2x2 binning.

## Credits & attribution

- **MI1602 driver** (`firmware/components/mi1602/`) is a **port of Meridian
  Innovation's official `pysenxor` 1.6.7 reference SDK** — it keeps the same
  register usage, bootup flow, frame layout and CRC convention. Meridian's
  C++ `libsenxor_samples` was used as a secondary reference for the SPI
  transfer. Refer to those upstream projects for their original terms.
- **SC850SL register init tables** were derived from M5Stack's production
  camera firmware and the OpenIPC SigmaStar reference driver
  (`sensor_sc850sl_mipi.c`, kept under `reference/`).
- **Robot36 SSTV** follows the published mode specification.

## License

MIT — see [LICENSE](LICENSE). The third-party material credited above
(pysenxor, vendor register tables) remains subject to its own upstream
licensing.
