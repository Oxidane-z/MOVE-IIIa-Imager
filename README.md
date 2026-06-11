# MOVE-IIIa Imager

Firmware for the **MOVE-IIIa** CubeSat imaging payload, running on an
**ESP32-P4** (M5Stamp-P4 + custom carrier). It drives a visible + a thermal
camera over MIPI-CSI and SPI, does the image processing on-chip (no external
ISP), and — for ground testing — exposes a full **web control station** over
WiFi with live preview, tuning, OTA update, and a console.

## Overview

| | |
|---|---|
| **Visible camera** | SmartSens **SC850SL** (4K), MIPI CSI-2 2-lane RAW10 — the flight sensor. Hand-rolled driver in `firmware/components/sc850sl/`, host-driven exposure/gain + closed-loop auto-exposure (the sensor has no internal AEC). |
| **Thermal camera** | Meridian Innovation **MI1602** (MI48 core), 160×120 radiometric, I²C control + SPI frame data. Driver in `firmware/components/mi1602/`. |
| **Image pipeline** | Software "ISP-lite" on the P4: `RAW10 → black-level → box-filter demosaic (RGGB) → gray-world AWB → 3×3 CCM → sRGB gamma → RGB565`. ~0.66 s per 4K→preview downscale; on-demand only. |
| **Ground station** | WiFi (ESP32-C6 add-on over SDIO) → a tabbed web UI: per-camera preview, image tuning, auto-exposure, **OTA firmware update**, live console, system-health telemetry incl. board temperature. **Ground test only** — the flight build pulls none of it. |
| **SSTV** | In-tree Robot36 encoder (`firmware/components/sstv_robot36/`) for an audio downlink path; web-triggered. |

### Build targets

Two camera targets share the firmware, selected via `CONFIG_CAMERA_TARGET_*`:

- **SC850SL** — the flight target (visible 4K), active in `sdkconfig.defaults`.
- **SC202CS on M5Stack Tab5** — a bench/demo target using upstream `esp_video`
  + `esp_cam_sensor` + the Tab5 BSP, with an on-screen preview and a Robot36
  SSTV TX **and** mic RX demo.

## Web ground station

Gated behind `CONFIG_GROUND_WIFI_ENABLE` (off in flight). Built with the
`_build_ground.bat` wrapper (a layered `sdkconfig.defaults` + ground overlay +
gitignored WiFi secret). After one USB flash of an OTA-capable image, **all
further updates go over WiFi** (`POST /api/ota`). Reachable at
`http://move-imager.local/`. Full operator guide: `docs/GROUND_WIFI.md`.

Web UI tabs (over a persistent telemetry dashboard + status bar):

- **RGB camera** — polled hi-res preview + a camera-streaming on/off switch
  (off sleeps the SC850SL to save power and stop it heating) + full-res HD/FHD
  still download + SSTV trigger.
- **LWIR camera** — polled thermal preview + its own streaming switch.
- **Image tuning** — AWB, white balance, black level, 3×3 CCM.
- **Auto exposure** — AE target/enable + manual exp/gain override.
- **OTA update** — upload a `.bin`, auto-reboot, bootloader auto-rollback.
- **Console** — live tail of the device log (no USB cable needed).

Telemetry includes board temperature (Microchip **AT30TS74** on LP-I²C) and the
MI1602 SenXor die temperature.

## Repository layout

```
firmware/            ESP-IDF v6.0.1 project (the build)
  main/              app entry + per-target app_*.c, ground_*.c (WiFi/web/OTA),
                     mi1602_probe.c, embedded ground_index.html
  components/
    sc850sl/         SC850SL visible-camera driver + register init tables
    mi1602/          MI1602 thermal-camera driver  (see Credits)
    sstv_robot36/    Robot36 SSTV encoder
  tools/             host scripts: capture_serial.py, raw_grab.py, usb_preview.py
  _build*.bat / _flash.bat / _reset_capture.bat   Windows build/flash wrappers
docs/                GROUND_WIFI.md, FLIGHT_ARCHITECTURE.md, R&D notes
reference/           vendor driver + register-table source copies, datasheet text
CLAUDE.md            per-session guide
PROJECT_MEMORY.md    full project history / context
```

## Build & flash

Requires ESP-IDF **v6.0.1** with the RISC-V toolchain.

```sh
cd firmware
idf.py set-target esp32p4          # one-time
idf.py menuconfig                  # pick CONFIG_CAMERA_TARGET_* if needed
idf.py build
idf.py -p <PORT> flash monitor
```

On Windows this repo uses wrapper `.bat` files (an MSYS/`export.bat` quirk):
`_build.bat` (flight), `_build_ground.bat` (ground station), `_flash.bat`,
`_reset_capture.bat`. See `firmware/HOW_TO_BUILD.md` and `docs/GROUND_WIFI.md`.

## Status

- **Working:** SC850SL 2-lane 4K RAW10 capture; software ISP-lite; closed-loop
  auto-exposure; MI1602 thermal capture (twilight false-colour); the full WiFi
  web ground station (preview, tuning, telemetry + board temp, OTA round-trip,
  console) — all hardware-verified.
- **In progress:** MI1602 frame integrity — the MI48 boots and streams real
  thermal frames, but the chunked SPI frame read mismatches the per-frame CRC
  and leaves background speckle; being tuned (SPI clock / read path / on-chip
  filters). Tracked as task #27.
- **Planned (flight):** RS485 OBC link + command dispatch; LP-core safety
  supervisor (reads board temp on LP-I²C, heartbeat watchdog); thermal/visible
  registration. See `docs/FLIGHT_ARCHITECTURE.md`.

## Credits & attribution

- **MI1602 driver** (`firmware/components/mi1602/`) is a **port of Meridian
  Innovation's official `pysenxor` reference SDK** — it keeps the same register
  usage, bootup flow, frame layout and CRC convention. Meridian's C++
  `libsenxor_samples` was a secondary reference for the SPI transfer. Refer to
  those upstream projects for their original terms.
- **SC850SL register init tables** were derived from M5Stack's production camera
  firmware and the OpenIPC SigmaStar reference driver (`sensor_sc850sl_mipi.c`,
  kept under `reference/`).
- **Robot36 SSTV** follows the published mode specification.

## License

MIT — see [LICENSE](LICENSE). The third-party material credited above
(pysenxor, vendor register tables) remains subject to its own upstream
licensing.
