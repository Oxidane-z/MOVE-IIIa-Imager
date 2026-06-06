# Ground-test WiFi + web station (C6 addon)

Ground-only convenience layer: WiFi + a browser control page + OTA, so the
flight board can be driven and updated **without a USB cable**. Everything is
gated behind `CONFIG_GROUND_WIFI_ENABLE` — the flight build pulls none of it.

> Status (2026-06-06): **fully hardware-verified.** Tabbed web UI (RGB / LWIR /
> tuning / exposure / OTA / console), poll-based per-view preview, full-res
> HD+FHD download, an auto-tailing console, telemetry/health, control, and a
> full OTA round-trip all confirmed at `move-imager.local`. The earlier MJPEG
> `/stream` was **removed**: it looped forever inside esp_http_server's single
> worker, so an open preview starved `/api/tlm` + `/api/log` (the "blank
> console" bug). Previews are now poll-based `/snapshot.jpg`. Three HW-only bugs
> were fixed during bring-up: an esp_hosted SDIO mempool OOM reboot-loop
> (`CONFIG_ESP_HOSTED_MEMPOOL_PREFER_SPIRAM=y`), a `/capture.jpg`
> per-request-alloc hang (reuse persistent JPEG buffers), and the single-worker
> stream wedge above.

## Hardware

- **M5Stack Stamp-AddOn C6 for P4** seated on the Stamp-P4's 20-pin SDIO header.
- The P4 has no radio; the C6 is a WiFi co-processor over SDIO via
  `esp_wifi_remote` (proxies the `esp_wifi` API) + `esp_hosted` (transport).
- SDIO host pins (P4 Slot-1 flexible GPIOs, wired by the addon):
  **CLK 43, CMD 44, D0 45, D1 46, D2 47, D3 48, reset 42** — set in
  `sdkconfig.defaults.ground`. Clean of all other peripherals.
- 2.4 GHz only (C6 is WiFi 6, no 5 GHz).
- The addon's pre-flashed ESP-Hosted slave firmware was compatible out of the
  box on first bring-up — no C6 reflash was needed.

## Credentials (never committed)

WiFi SSID/password live in `firmware/sdkconfig.ground.secret` (gitignored):

```
CONFIG_GROUND_WIFI_SSID="your-ssid"
CONFIG_GROUND_WIFI_PASSWORD="your-pass"
```

The non-secret ground overrides (enable flag, C6 slave target, SDIO pins,
hostname) are in the committed `firmware/sdkconfig.defaults.ground`.

## Build & flash

Ground build uses a 3-file `SDKCONFIG_DEFAULTS` chain
(`sdkconfig.defaults` + `sdkconfig.defaults.ground` + `sdkconfig.ground.secret`)
via its own wrapper. The flight build (`_build.bat`) is untouched.

```sh
# from firmware/ in MSYS bash:
cmd //c "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware\_build_ground.bat"   # ground build
cmd //c "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware\_flash.bat"          # USB flash (COM16)
```

**First flash must be over USB** — the board needs an OTA-capable image before
OTA works. After that, update over WiFi (see OTA below). If the partition
layout changed, `cmd //c _erase_flash.bat` once before flashing.

## Connecting

- mDNS: **http://move-imager.local/** (hostname = `CONFIG_GROUND_WIFI_HOSTNAME`).
  Works from macOS/Linux/Windows that support mDNS; also shows as
  `move-imager` in the router's client list (DHCP hostname).
- Or the raw IP — printed in the boot log as `got ip: x.x.x.x` and visible in
  the router.

## Web UI / endpoints

| Endpoint                       | Method | Purpose                                                       |
|--------------------------------|--------|---------------------------------------------------------------|
| `/`                            | GET    | tabbed control page (embedded `ground_index.html`)            |
| `/api/tlm`                     | GET    | telemetry JSON (exp/gain/mean/sat/wb/timings/heap/ip…)        |
| `/api/cmd`                     | POST   | control — query params below                                  |
| `/snapshot.jpg?view=rgb\|lwir` | GET    | one on-demand JPEG of a view (default RGB 960×540, LWIR 480×360); the preview tabs poll this |
| `/capture.jpg?res=hd\|fhd`     | GET    | full-res still download — `hd` 1280×720, `fhd` 1920×1080      |
| `/api/ota`                     | POST   | firmware update — raw `.bin` as the body                      |
| `/api/log`                     | GET    | recent device log (last ~4 KB; the console tab auto-tails it) |

`/api/cmd` params (omit any to leave unchanged):
- exposure/AE: `ae_target=<mean>`, `ae_en=0|1`, `exp=<lines>`, `gain=<x1024>`
- colour: `awb=0|1`, `wb_r=<f>`, `wb_b=<f>`, `bl=<n>`, `ccm0..ccm8=<f>` (3×3)
- ops: `usb=0|1` (USB push), `save=1` (persist to NVS), `reboot=1`, `sstv=1`

## OTA over WiFi

1. Build the new image: `_build_ground.bat` → `firmware/build/cubesat_imager.bin`.
2. On the page, *Firmware update* → pick that `.bin` → **Upload & reboot**
   (or `curl --data-binary @build/cubesat_imager.bin http://move-imager.local/api/ota`).
3. The board writes it to the **inactive** OTA slot, marks it bootable, reboots.
4. `app_run()` confirms the new image valid only after a ~5 s burn-in; if it
   crashes first, the bootloader **auto-rolls back** to the previous slot
   (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`, dual `ota_0`/`ota_1` in
   `partitions.csv`). See FLIGHT_ARCHITECTURE.md §12.

## Compute / preview behaviour

- Previews are **poll-based**, not a persistent stream. Each `/snapshot.jpg`
  renders one view on demand (the ~0.66 s 4K→RGB box-filter downscale, or an
  MI1602 thermal grab), JPEG-encodes, sends, and returns — so the single
  esp_http_server worker is free between frames and `/api/tlm` + `/api/log` stay
  responsive while a preview tab polls. The browser chains the next poll on the
  `<img>` onload, self-pacing to the render rate (~1 fps). Verified: under a
  continuous snapshot flood, telemetry + log still answer in ≤2.5 s, never hang.
- Only the **active tab's** heavy poller runs (RGB *or* LWIR snapshot, *or* the
  console log — never all at once), so an idle board (no preview tab open) just
  does capture + raw stats + AE + telemetry, cheap.
- Render now happens in the httpd worker, not the camera task, so the camera
  task's `isp_us` reads ~0 in telemetry (expected). The legacy USB binary push
  (`tools/usb_preview.py`) still defaults **off**; re-enable with `/api/cmd?usb=1`.
- `/capture.jpg?res=fhd` (1920×1080) is the full-res ceiling: native 4K RGB565
  input is 16.6 MB and won't fit beside the 16.6 MB capture buffer in free PSRAM.

## Web UI (tabs)

Six tabs over a persistent telemetry dashboard + a status bar (connection,
cam/thermal dots, Save-to-NVS, Reboot):

- **RGB camera** — polled hi-res preview (960×540) + full-res download buttons
  (HD / FHD) + SSTV trigger.
- **LWIR camera** — polled thermal preview. NOTE: the MI1602 readout path works,
  but the MI48 isn't yet producing a calibrated frame (task #27), so this shows
  live *uncalibrated* FPA noise until thermal bring-up finishes — not a fault.
- **Image tuning** — AWB toggle, WB R/B, black level, 3×3 CCM.
- **Auto exposure** — AE target + enable, manual exp/gain override.
- **OTA update** — pick a `.bin`, upload, auto-reboot (with rollback).
- **Console** — auto-tails `/api/log` every 1.5 s (auto-scrolls when at bottom).

Only the active tab's heavy poller runs, so switching tabs is what starts/stops
each preview or the log tail.

## Done since P2

Tabbed web UI, poll-based per-view preview (`/snapshot.jpg?view=`), full-res
HD+FHD download (`/capture.jpg?res=`), auto-tailing console, OTA, `/api/log`,
system-health telemetry + reboot + NVS-persisted settings, live colour tuning
(AWB / WB / black-level / CCM) + focus aid, and web-triggered SSTV. **All
hardware-verified (2026-06-06)** at move-imager.local, including a full OTA
round-trip and console responsiveness under preview load.

## Still TODO

- RS422 link + command dispatch to the OBC — the flight comms (task #28 step 2).
- Board temperature (AT30TS74 @0x48) — needs the LP-I²C bus brought up.
- Store the FHD still to the FAT `storage` partition (pairs with RS422 downlink).
- Re-check SDIO at 40 MHz (the C6 reports the PCB supports it; we run 20 MHz).
- WiFi link can flap at low RSSI (≲ -80 dBm), and a marginal USB rail can brown
  out the board under camera+radio load — keep it near the AP on solid power.
- MI1602 thermal: I²C/SPI readout is live, but the MI48 isn't producing a
  calibrated frame yet (task #27, hardware) — the LWIR tab shows raw noise.
