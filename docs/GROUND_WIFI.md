# Ground-test WiFi + web station (C6 addon)

Ground-only convenience layer: WiFi + a browser control page + OTA, so the
flight board can be driven and updated **without a USB cable**. Everything is
gated behind `CONFIG_GROUND_WIFI_ENABLE` — the flight build pulls none of it.

> Status (2026-06): P1 (link+STA) and P2 (telemetry+control) are
> hardware-verified. P3 (JPEG live preview), OTA-over-WiFi, mDNS, on-demand
> ISP, and `/api/log` are **built clean but not yet hardware-tested** (board was
> unplugged before the flash). First job when hardware is back: one USB flash,
> then exercise the web UI.

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

| Endpoint        | Method | Purpose                                            |
|-----------------|--------|----------------------------------------------------|
| `/`             | GET    | control page (embedded `ground_index.html`)        |
| `/api/tlm`      | GET    | telemetry JSON (exp/gain/mean/sat/wb/timings/ip…)  |
| `/api/cmd`      | POST   | control — query params below                       |
| `/snapshot.jpg` | GET    | one hardware-JPEG frame of the 640×240 composite   |
| `/stream`       | GET    | MJPEG live preview (RGB \| thermal)                |
| `/api/ota`      | POST   | firmware update — raw `.bin` as the body           |
| `/api/log`      | GET    | recent device log (last ~4 KB)                     |

`/api/cmd` params (integers; omit to leave unchanged):
`ae_target=<mean>`, `ae_en=0|1`, `exp=<lines>`, `gain=<x1024>`, `usb=0|1`
(toggle the legacy USB binary preview push), `sstv=1`/`capture=1` (P4, TODO).

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

- The software ISP (the ~0.6 s 4K→RGB box-filter downscale) runs **on demand**:
  only when a `/stream` viewer is connected *or* the USB push is on. An idle
  board (no viewer) just does capture + raw stats + AE + telemetry — cheap.
- The legacy USB binary preview push (host `tools/usb_preview.py`) defaults
  **off** on the WiFi build; the web `/stream` replaces it and it frees the USB
  console + ~0.36 s/frame of I/O. Re-enable at runtime with `/api/cmd?usb=1`.

## Still TODO (P4 remainder)

- Full-resolution HD still download (`/capture.raw` or a full-frame JPEG).
- SSTV TX trigger from the web (`/api/cmd?sstv=1`; task #23 re-enables the TX).
- Re-check SDIO at 40 MHz (the C6 reports the PCB supports it; we run 20 MHz).
