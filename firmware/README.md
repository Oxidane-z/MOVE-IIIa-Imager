# CubeSat Imaging Payload Firmware

ESP-IDF v6.0 firmware for the ESP32-P4 (M5Stamp-P4 module). Drives a
SC850SL 8 MP visible camera over MIPI CSI-2 and an MI1602 thermal camera
over SPI, and talks to the CubeSat OBC over RS-422 UART.

| | |
|---|---|
| Project root  | `firmware/`                          |
| Target        | `esp32p4`                            |
| IDF version   | v6.0.1 (latest stable, 2026-04-27)    |
| Memory        | 16 MB flash · 32 MB octal PSRAM       |
| First flash   | see [HOW_TO_BUILD.md](HOW_TO_BUILD.md) |
| MI1602 driver | see [MI1602_INTEGRATION.md](MI1602_INTEGRATION.md) |
| Project memory | see [`../PROJECT_MEMORY.md`](../PROJECT_MEMORY.md) — read this first if you've stepped away |

## Quick start

```powershell
# In the ESP-IDF 6.0 PowerShell that the installer creates:
cd "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware"
idf.py set-target esp32p4
idf.py build
idf.py -p COM7 flash monitor   # replace COM7 with your actual port
```

The default `main.c` runs **Phase 1**: opens I²C, attempts to talk to the
SC850SL at address 0x30, prints the chip ID (expect `0x9d1e`). On a bare
Stamp-P4 with no carrier the I²C probe NACKs — that's the expected log
output before the FPC is wired.

## Folder layout

```
firmware/
├── CMakeLists.txt          # top-level — finds MI1602 sibling automatically
├── sdkconfig.defaults      # locked-in build settings (PSRAM, flash, etc.)
├── partitions.csv          # 16 MB layout
├── main/                   # application entry point (Phase 1 milestone)
└── components/
    ├── sc850sl/            # SC850SL camera driver (full register tables)
    ├── isp_pipeline/       # P4 ISP wiring         (stub — phase 2)
    ├── rs422_protocol/     # OBC comm              (stub — phase 5)
    └── sstv_robot36/       # SSTV encoder          (stub — phase 4)
```

The MI1602 thermal-camera driver is vendored in-tree at
`components/mi1602/` (moved here June 2026 from the old `../../MI1602 Dev/`
sibling so a fresh clone builds standalone). ESP-IDF auto-discovers
`components/`, so no `EXTRA_COMPONENT_DIRS` is needed.
