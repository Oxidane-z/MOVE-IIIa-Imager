# 🎯 Confirmed: M5Stack already validated SC850SL 2-lane 4K15 RAW10

Extracted from `/opt/etc/sc850sl_sdr_2lane_15fps.ini` in the LLM630 rootfs.

## Side-by-side comparison

| Parameter | 4-lane (default M5Stack) | **2-lane (target — what we want!)** |
|---|---|---|
| Resolution | 3840 × 2160 | **3840 × 2160 (no crop!)** |
| Frame rate | 30 fps | **15 fps** |
| Bit depth | RAW10 | RAW10 |
| MIPI lanes | 4 | **2** |
| Per-lane rate | 1080 Mbps | **1080 Mbps (identical)** |
| Pixel format | 0x85 (RAW10_PACKED) | 0x85 (RAW10_PACKED) |
| MIPI DT | 0x2B (decimal 43) | 0x2B |
| Sensor mode | 1 (linear SDR) | 1 (linear SDR) |
| **nSettingIndex** | **0** | **33** ★ |
| nDataLaneMap | {0, 1, 3, 4} | {0, 1, 3, 4} |
| nClkLane | {2, 5} | {2, 5} |

## What this means

- **2-lane is NOT a hack**. M5Stack already shipped a production-validated
  2-lane mode in their Axera BSP. The SC850SL fully supports it.
- **Per-lane rate stays at 1080 Mbps** — the sensor handles lane reduction
  by halving the frame rate (30 → 15), not by changing the MIPI clock.
  This means timing margin on the bus is identical to the 4-lane mode.
- **ESP32-P4 budget check:** P4 D-PHY ceiling is 1.5 Gbps/lane. 1080 Mbps
  fits with 28 % margin. Should lock cleanly.
- **Lane mapping is identical** between 2L and 4L modes (just the upper
  two lanes go unused in 2L mode). Carrier wiring is simpler.

## The magic field: `nSettingIndex = 33`

The sensor driver `libsns_sc850sl.so` contains multiple register init
tables indexed 0…N. The .ini files select among them:

- `nSettingIndex = 0`  → 4K30 4-lane (default in 4lane.ini)
- `nSettingIndex = 33` → 4K15 2-lane ★

The actual SC850SL register writes for setting 33 are buried inside the
.so. Next step: extract them via DWARF symbol table from
`libsns_sc850sl.so.0.0.0`.

## Implications for the CubeSat / ESP32-P4 plan

This is the ideal CubeSat capture mode:

- **4K full resolution** (8 MP, full sensor FOV) — what you wanted
- **15 fps readout** = 67 ms per frame — plenty fast for one-shot stills
- **2-lane MIPI** — matches Stamp-P4 hardware
- **RAW10** — fits straight into ESP32-P4 ISP (which maxes at 1080p, BUT
  we feed the 4K through with the ISP's memory-input mode for crop/scale
  to 1080p in hardware afterwards; or stream straight to PSRAM and use
  the PPA to scale)
- **No binning needed** — 4K capture + software/hardware 1/4 downscale
  to 1920×1080 gives us all our use cases (S-band downlink + SSTV)

## What's still missing

- The actual register init sequence inside `libsns_sc850sl.so` for
  setting index 33 — must extract from the .so binary
- EXTCLK frequency for the 2-lane mode (almost certainly still 24 MHz,
  but confirm from the .so or from XML/JSON config)
- AE/AGC tuning for low-light (the `sc850sl_sdr_ptnw768_600G_25fps.bin`
  is a binary tuning table — likely contains AE LUT, BLC, gamma curves)

## All extracted files (gold inventory)

```
opt/etc/sc850sl_sdr_2lane_15fps.ini             ← THE KEY FILE
opt/etc/sc850sl_sdr_4lane.ini
opt/etc/sc850sl_sdr_4lane_620q.ini
opt/etc/sc850sl_hdr_4lane.ini
opt/etc/sc850sl_hdr_4lane_620q.ini
opt/etc/sc850sl_single_sdr_2lane_15fps_entry.ini
opt/etc/sc850sl_single_sdr_4lane_entry.ini
opt/etc/sc850sl_single_hdr_4lane_entry.ini
opt/etc/sc850sl_sdr_mode3_switch_mode7.bin       (674 KB - mode switch fw)
opt/etc/sc850sl_sdr_ptnw768_600G_25fps.bin       (674 KB - 25fps PTN mode)
opt/etc/sc850sl_hdr_2x_ratio_default.bin         (189 KB - HDR ratio LUT)
opt/etc/sc850sl_hdr_2x_ratio_1to1.bin            (189 KB - HDR ratio LUT)
opt/lib/libsns_sc850sl.so.0.0.0                  (2.73 MB - driver) ★★
opt/bin/FRTDemo/config/ipc/sensor/sc850sl.json
opt/bin/FRTTest/config/ipc/sensor/sc850sl.json
```
