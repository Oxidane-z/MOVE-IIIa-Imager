# M5Stack LLM630 SC850SL Configuration (extracted from llm-camera 1.9)

Extracted from `gSc850sl*Attr` symbols in `opt/m5stack/bin/llm_camera-1.9`
(non-stripped ARM64 ELF with DWARF debug info).

These are Axera SDK convention `AX_SNS_*_ATTR_T` structures.

## gSc850slMipiAttr (20 bytes)

```
+0000  ePhyMode      = 0           # MIPI mode
+0004  eLaneNum      = 4           # ★ 4 lanes
+0008  nDataRate     = 0x50  (80)  # likely units-encoded
+000c  nDataLaneMap  = {0,1,3,4,2,5,0,0}   # per-lane source mapping
```

## gSc850slSnsAttr (40 bytes) — sensor work-point

```
+0000  nWidth        = 3840        # ★ resolution
+0004  nHeight       = 2160
+0008  fFrameRate    = 30.0  (float)  # ★ 30 fps
+000c  eSnsType      = 1           # MIPI interface type
+0010  eRawType      = 10          # ★ RAW10  (NOT RAW12!)
```

## gSc850slSnsClkAttr (8 bytes)

```
+0000  eClkSrc       = 0
+0004  nFreq         = 0x016e3600 = 24,000,000   # ★ EXTCLK = 24 MHz
```

## gSc850slChn0Attr (36 bytes)

```
+0000  width   = 3840
+0004  height  = 2160
+0008  stride  = 3840
+000c  pixfmt? = 3
+0010  ?       = 1
+0014  ?       = 2
+0018  ?       = 4
```

## gSc850slPipeAttr / gSc850slDevAttr

Both reference 3840×2160. DevAttr has 5 copies of the (3840, 2160) tuple at
strided offsets — looks like multiple Bayer windows / crop rectangles, all
identical → single full-frame readout.

## What this changes

The OpenIPC driver we already pulled is for **27 MHz, 4-lane, RAW12,
1080 Mbps/lane**. M5Stack's actual flight configuration is:

| Param | OpenIPC table | M5Stack (this binary) |
|-------|---------------|-----------------------|
| EXTCLK | **27 MHz** | **24 MHz** |
| Lanes | 4 | 4 |
| Bit depth | **RAW12** | **RAW10** |
| Mode | 4K30 | 4K30 |
| FPS | 30 | 30 |

So **two of the most important parameters differ**:
- 24 MHz vs 27 MHz → all PLL ratios in the OpenIPC table are
  slightly wrong at 24 MHz. SC850SL might still lock and just run ~11 %
  slow, but it's not the M5Stack-tuned operating point.
- RAW10 vs RAW12 → 0x3031 = 0x0a (not 0x0c); per-pixel data is
  20 % less.

## What's still missing

The actual SC850SL register init table lives in two external files that
this .deb does NOT bundle:

- `/opt/etc/sc850sl_sdr.bin`     (linear / SDR mode register table)
- `/opt/etc/sc850sl_hdr_2x.bin`  (HDR mode register table)

And the sensor driver shared library:

- `/opt/lib/libsns_sc850sl.so`   (loads .bin, programs sensor)

## How to get those files

1. **Pull straight from a flashed LLM630 if you have one**:
   `scp root@<llm630-ip>:/opt/etc/sc850sl_sdr.bin .`
   `scp root@<llm630-ip>:/opt/lib/libsns_sc850sl.so .`

2. **Find the M5Stack BSP package** — M5Stack apt repo likely has a
   `axera-bsp`, `libsns-sc850sl`, or similar package with these files.
   The .deb the user has is the camera *application*, not the sensor driver.

3. **Look in M5Stack's official image / SDK release on GitHub** —
   they distribute LLM630 system images that contain these files.

Once we get `sc850sl_sdr.bin`, parse it as `{u16 reg, u8 val, u8 pad}`
records (or whatever the format is — should be obvious from the first few
bytes; will start with `0x0103 0x01` for soft reset). That gives us the
M5Stack-tuned init table for 24 MHz, 4-lane, RAW10.

## Implication for the ESP32-P4 2-lane plan

Even without the .bin file, **knowing M5Stack uses 24 MHz changes the bench
plan**:

- The Stamp-P4 can natively output 24 MHz on a GPIO via the LEDC peripheral
  or a CLK_OUT pin — no external XO needed on the carrier.
- We have two known operating points now (OpenIPC 27 MHz/RAW12 and M5Stack
  24 MHz/RAW10) — they're not enough to interpolate, but they bracket the
  PLL parameter space.
- If we get the M5Stack init table, we can use it directly with one change
  (0x3018 lane count) and likely get 4K15 RAW10 on 2 lanes via
  half-rate auto-fallback.
