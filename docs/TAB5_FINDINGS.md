# M5Stack Tab5 — sensor & pinout findings

> **⚠ Historical / superseded.** When this document was written we
> believed the Tab5 camera was an SC2336 based on M5Stack's
> M5Tab5-UserDemo source. Later (Phase 2) we ran the chip and read
> back PID `0xeb52`, which is **SC202CS** — a different SmartSens
> sensor. The actual driver we use is `esp_cam_sensor`'s SC202CS, not
> SC2336. **For the current state of Tab5 integration see
> `PROJECT_MEMORY.md` §8.5.** The rest of this file is kept for the
> pinout investigation it captures (I²C and I²S pins are accurate
> regardless of which sensor is on the bus).

Source: cloned [M5Tab5-UserDemo](https://github.com/m5stack/M5Tab5-UserDemo)
to `m5_research/M5Tab5-UserDemo/`.

## ⚠ The sensor is **SC2336**, NOT SC2356 or SC202CS *(WRONG — see banner above)*

Marketing pages disagree:

| Source | Claims sensor is | Reality |
|---|---|---|
| M5Stack shop page | SC2356 | wrong (marketing label) |
| Espressif BSP component README | SC202CS | wrong |
| **Tab5 user-demo source code** | **SC2336** | **ground truth** |

Evidence in the user-demo:
- `platforms/tab5/components/esp_cam_sensor/sensors/sc2336/sc2336.c`
  (1819-line production driver, Apache-2.0, Espressif Systems)
- `platforms/tab5/components/esp_cam_sensor/CMakeLists.txt`:
  `if(CONFIG_CAMERA_SC2336)` …
- `platforms/tab5/components/esp_video/examples/.../app_sc2336_custom_settings.h`

SC2336 belongs to the **same SmartSens family as our target SC850SL**.
Same I²C protocol (16-bit reg + 8-bit val, SCCB), same address default
(`0x30`), same PLL preamble convention (0x36e9/0x36f9), same lane-control
register (0x3018), same DT codes.

## Tab5 BSP pinout (camera & audio)

Extracted from
`platforms/tab5/components/m5stack_tab5/include/bsp/m5stack_tab5.h` and
`m5stack_tab5.c`.

### Camera

| Signal | Pin | Note |
|---|---|---|
| **MCLK / EXTCLK** | **GPIO 36** | LEDC ch 0, `LEDC_TIMER_1_BIT`, **24 MHz** — same recipe we independently arrived at. M5Stack's `bsp_cam_osc_init()` uses freq=24000000, duty=1, exactly mirroring our `sc850sl.c`. |
| I²C SDA | GPIO 31 | bus port 0 (`BSP_I2C_NUM`) |
| I²C SCL | GPIO 32 | |
| RESET / XSHUTDN | not in BSP header | likely tied high or behind I²C IO expander |
| PWDN | not in BSP header | same |
| MIPI CSI lanes | hardware-fixed | P4 has dedicated MIPI pins; no GPIO-selectable |

I²C 7-bit address: **0x30** (`#define SC2336_SCCB_ADDR 0x30` in
`sc2336.h`) — coincidentally the same as SC850SL's default.

### Audio (ES8388 codec + ES7210 AEC front-end on shared I²S)

| Signal | Pin |
|---|---|
| I²S BCLK | GPIO 27 |
| I²S MCLK | GPIO 30 |
| I²S LRCLK / WS | GPIO 29 |
| I²S DOUT (P4 → ES8388 speaker) | GPIO 26 |
| I²S DIN (ES7210 mic → P4) | GPIO 28 |
| Power amp enable | not used (-1) |

### Display (5″ 1280×720 MIPI-DSI, ILI9881C / ST7123)

| Signal | Pin |
|---|---|
| Backlight | GPIO 22 |
| RESET | NC (controlled via I²C IO expander) |
| Touch INT | NC |
| Touch RESET | NC (IO expander) |

### microSD (SDIO 4-bit)

D0=GPIO39, D1=GPIO40, D2=GPIO41, D3=GPIO42, CMD=GPIO44, CLK=GPIO43.

## How M5Stack drives the sensor — software stack

Tab5 demo uses Espressif's **`esp_video` + `esp_cam_sensor`** combo (the
official Linux V4L2-style API for ESP-IDF v6.0+):

```
[app] open("/dev/video0") → ioctl VIDIOC_*
   ↓
esp_video (V4L2 layer)
   ↓
esp_video_csi_device.c (MIPI CSI port → CSI host + ISP)
   ↓
esp_cam_sensor (sensor abstraction; auto-detect)
   ↓
sensors/sc2336/sc2336.c (concrete driver)
```

Configuration is via `esp_video_init_csi_config_t`:

```c
static const esp_video_init_csi_config_t csi_config[] = {
    {
        .sccb_config = {
            .init_sccb = true,
            .i2c_config = { .port=0, .scl_pin=32, .sda_pin=31 },
            .freq = 100000,
        },
        .reset_pin = -1,
        .pwdn_pin  = -1,
    },
};
```

Then the app code uses standard Linux V4L2 ioctls (`VIDIOC_QUERYCAP`,
`VIDIOC_S_FMT`, `VIDIOC_REQBUFS`, `VIDIOC_QBUF`, `VIDIOC_DQBUF`,
`VIDIOC_STREAMON`) — exactly like a Linux V4L2 camera app.

## What this changes for our firmware

1. **Don't write our own SC2336 driver.** Use Espressif's
   `esp_cam_sensor` + `esp_video` components via the component manager:

   ```yaml
   # firmware/main/idf_component.yml
   dependencies:
     espressif/esp_video:       "^1.0"
     espressif/esp_cam_sensor:  "^1.3"
   ```

2. **Keep our `sc850sl/` component intact.** It's still the path for the
   real flight sensor — SC850SL is *not* in `esp_cam_sensor`, so our
   hand-rolled driver remains necessary.

3. **Add a Tab5 build variant** that uses SC2336 via the official driver
   (for end-to-end pipeline validation) and a flight-target build variant
   that uses our `sc850sl/` (for the real mission).

4. **Validate our `sc850sl/` API shape** against the Espressif
   `esp_cam_sensor` API. They both wrap "init sensor → stream on → stream
   off → set window/gain/exposure". Once we make our API surface match
   `esp_cam_sensor_device_t` ops, we could submit `sc850sl` upstream as
   a new sensor in their library — useful for the wider community.

5. **The LEDC-1-bit-duty-24-MHz trick we discovered** matches M5Stack's
   own BSP exactly. Validation that our approach was correct.

## SC2336 default formats (per `sc2336.c`)

- RAW8 1280×720 @ 30 fps   (default)
- RAW10 1280×720 @ 30 fps
- RAW10 1920×1080 @ 25/30 fps
- RAW8 800×800 @ 30 fps
- RAW8 1920×1080 @ 30 fps

For our use case on Tab5, **RAW10 1920×1080 @ 30 fps** is the natural
pick — it's the highest fidelity, fits straight through the P4 ISP (1080p
is the ISP's max input), and exercises the entire pipeline at full
resolution.

## Next concrete step for Tab5 integration

Add a Kconfig switch:

```
choice CAMERA_TARGET_SENSOR
    prompt "Visible camera sensor"
    default CAMERA_TARGET_SC850SL
    config CAMERA_TARGET_SC850SL
        bool "SC850SL (flight target)"
    config CAMERA_TARGET_SC2336_TAB5
        bool "SC2336 on M5Stack Tab5 (bench-only)"
endchoice
```

And in `main/main.c`, branch:

```c
#if CONFIG_CAMERA_TARGET_SC2336_TAB5
    /* Init via esp_video / esp_cam_sensor — auto-detects SC2336. */
    esp_video_init(&video_cfg);
    open("/dev/video0", ...);
#else
    /* Hand-rolled SC850SL bring-up. */
    sc850sl_init(&cfg, &cam);
    sc850sl_probe(cam, ...);
#endif
```

Both build out of the same source tree; the user picks one via
`idf.py menuconfig`. This means we can develop the rest of the pipeline
(ISP/JPEG/SSTV/RS-422) on the SC2336-Tab5 path while waiting for the
SC850SL hardware.
