# `mi1602` — Meridian Innovation MI1602 thermal-camera driver for ESP-IDF

160 × 120 16-bit (deci-Kelvin) thermal sensor on **ESP32-P4**, ESP-IDF v6.0+.

The driver is a port of Meridian Innovation's **official `pysenxor 1.6.7`
SDK** (`pysenxor-1.6.7/senxor/`), keeping the same register usage, bootup
flow, frame layout and CRC convention. Meridian's C++ `libsenxor_samples`
package was used as a secondary reference for SPI behaviour. Refer to those
upstream projects for their original licensing terms.

## Transport

| Bus | Purpose |
|-----|---------|
| **I²C** | Register reads/writes (status, FRAME_MODE, emissivity, filters). 7-bit address 0x40 or 0x41 depending on the ADDR strap. |
| **SPI** | Frame data. Mode 0, MSB-first, 8-bit word, big-endian uint16 pixels. Driver pulses CS_N via GPIO because the MI48 needs CS asserted across the *entire* frame transfer (the SPI hardware CS toggles per transaction). |

Optional GPIOs: `RESET_N` (active-low pulse), `DATA_READY` (input, level 1
== frame ready), `SYSCLK` (LEDC clock output if the module needs one).

## Public API

See [`include/mi1602.h`](include/mi1602.h). Quick tour:

```c
mi1602_config_t cfg = { ... };
mi1602_handle_t cam = NULL;
ESP_ERROR_CHECK(mi1602_init(&cfg, &cam));
ESP_ERROR_CHECK(mi1602_bootup(cam));
ESP_ERROR_CHECK(mi1602_probe(cam));

mi1602_set_fps(cam, 6);                  /* ≈ 4 fps */
mi1602_set_emissivity(cam, 0.95f);

uint16_t pixels[MI1602_FPA_PIXELS];
mi1602_frame_header_t hdr;
ESP_ERROR_CHECK(mi1602_capture_single(cam, pixels, &hdr));

/* pixels[i] is uint16 deci-Kelvin; convert as needed. */
float c = mi1602_dk_to_celsius(pixels[i]);
```

For continuous streaming:

```c
void on_frame(const uint16_t *p, const mi1602_frame_header_t *h, void *ctx) {
    /* runs on the driver's stream task — may block briefly */
}
mi1602_start_streaming(cam, on_frame, NULL);
/* ... */
mi1602_stop_streaming(cam);
mi1602_deinit(cam);
```

## On-chip filters vs. host post-processing

The MI48 firmware itself implements STARK, temporal averaging, 3×3 median,
and min/max stabilization. Enable them with:

```c
mi1602_set_filter_stark   (cam, true);
mi1602_set_filter_temporal(cam, true, /* strength */ 10);
mi1602_set_filter_median  (cam, true);
mi1602_set_mms            (cam, true);
```

The Python SDK additionally ships `mi48_filters_*.{dll,so,dylib}` — a
closed-source binary doing DNLCE, host-side STARK, KXMS and distance
compensation. **The source for that binary was not provided**, so a
literal port is not possible. The driver provides:

- `mi1602_minmax_stab_*` — ported from `pysenxor/senxor/denoise.py`
- `mi1602_contrast_enhance` — ported from `pysenxor/senxor/contrast_enhancement.py` (ISOCC'2024)
- `mi1602_remap_u8`, `mi1602_dk_to_celsius_buf` — utility maps

See `mi1602_filters_stub.c` for the documented gap.

## Integration with the SC850SL firmware

The parent project at `../../SC850SL Dev/firmware/` auto-discovers this
component via `EXTRA_COMPONENT_DIRS` (see
`SC850SL Dev/firmware/CMakeLists.txt:12-18` and
`SC850SL Dev/firmware/MI1602_INTEGRATION.md`). Just having
`MI1602 Dev/components/mi1602/` exist is enough — no symlinks, no
submodules.

To consume the driver from `main`, add `mi1602` to the `REQUIRES` list
of `firmware/main/CMakeLists.txt` and call `mi1602_init(...)` from
`app_main`.

## Pin defaults

Pin defaults (in `Kconfig.projbuild`) deliberately avoid the GPIOs the
SC850SL driver claims (8/9 for I²C0, 10/11/14 for sensor control).
Default placement:

| Signal | GPIO |
|--------|------|
| I²C SDA / SCL | 18 / 19 |
| SPI SCLK / MISO / MOSI / CS_N | 20 / 21 / 22 / 23 |
| RESET_N | 24 |
| DATA_READY | 25 |
| SYSCLK | -1 (off) |

Adjust to your carrier board.

## Source provenance

| Concept | pysenxor reference |
|---------|--------------------|
| Register map | `senxor/mi48.py:99-228` |
| Bootup sequence | `senxor/mi48.py:378-417` |
| SPI semantics | `senxor/interfaces.py:61-122` |
| I²C semantics | `senxor/interfaces.py:28-58` |
| CRC algorithm | `senxor/mi48.py:13, 270, 541-546` |
| Header parse | `senxor/mi48.py:1206-1251` |
| Single-shot capture example | `example/single_capture_spi.py:28-217` |
| MinMaxStabilization | `senxor/denoise.py:9-54` |
| Contrast enhance (ISOCC'24) | `senxor/contrast_enhancement.py:5-29` |
