/*
 * MI1602 — Meridian Innovation Senxor MI48xx thermal camera driver
 *
 * Target: ESP32-P4 with ESP-IDF v6.0+
 *
 * Transport: SPI (frame data, full-duplex 8-bit MSB-first, mode 0, big-endian
 *            uint16 pixels) + I²C (control/status registers, chip address
 *            0x40 or 0x41 depending on ADDR strap).
 *
 * FPA: 160 × 120 pixels, 16-bit deci-Kelvin (dK = K × 10).
 *      0 °C  -> dK = 2731
 *     50 °C  -> dK = 3231
 *
 * Companion init/streaming flow modelled after pysenxor 1.6.7
 *   - registers:    pysenxor/senxor/mi48.py:99-228
 *   - bootup:       pysenxor/senxor/mi48.py:378-417
 *   - capture:      pysenxor/example/single_capture_spi.py:28-217
 *   - SPI semantics: pysenxor/senxor/interfaces.py:61-122
 *
 * All driver entry points return esp_err_t. mi1602_handle_t is opaque.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ----------------------------------------------------------------------- */
/*  Constants                                                              */
/* ----------------------------------------------------------------------- */

#define MI1602_I2C_ADDR_DEFAULT       0x40
#define MI1602_I2C_ADDR_ALT           0x41

#define MI1602_FPA_COLS               160
#define MI1602_FPA_ROWS               120
#define MI1602_FPA_PIXELS             (MI1602_FPA_COLS * MI1602_FPA_ROWS)
#define MI1602_FRAME_HDR_WORDS        MI1602_FPA_COLS   /* one header row */
#define MI1602_FRAME_PIXEL_BYTES      (MI1602_FPA_PIXELS * 2)
#define MI1602_FRAME_HDR_BYTES        (MI1602_FRAME_HDR_WORDS * 2)
#define MI1602_FRAME_TOTAL_BYTES      (MI1602_FRAME_HDR_BYTES + MI1602_FRAME_PIXEL_BYTES)

/* STATUS register bits — mirror pysenxor mi48.py:215-220 */
#define MI1602_STATUS_READOUT_TOO_SLOW   0x02
#define MI1602_STATUS_SXIF_ERROR         0x04
#define MI1602_STATUS_CAPTURE_ERROR      0x08
#define MI1602_STATUS_DATA_READY         0x10
#define MI1602_STATUS_BOOTING_UP         0x20

/* FRAME_MODE register bits */
#define MI1602_MODE_SINGLE_FRAME         0x01
#define MI1602_MODE_CONTINUOUS_STREAM    0x02
#define MI1602_MODE_NO_HEADER            0x20
#define MI1602_MODE_DATA_TYPE_ADC        0x80

/* ----------------------------------------------------------------------- */
/*  Configuration & handle                                                 */
/* ----------------------------------------------------------------------- */

typedef struct {
    /* I²C control bus (caller-owned). */
    i2c_master_bus_handle_t  i2c_bus;
    uint8_t                  i2c_addr;        /* MI1602_I2C_ADDR_DEFAULT (0x40) or 0x41 */
    uint32_t                 i2c_freq_hz;     /* 100000 .. 400000 */

    /* SPI frame bus. The driver creates the device on the supplied host.
     * If the host is not initialised yet, the driver will call
     * spi_bus_initialize() with the sclk/miso/mosi pins; otherwise it
     * just adds itself as a device (set spi_bus_already_initialised). */
    spi_host_device_t        spi_host;        /* SPI2_HOST or SPI3_HOST on P4 */
    bool                     spi_bus_already_initialised;
    gpio_num_t               sclk_gpio;
    gpio_num_t               miso_gpio;
    gpio_num_t               mosi_gpio;       /* driver pushes dummy 0s here */
    gpio_num_t               cs_n_gpio;       /* manual GPIO control (MI48 needs CS asserted across the full frame) */
    uint32_t                 spi_freq_hz;     /* 7.8e6 / 15.6e6 / 31.2e6; default 15.6e6 */

    /* Control GPIOs (-1 = unused). */
    gpio_num_t               reset_n_gpio;    /* active-low pulse: 50 µs assert, 50 ms deassert */
    gpio_num_t               data_ready_gpio; /* active-high input; level 1 == frame ready */
    gpio_num_t               sysclk_gpio;     /* LEDC clock to module (if module needs one) */
    uint32_t                 sysclk_hz;       /* typically 3000000 */
} mi1602_config_t;

typedef struct mi1602_t *mi1602_handle_t;

/* ----------------------------------------------------------------------- */
/*  Frame header                                                           */
/* ----------------------------------------------------------------------- */

typedef struct {
    uint16_t frame_counter;          /* word[0] */
    float    senxor_vdd;             /* word[1] / 10000 */
    float    senxor_temperature;     /* word[2] / 100 + KELVIN_0 */
    uint32_t timestamp;              /* word[3] | (word[4] << 16) */
    float    pixel_max_c;            /* word[5] / 10 + KELVIN_0 */
    float    pixel_min_c;            /* word[6] / 10 + KELVIN_0 */
    uint16_t crc_from_header;        /* word[7] (header-reported CRC) */
    uint16_t iplock1;                /* word[10] */
    uint16_t iplock2;                /* word[11] */
    bool     crc_ok;                 /* driver-computed CRC matches header */
} mi1602_frame_header_t;

/* ----------------------------------------------------------------------- */
/*  Lifecycle                                                              */
/* ----------------------------------------------------------------------- */

esp_err_t mi1602_init(const mi1602_config_t *cfg, mi1602_handle_t *out);
void      mi1602_deinit(mi1602_handle_t h);

/* ----------------------------------------------------------------------- */
/*  Probe / camera info                                                    */
/* ----------------------------------------------------------------------- */

/* Lightweight I²C reachability check (reads FW_VERSION_1). */
esp_err_t mi1602_probe(mi1602_handle_t h);

esp_err_t mi1602_get_camera_info(mi1602_handle_t h,
                                 uint8_t  *senxor_type,
                                 uint8_t  *module_type,
                                 uint16_t *fw_version,   /* (major << 8) | minor */
                                 uint8_t  *fw_build,
                                 uint8_t   serial[6]);

/* ----------------------------------------------------------------------- */
/*  Low-level register access                                              */
/* ----------------------------------------------------------------------- */

esp_err_t mi1602_reg_read (mi1602_handle_t h, uint8_t reg, uint8_t *val);
esp_err_t mi1602_reg_write(mi1602_handle_t h, uint8_t reg, uint8_t  val);

/* ----------------------------------------------------------------------- */
/*  Typed configuration helpers                                            */
/* ----------------------------------------------------------------------- */

esp_err_t mi1602_set_fps(mi1602_handle_t h, uint8_t fps_divisor);     /* fps ≈ 25.5 / divisor */
esp_err_t mi1602_set_emissivity (mi1602_handle_t h, float emissivity); /* 0 .. 1 */
esp_err_t mi1602_set_offset_corr(mi1602_handle_t h, float kelvin);     /* signed ±12.7 K */
esp_err_t mi1602_set_sens_factor(mi1602_handle_t h, float factor);     /* 1.00 → 0x64 */
esp_err_t mi1602_set_otf        (mi1602_handle_t h, float factor);     /* OBJ_TEMP_FACTOR */

/* MI48 on-chip filters — controlled via I²C, no host CPU cost. */
esp_err_t mi1602_set_filter_temporal(mi1602_handle_t h, bool enable, uint16_t strength);
esp_err_t mi1602_set_filter_stark   (mi1602_handle_t h, bool enable);
esp_err_t mi1602_set_filter_median  (mi1602_handle_t h, bool enable);
esp_err_t mi1602_set_mms            (mi1602_handle_t h, bool enable);

/* ----------------------------------------------------------------------- */
/*  Bootup / status                                                        */
/* ----------------------------------------------------------------------- */

/* Reset (if pin available), clear FRAME_MODE, poll until BOOTING_UP=0. */
esp_err_t mi1602_bootup(mi1602_handle_t h);

esp_err_t mi1602_get_status(mi1602_handle_t h, uint8_t *status);
esp_err_t mi1602_get_mode  (mi1602_handle_t h, uint8_t *mode);

/* ----------------------------------------------------------------------- */
/*  Frame capture                                                          */
/* ----------------------------------------------------------------------- */

/* One-shot capture: writes FRAME_MODE=SINGLE_FRAME, waits DATA_READY, reads
 * one frame over SPI, byte-swaps to host order. If out_hdr is non-NULL the
 * header row is captured and parsed (CRC over pixels is also computed and
 * stored in out_hdr->crc_ok). If out_hdr is NULL the NO_HEADER mode bit is
 * set, saving 320 bytes per frame.
 *
 * out_pixels must point to MI1602_FPA_PIXELS uint16 words. */
esp_err_t mi1602_capture_single(mi1602_handle_t h,
                                uint16_t *out_pixels,
                                mi1602_frame_header_t *out_hdr);

/* Debug / bring-up: clock n raw bytes off MISO with CS_N asserted for the whole
 * transfer. NO I2C, NO FRAME_MODE trigger, NO DATA_READY wait, NO byte-swap --
 * just whatever the MI48 happens to be putting on the SPI bus. Use it to see if
 * the sensor is streaming frame data independent of the I2C control handshake
 * (e.g. still SXIF_ERROR / BOOTING_UP on I2C but pixels flowing on SPI). n is
 * clamped to the driver's internal scratch size (>= one full frame). */
esp_err_t mi1602_spi_read_raw(mi1602_handle_t h, uint8_t *buf, size_t n);

/* Capture a frame WITHOUT the I2C FRAME_MODE trigger: wait for DATA_READY
 * (GPIO if wired, else STATUS poll), then read the frame off SPI and parse the
 * header. For a module that already has a frame pending (DATA_READY asserted)
 * while its I2C control writes are NACKing (stuck boot). Same outputs as
 * mi1602_capture_single. */
esp_err_t mi1602_capture_no_trigger(mi1602_handle_t h, uint16_t *out_pixels,
                                    mi1602_frame_header_t *out_hdr);

/* Read a frame off SPI RIGHT NOW: no FRAME_MODE trigger AND no DATA_READY wait.
 * For a module already free-running on SPI (continuous stream) while its
 * DATA_READY line / boot handshake is unavailable. The frame may be torn if the
 * read isn't aligned to a frame boundary, so always check out_hdr->crc_ok and
 * frame_counter. */
esp_err_t mi1602_read_frame_now(mi1602_handle_t h, uint16_t *out_pixels,
                                mi1602_frame_header_t *out_hdr);

/* Streaming: spawns a background task that delivers frames via cb.
 * cb runs on the streaming task, NOT in ISR context, so it may block. */
typedef void (*mi1602_frame_cb_t)(const uint16_t *pixels,
                                  const mi1602_frame_header_t *hdr,
                                  void *user_ctx);

esp_err_t mi1602_start_streaming(mi1602_handle_t h,
                                 mi1602_frame_cb_t cb,
                                 void *user_ctx);
esp_err_t mi1602_stop_streaming(mi1602_handle_t h);

/* ----------------------------------------------------------------------- */
/*  Temperature conversion (pure functions, no I/O)                        */
/* ----------------------------------------------------------------------- */

static inline float mi1602_dk_to_celsius(uint16_t dk)
{
    return (float)dk / 10.0f - 273.15f;
}

static inline float mi1602_dk_to_fahrenheit(uint16_t dk)
{
    return ((float)dk / 10.0f - 273.15f) * 9.0f / 5.0f + 32.0f;
}

void mi1602_dk_to_celsius_buf(const uint16_t *dk, float *celsius, size_t npix);

/* ----------------------------------------------------------------------- */
/*  Host-side post-processing                                              */
/* ----------------------------------------------------------------------- */

/* Min/Max stabilization — port of pysenxor denoise.py:9-54.
 * Caller pre-sorts the frame (ascending). n_outliers_min/max are clipped
 * indices from the ends of the sorted array. ra_depth ∈ [0, 8]; 0 disables
 * rolling-average smoothing of the (min, max) outputs. */
typedef struct {
    uint8_t  n_outliers_min;
    uint8_t  n_outliers_max;
    uint16_t ra_min_buf[8];
    uint16_t ra_max_buf[8];
    uint8_t  ra_depth;
    uint8_t  ra_idx;
    bool     ra_primed;
} mi1602_minmax_stab_t;

void mi1602_minmax_stab_init(mi1602_minmax_stab_t *s,
                             uint8_t n_out_min,
                             uint8_t n_out_max,
                             uint8_t ra_depth);
void mi1602_minmax_stab_apply(mi1602_minmax_stab_t *s,
                              const uint16_t *sorted_data, size_t n,
                              uint16_t *out_min, uint16_t *out_max);

/* Contrast enhancement (ISOCC'2024 by Cheol-Ho Choi et al.) —
 * port of pysenxor contrast_enhancement.py:5-29.
 * src: uint16 frame; dst: uint8 (0..255). gamma ≈ 1.5 is the default. */
esp_err_t mi1602_contrast_enhance(const uint16_t *src, size_t npix,
                                  uint8_t *dst, float gamma);

/* Linear remap to uint8 — port of pysenxor utils.py:remap (208-259).
 * Pixels at or below in_min map to 0; at or above in_max map to 255. */
void mi1602_remap_u8(const uint16_t *src, size_t npix, uint8_t *dst,
                     uint16_t in_min, uint16_t in_max);

#ifdef __cplusplus
}
#endif
