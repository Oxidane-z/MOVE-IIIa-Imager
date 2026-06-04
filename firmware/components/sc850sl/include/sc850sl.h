/*
 * SC850SL — SmartSens 8 MP CMOS image sensor driver
 *
 * Target operating mode for the CubeSat imaging payload:
 *   3840 × 2160, 15 fps, RAW10, 2-lane MIPI CSI-2 @ 1080 Mbps/lane.
 *
 * Register init tables 0–3 are baked into the driver from the M5Stack
 * BSP (extracted from libsns_sc850sl.so .data; see PROJECT_MEMORY.md
 * section 3 for provenance).
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Default I²C 7-bit address with SID strapped low (or NC — internal pull-down). */
#define SC850SL_I2C_ADDR_DEFAULT  0x30
/* Sensor ID readback for sanity check (0x3107=0x9d, 0x3108=0x1e). */
#define SC850SL_CHIP_ID           0x9d1e

typedef enum {
    SC850SL_MODE_4K30_4LANE_RAW10     = 0, /* table 0 — M5Stack default */
    SC850SL_MODE_4K30_4LANE_HDR_DOL   = 1, /* table 1 — HDR */
    SC850SL_MODE_4K15_2LANE_RAW10     = 2, /* table 2 ★ CubeSat target */
    SC850SL_MODE_4K15_2LANE_RAW10_ALT = 3, /* table 3 — alt HTS */
} sc850sl_mode_t;

typedef struct {
    /* I²C */
    i2c_master_bus_handle_t i2c_bus;          /* opened by caller */
    uint8_t                 i2c_addr;         /* default SC850SL_I2C_ADDR_DEFAULT */
    uint32_t                i2c_freq_hz;      /* per-device clock, e.g. 100000 */

    /* Sensor control GPIOs (set any to GPIO_NUM_NC = -1 if not used) */
    gpio_num_t              xshutdn_gpio;     /* active-low; -1 if always-on */
    gpio_num_t              rail_en_dovdd;    /* optional rail load-switch ENs */
    gpio_num_t              rail_en_dvdd;
    gpio_num_t              rail_en_avdd;

    /* EXTCLK source */
    gpio_num_t              extclk_gpio;      /* set -1 if external XO supplies clock */
    uint32_t                extclk_hz;        /* normally 24000000 */

    /* Default capture mode */
    sc850sl_mode_t          mode;
} sc850sl_config_t;

typedef struct sc850sl_t *sc850sl_handle_t;

/**
 * Power-up the sensor (rails + XSHUTDN + EXTCLK) and create a driver
 * handle. Does NOT load the init table — call sc850sl_probe() first to
 * verify the I²C link, then sc850sl_load_init_table() to start configuring.
 *
 * @param[in]  cfg  Configuration. Caller retains ownership of i2c_bus.
 * @param[out] out  Handle. Free with sc850sl_deinit().
 */
esp_err_t sc850sl_init(const sc850sl_config_t *cfg, sc850sl_handle_t *out);

/**
 * Read the chip ID register pair (0x3107, 0x3108).
 *
 * @param[in]  h        Handle from sc850sl_init().
 * @param[out] chip_id  16-bit ID. Expect SC850SL_CHIP_ID (0x9d1e).
 *
 * Common failures:
 *   - ESP_ERR_TIMEOUT / ESP_FAIL: I²C NACK → sensor unpowered or wrong address
 *   - ESP_OK but chip_id != 0x9d1e: wrong sensor on the bus
 */
esp_err_t sc850sl_probe(sc850sl_handle_t h, uint16_t *chip_id);

/**
 * Walk the register init table selected by cfg->mode (or by the mode
 * argument here, if you want to override). Blocking; ~200 ms for the
 * full 191-entry RAW10 2-lane table at 100 kHz I²C.
 */
esp_err_t sc850sl_load_init_table(sc850sl_handle_t h, sc850sl_mode_t mode);

/* Output window / cropping (registers 0x3208..0x3213). */
esp_err_t sc850sl_set_window(sc850sl_handle_t h,
                             uint16_t width, uint16_t height,
                             uint16_t x_start, uint16_t y_start);

/* Enable / disable the on-die incremental test pattern (reg 0x4501 bit 3). */
esp_err_t sc850sl_set_test_pattern(sc850sl_handle_t h, bool enable);

/* Mirror / flip (reg 0x3221). */
esp_err_t sc850sl_set_mirror_flip(sc850sl_handle_t h, bool mirror, bool flip);

/* Stream control. Sequencing matters:
 *   - stream_on: 0x302c=0x00 then 0x0100=0x01
 *   - stream_off: 0x0100=0x00 (sensor goes to soft-sleep, keeps regs)
 * See datasheet 1.4.3 — order must not change.                            */
esp_err_t sc850sl_stream_on(sc850sl_handle_t h);
esp_err_t sc850sl_stream_off(sc850sl_handle_t h);

/* Deep sleep (low-power) — 0x0100=0x00 then 0x302c=0x0f. */
esp_err_t sc850sl_sleep(sc850sl_handle_t h);
/* Wake from deep sleep — 0x302c=0x00 then 0x0100=0x01. */
esp_err_t sc850sl_wake(sc850sl_handle_t h);

/* Low-level register access (escape hatch for bring-up / debug). */
esp_err_t sc850sl_write_reg(sc850sl_handle_t h, uint16_t reg, uint8_t val);
esp_err_t sc850sl_read_reg(sc850sl_handle_t h, uint16_t reg, uint8_t *val);

/* Release handle, cut power rails, leave XSHUTDN low. */
void      sc850sl_deinit(sc850sl_handle_t h);

#ifdef __cplusplus
}
#endif
