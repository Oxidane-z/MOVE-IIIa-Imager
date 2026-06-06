/*
 * Per-target application entry point.
 *
 * One of `app_run_sc850sl()` or `app_run_sc202cs_tab5()` is compiled
 * into the binary, selected by the CAMERA_TARGET_* Kconfig choice.
 * main.c just calls `app_run()` which is an alias defined by
 * whichever target module is linked in.
 *
 * Each entry point is responsible for the full lifetime of the
 * application — it should not return.
 */
#pragma once

#include "sdkconfig.h"
#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

void app_run(void);

/* Probe-only check on the MI1602 thermal camera. Safe to call even when
 * the IR module isn't connected (logs a warning + returns silently).
 * Compiles to a no-op when CONFIG_MI1602_ENABLED is unset.
 *
 * If `shared_bus` is non-NULL, the existing I²C master bus is used (e.g.
 * Tab5 BSP grabs port 1 first; share that handle). Otherwise the helper
 * opens its own bus using the CONFIG_MI1602_I2C_* pins. */
struct i2c_master_bus_t;
typedef struct i2c_master_bus_t *i2c_master_bus_handle_t;
void mi1602_try_probe(i2c_master_bus_handle_t shared_bus);

/* Capture one MI1602 thermal frame, false-color it, and nearest-neighbor
 * scale it into dst (RGB565) at dw×dh, writing rows with `dst_stride`
 * pixels of pitch (so it can target a sub-rectangle of a wider composite
 * buffer). Returns ESP_ERR_INVALID_STATE if the IR module wasn't probed
 * successfully, or a capture error. Compiles to a stub returning
 * ESP_ERR_NOT_SUPPORTED when CONFIG_MI1602_ENABLED is unset.
 *
 * `bytes_per_pixel` of dst is 2 (RGB565). Returns ESP_OK on success. */
esp_err_t mi1602_aux_capture_rgb565(uint16_t *dst, int dw, int dh,
                                    int dst_stride);
bool      mi1602_aux_available(void);
/* Latched SenXor (MI48) die temperature in Celsius from the last thermal
 * capture; < -273 if none captured yet / header invalid. For ground telemetry. */
float     mi1602_aux_temp_c(void);

#ifdef __cplusplus
}
#endif
