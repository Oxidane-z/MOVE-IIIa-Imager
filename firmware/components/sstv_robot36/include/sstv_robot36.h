/*
 * Robot36 SSTV encoder.
 *
 * Robot36 transmits a 320×240 YUV image in ~36 seconds. Audio is FM
 * encoded: 1500 Hz = black, 2300 Hz = white. Sync pulses at 1200 Hz.
 *
 * Spec reference: http://www.classicsstv.com/daniels.php  &
 *   https://en.wikipedia.org/wiki/Slow-scan_television#Robot_36
 *
 * Usage:
 *   1. Allocate a sample buffer:  ~36 s × sample_rate × sizeof(int16_t)
 *      At 16 kHz: 36 × 16000 × 2 = 1.15 MB. Use PSRAM.
 *   2. Call sstv_robot36_encode() to fill it from a 320×240 RGB565 frame.
 *   3. Push the buffer to your I²S audio sink at sample_rate.
 *
 * This implementation generates the whole transmission in one pass into
 * a caller-provided buffer (no realtime feeding). Total ~1 MB at 16 kHz.
 */
#pragma once

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SSTV_ROBOT36_WIDTH   320
#define SSTV_ROBOT36_HEIGHT  240
/* At 16 kHz, the full transmission needs this many samples (≈ 36.23 s). */
#define SSTV_ROBOT36_SAMPLES_AT_16K  580000

/* Estimate the sample count required at a given rate. */
size_t sstv_robot36_sample_count(uint32_t sample_rate_hz);

/* Encode a 320×240 RGB565 image into mono int16 PCM samples.
 * Returns ESP_OK on success or ESP_ERR_INVALID_SIZE if the buffer is
 * too small. *out_samples receives the actual sample count written. */
esp_err_t sstv_robot36_encode(const uint16_t *rgb565,    /* 320×240 */
                              uint32_t sample_rate_hz,    /* 16000 .. 48000 */
                              int16_t  *out_pcm,
                              size_t    out_pcm_capacity,
                              size_t   *out_samples);

#ifdef __cplusplus
}
#endif
