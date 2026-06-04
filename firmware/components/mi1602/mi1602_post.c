/*
 * Host-side post-processing for MI1602 frames.
 *
 * Functions ported from pysenxor where source is available:
 *   - mi1602_dk_to_celsius_buf       (utility)
 *   - mi1602_remap_u8                ← pysenxor utils.py:208-259
 *   - mi1602_minmax_stab_*           ← pysenxor denoise.py:9-54
 *   - mi1602_contrast_enhance        ← pysenxor contrast_enhancement.py:5-29
 *
 * The closed-source filters (STARK / DNLCE / KXMS host-side, distance
 * compensation) live as stubs in mi1602_filters_stub.c.
 */
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "esp_err.h"

#include "mi1602.h"

/* ----------------------------------------------------------------------- */
/*  dK → °C buffer                                                         */
/* ----------------------------------------------------------------------- */

void mi1602_dk_to_celsius_buf(const uint16_t *dk, float *celsius, size_t npix)
{
    for (size_t i = 0; i < npix; ++i) {
        celsius[i] = (float)dk[i] / 10.0f - 273.15f;
    }
}

/* ----------------------------------------------------------------------- */
/*  Linear remap to uint8 (pysenxor utils.py:208-259)                      */
/* ----------------------------------------------------------------------- */

void mi1602_remap_u8(const uint16_t *src, size_t npix, uint8_t *dst,
                     uint16_t in_min, uint16_t in_max)
{
    if (in_max <= in_min) {
        memset(dst, 0, npix);
        return;
    }
    const uint32_t span = (uint32_t)(in_max - in_min);
    for (size_t i = 0; i < npix; ++i) {
        uint16_t v = src[i];
        if (v <= in_min) {
            dst[i] = 0;
        } else if (v >= in_max) {
            dst[i] = 255;
        } else {
            uint32_t num = (uint32_t)(v - in_min) * 255u;
            dst[i] = (uint8_t)(num / span);
        }
    }
}

/* ----------------------------------------------------------------------- */
/*  Min/Max stabilization (pysenxor denoise.py:9-54)                       */
/*                                                                          */
/*  The caller pre-sorts (ascending) the frame's pixel array. We pick the   */
/*  (n_outliers_min)th element from the bottom and (n_outliers_max+1)th     */
/*  from the top, then optionally smooth them via a rolling average.        */
/* ----------------------------------------------------------------------- */

void mi1602_minmax_stab_init(mi1602_minmax_stab_t *s,
                             uint8_t n_out_min,
                             uint8_t n_out_max,
                             uint8_t ra_depth)
{
    memset(s, 0, sizeof(*s));
    s->n_outliers_min = n_out_min;
    s->n_outliers_max = n_out_max;
    s->ra_depth       = (ra_depth > 8) ? 8 : ra_depth;
}

static uint16_t s_ra_push_and_average(uint16_t *buf, uint8_t depth, uint8_t idx,
                                      bool primed, uint16_t sample)
{
    buf[idx] = sample;
    uint8_t n = primed ? depth : (uint8_t)(idx + 1);
    uint32_t sum = 0;
    for (uint8_t i = 0; i < n; ++i) {
        sum += buf[i];
    }
    return (uint16_t)(sum / n);
}

void mi1602_minmax_stab_apply(mi1602_minmax_stab_t *s,
                              const uint16_t *sorted_data, size_t n,
                              uint16_t *out_min, uint16_t *out_max)
{
    size_t i_min = (s->n_outliers_min < n) ? s->n_outliers_min : 0;
    size_t i_max = (s->n_outliers_max + 1 < n) ? (n - 1 - s->n_outliers_max) : (n - 1);
    uint16_t mn = sorted_data[i_min];
    uint16_t mx = sorted_data[i_max];

    if (s->ra_depth > 0) {
        mn = s_ra_push_and_average(s->ra_min_buf, s->ra_depth, s->ra_idx, s->ra_primed, mn);
        mx = s_ra_push_and_average(s->ra_max_buf, s->ra_depth, s->ra_idx, s->ra_primed, mx);

        if (++s->ra_idx >= s->ra_depth) {
            s->ra_idx = 0;
            s->ra_primed = true;
        }
    }

    *out_min = mn;
    *out_max = mx;
}

/* ----------------------------------------------------------------------- */
/*  Contrast enhancement (pysenxor contrast_enhancement.py:5-29)           */
/*                                                                          */
/*  Three-stream histogram equalisation (ISOCC'2024 by Cheol-Ho Choi et    */
/*  al.): normalise to 0..255, then build three uint8 streams              */
/*    n0 = direct                                                          */
/*    n1 = pow(normalised, 1/gamma) * 255                                  */
/*    n2 = pow(normalised, gamma)   * 255                                  */
/*  Combine their histograms, derive a single CDF, map all three through   */
/*  it, and average. Output is uint8 (0..255).                             */
/* ----------------------------------------------------------------------- */

esp_err_t mi1602_contrast_enhance(const uint16_t *src, size_t npix,
                                  uint8_t *dst, float gamma)
{
    if (src == NULL || dst == NULL || npix == 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (gamma <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    /* 1. find min/max */
    uint16_t mn = src[0], mx = src[0];
    for (size_t i = 1; i < npix; ++i) {
        if (src[i] < mn) mn = src[i];
        if (src[i] > mx) mx = src[i];
    }
    if (mx == mn) {
        memset(dst, 0, npix);
        return ESP_OK;
    }

    const float span = (float)(mx - mn);
    const float inv_gamma = 1.0f / gamma;

    /* 2. histograms (256 bins each) of n0, n1, n2 — combined into hn[] */
    uint32_t hist_combined[256] = { 0 };

    /* First pass: build n0 in `dst` (reused as scratch), accumulate hist */
    for (size_t i = 0; i < npix; ++i) {
        float norm = (float)(src[i] - mn) / span;
        if (norm < 0.0f) norm = 0.0f;
        else if (norm > 1.0f) norm = 1.0f;

        float n0f = norm * 255.0f;
        float n1f = powf(norm, inv_gamma) * 255.0f;
        float n2f = powf(norm, gamma)     * 255.0f;

        uint8_t n0 = (uint8_t)(n0f + 0.5f);
        uint8_t n1 = (uint8_t)(n1f + 0.5f);
        uint8_t n2 = (uint8_t)(n2f + 0.5f);

        dst[i] = n0;        /* stash n0; we'll reuse this slot at the end */
        hist_combined[n0]++;
        hist_combined[n1]++;
        hist_combined[n2]++;
    }

    /* 3. CDF over the averaged histogram (sum of three histograms / 3, but
     *    since we only need ratios, the division is folded into the
     *    normalisation by 3*npix). */
    uint32_t cdf[256];
    uint32_t acc = 0;
    const uint32_t denom = 3u * (uint32_t)npix;
    for (int i = 0; i < 256; ++i) {
        acc += hist_combined[i];
        /* scaled_cdf[i] = round(acc / denom * 255) — keep as uint8 in cdf */
        cdf[i] = (uint32_t)(((uint64_t)acc * 255u + denom / 2u) / denom);
    }

    /* 4. Second pass: lookup i0, i1, i2 and average into dst */
    for (size_t i = 0; i < npix; ++i) {
        float norm = (float)(src[i] - mn) / span;
        if (norm < 0.0f) norm = 0.0f;
        else if (norm > 1.0f) norm = 1.0f;

        uint8_t n0 = dst[i];
        uint8_t n1 = (uint8_t)(powf(norm, inv_gamma) * 255.0f + 0.5f);
        uint8_t n2 = (uint8_t)(powf(norm, gamma)     * 255.0f + 0.5f);

        uint32_t avg = cdf[n0] + cdf[n1] + cdf[n2];
        dst[i] = (uint8_t)((avg + 1u) / 3u);
    }

    return ESP_OK;
}
