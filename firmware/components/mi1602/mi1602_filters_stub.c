/*
 * Stubs for the closed-source MI48 host-side filters.
 *
 * pysenxor wraps a binary `mi48_filters_*.{dll,so,dylib}` (see
 * pysenxor-1.6.7/senxor/dllwrapper.py) that ships with the desktop SDK.
 * Source for those binaries was NOT provided alongside the Python or the
 * C++ samples, so we cannot literally port them.
 *
 * What you get equivalent functionality from, on this driver, today:
 *
 *   - "Temporal" (rolling time average)
 *         → mi1602_set_filter_temporal()
 *           (enables the MI48 on-chip filter, registers 0xD0..0xD2)
 *
 *   - STARK (edge-preserving contrast)
 *         → mi1602_set_filter_stark()
 *           (enables the MI48 on-chip implementation, register 0x20)
 *
 *   - Median (3×3 spatial median)
 *         → mi1602_set_filter_median()
 *           (enables the MI48 on-chip implementation, register 0x30)
 *
 *   - Min/Max stabilization (temporal smoothing of frame extrema)
 *         → mi1602_set_mms() for the on-chip version (register 0x25),
 *           or mi1602_minmax_stab_* for a host-side equivalent that we
 *           did port from pysenxor's denoise.py.
 *
 *   - Contrast enhancement (ISOCC'2024)
 *         → mi1602_contrast_enhance() — source-ported from
 *           contrast_enhancement.py.
 *
 * If you specifically need bit-identical output to the desktop SDK's
 * DNLCE, host-side STARK, KXMS or distance-compensation filters, you
 * need the proprietary source from Meridian Innovation. The stubs below
 * exist as documented placeholders so callers know where they would
 * plug in.
 */

#include "esp_err.h"

#if 0  /* not declared in mi1602.h yet — placeholder API for future expansion */

esp_err_t mi1602_dnlce(const uint16_t *in, uint16_t *out, size_t npix);
esp_err_t mi1602_stark_host(const uint16_t *in, uint16_t *out, size_t npix);
esp_err_t mi1602_kxms(uint16_t *frame, size_t npix);
esp_err_t mi1602_distance_compensation(float t_max_c, float t_bg_c,
                                       float distance_m, float *out_t_c);

#endif

/* The translation unit must contain at least one symbol so that older
 * toolchains do not warn about an empty object file. */
const char mi1602_filters_stub_doc[] =
    "Closed-source filters from Meridian Innovation's mi48_filters DLL "
    "are NOT bundled. See header comment in mi1602_filters_stub.c.";
