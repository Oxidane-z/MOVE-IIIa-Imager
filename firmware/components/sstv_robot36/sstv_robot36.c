/*
 * Robot36 SSTV encoder — see header for spec reference.
 *
 * Implementation notes:
 *  - We model the FM modulator as a phase accumulator advancing each
 *    output sample by `2π·freq/Fs`. No filtering; output is a clean sine.
 *  - Y is the full 320 columns on every line; chrominance alternates
 *    between R-Y (odd lines) and B-Y (even lines) at half horizontal rate.
 *  - Each pixel maps to a frequency: 1500 Hz at value 0, 2300 Hz at 255.
 */
#include "sstv_robot36.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#include "esp_log.h"

static const char *TAG = "sstv36";

/* ----- protocol constants (milliseconds, frequencies in Hz) ----- */

#define FREQ_BLACK        1500.0f
#define FREQ_WHITE        2300.0f
#define FREQ_SYNC         1200.0f
#define FREQ_PORCH        1500.0f
#define FREQ_VIS_BIT0     1300.0f   /* "0" bit */
#define FREQ_VIS_BIT1     1100.0f   /* "1" bit */
#define FREQ_VIS_START    1900.0f
#define FREQ_VIS_HEADER   1900.0f

#define MS_LEADER          300.0f
#define MS_BREAK           10.0f
#define MS_VIS_BIT         30.0f
#define MS_LINE_SYNC        9.0f
#define MS_LINE_PORCH       3.0f
#define MS_Y_SCAN          88.0f
#define MS_CHROMA_SEP_EVEN  4.5f
#define MS_CHROMA_SEP_ODD   4.5f
#define MS_CHROMA_PORCH     1.5f
#define MS_CHROMA_SCAN     44.0f

/* Trailing silence appended after the 240th scanline.
 *
 * Robot36 has no protocol-defined end-of-frame marker — the receiver
 * is expected to know to stop after 240 lines. But mobile SSTV apps
 * (Robot36 / SSTV Slow Scan, etc.) use a trailing-silence VOX trigger
 * to decide when to FINALIZE and auto-save the decoded image. Without
 * a quiet tail they leave the frame in a "still receiving" state.
 *
 * 1.0 s of zero PCM after the last scan gets us reliable auto-save on
 * every decoder we've tried. */
#define MS_TRAILING_SILENCE 1000.0f

#define VIS_CODE_ROBOT36   0x88   /* per spec */

/* ----- helpers ----- */

static inline float clampf(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline uint8_t rgb565_to_y(uint16_t px)
{
    uint8_t r5 = (px >> 11) & 0x1f;
    uint8_t g6 = (px >> 5)  & 0x3f;
    uint8_t b5 =  px        & 0x1f;
    uint8_t r = (r5 << 3) | (r5 >> 2);
    uint8_t g = (g6 << 2) | (g6 >> 4);
    uint8_t b = (b5 << 3) | (b5 >> 2);
    /* BT.601 luma */
    int y = (66*r + 129*g + 25*b + 128) >> 8;
    return (uint8_t)(y + 16);
}

static inline uint8_t rgb565_to_ry(uint16_t px)
{
    uint8_t r5 = (px >> 11) & 0x1f;
    uint8_t g6 = (px >> 5)  & 0x3f;
    uint8_t b5 =  px        & 0x1f;
    uint8_t r = (r5 << 3) | (r5 >> 2);
    uint8_t g = (g6 << 2) | (g6 >> 4);
    uint8_t b = (b5 << 3) | (b5 >> 2);
    /* V = R-Y (Cr-ish) */
    int v = (112*r - 94*g - 18*b + 128) >> 8;
    return (uint8_t)(v + 128);
}

static inline uint8_t rgb565_to_by(uint16_t px)
{
    uint8_t r5 = (px >> 11) & 0x1f;
    uint8_t g6 = (px >> 5)  & 0x3f;
    uint8_t b5 =  px        & 0x1f;
    uint8_t r = (r5 << 3) | (r5 >> 2);
    uint8_t g = (g6 << 2) | (g6 >> 4);
    uint8_t b = (b5 << 3) | (b5 >> 2);
    /* U = B-Y (Cb-ish) */
    int u = (-38*r - 74*g + 112*b + 128) >> 8;
    return (uint8_t)(u + 128);
}

/* ----- streaming oscillator (phase accumulator) ----- */

typedef struct {
    int16_t  *out;
    size_t    cap;
    size_t    n;          /* samples written so far */
    float     phase;      /* radians */
    uint32_t  fs;
} osc_t;

static void osc_freq_ms(osc_t *o, float freq, float ms)
{
    const float dphi = 2.0f * (float)M_PI * freq / (float)o->fs;
    const size_t nsamps = (size_t)((ms * (float)o->fs) / 1000.0f + 0.5f);
    for (size_t i = 0; i < nsamps; ++i) {
        if (o->n >= o->cap) return;
        float s = sinf(o->phase);
        /* 80% peak headroom — leaves room for codec output stage. */
        o->out[o->n++] = (int16_t)(s * 26000.0f);
        o->phase += dphi;
        if (o->phase > 2.0f * (float)M_PI) o->phase -= 2.0f * (float)M_PI;
    }
}

/* Emit `nsamps` samples whose frequency follows `freq_at(t)` linearly
 * sampled from `pixels` (length `npix`). Each pixel value 0..255 maps
 * to FREQ_BLACK..FREQ_WHITE. Pixel duration = ms / npix. */
static void osc_scanline(osc_t *o, const uint8_t *pixels, size_t npix, float ms)
{
    const size_t total = (size_t)((ms * (float)o->fs) / 1000.0f + 0.5f);
    if (npix == 0 || total == 0) return;
    for (size_t i = 0; i < total; ++i) {
        if (o->n >= o->cap) return;
        size_t pix_idx = (i * npix) / total;
        float v = (float)pixels[pix_idx] / 255.0f;
        float freq = FREQ_BLACK + v * (FREQ_WHITE - FREQ_BLACK);
        float dphi = 2.0f * (float)M_PI * freq / (float)o->fs;
        float s = sinf(o->phase);
        o->out[o->n++] = (int16_t)(s * 26000.0f);
        o->phase += dphi;
        if (o->phase > 2.0f * (float)M_PI) o->phase -= 2.0f * (float)M_PI;
    }
}

/* ----- API ----- */

size_t sstv_robot36_sample_count(uint32_t sample_rate_hz)
{
    /* Header (~700 ms) + 240 lines × 150 ms + trailing silence + margin */
    const float total_ms = 700.0f + 240.0f * 150.0f + MS_TRAILING_SILENCE + 200.0f;
    return (size_t)(total_ms * (float)sample_rate_hz / 1000.0f + 1024);
}

esp_err_t sstv_robot36_encode(const uint16_t *rgb565,
                              uint32_t sample_rate_hz,
                              int16_t  *out_pcm,
                              size_t    out_pcm_capacity,
                              size_t   *out_samples)
{
    if (!rgb565 || !out_pcm || sample_rate_hz < 8000) {
        return ESP_ERR_INVALID_ARG;
    }
    if (out_pcm_capacity < sstv_robot36_sample_count(sample_rate_hz)) {
        return ESP_ERR_INVALID_SIZE;
    }

    osc_t o = { .out = out_pcm, .cap = out_pcm_capacity, .n = 0,
                .phase = 0.0f, .fs = sample_rate_hz };

    /* --- Header --- */
    osc_freq_ms(&o, FREQ_VIS_HEADER, MS_LEADER);
    osc_freq_ms(&o, FREQ_SYNC,       MS_BREAK);
    osc_freq_ms(&o, FREQ_VIS_HEADER, MS_LEADER);

    /* --- VIS code (start bit + 8 data + parity + stop bit) --- */
    osc_freq_ms(&o, FREQ_VIS_START, MS_VIS_BIT);    /* start = 1200 Hz */
    uint8_t vis = VIS_CODE_ROBOT36;
    uint8_t parity = 0;
    for (int b = 0; b < 7; ++b) {                    /* 7 data bits, LSB first */
        int bit = (vis >> b) & 1;
        parity ^= bit;
        osc_freq_ms(&o, bit ? FREQ_VIS_BIT1 : FREQ_VIS_BIT0, MS_VIS_BIT);
    }
    osc_freq_ms(&o, parity ? FREQ_VIS_BIT1 : FREQ_VIS_BIT0, MS_VIS_BIT);
    osc_freq_ms(&o, FREQ_VIS_START, MS_VIS_BIT);    /* stop = 1200 Hz */

    /* --- 240 image lines, two image lines per Robot36 "frame line"
     * pattern. Robot36 actually interleaves: each transmitted scanline
     * carries Y for the current line PLUS subsampled R-Y or B-Y for
     * pairs of lines. We approximate the standard order:
     *   line k even (k=0,2,4,...): Y_k  then R-Y averaged of (k,k+1)
     *   line k odd  (k=1,3,5,...): Y_k  then B-Y averaged of (k-1,k)
     * Average over two lines for the chroma row. */
    uint8_t y_row[SSTV_ROBOT36_WIDTH];
    uint8_t chroma_row[SSTV_ROBOT36_WIDTH / 2];

    for (int row = 0; row < SSTV_ROBOT36_HEIGHT; ++row) {
        const uint16_t *line = &rgb565[row * SSTV_ROBOT36_WIDTH];
        for (int x = 0; x < SSTV_ROBOT36_WIDTH; ++x) {
            y_row[x] = rgb565_to_y(line[x]);
        }

        /* Build chroma_row: avg of two consecutive image lines, then
         * subsample by 2 horizontally. R-Y on even, B-Y on odd. */
        int pair_row = (row & ~1);    /* even line of the pair */
        const uint16_t *l0 = &rgb565[pair_row * SSTV_ROBOT36_WIDTH];
        const uint16_t *l1 = &rgb565[(pair_row + (pair_row + 1 < SSTV_ROBOT36_HEIGHT ? 1 : 0)) * SSTV_ROBOT36_WIDTH];
        for (int x = 0; x < SSTV_ROBOT36_WIDTH / 2; ++x) {
            int x0 = x * 2;
            int x1 = x * 2 + 1;
            uint16_t a, b;
            if ((row & 1) == 0) { /* even line → R-Y */
                a = (rgb565_to_ry(l0[x0]) + rgb565_to_ry(l1[x0])) / 2;
                b = (rgb565_to_ry(l0[x1]) + rgb565_to_ry(l1[x1])) / 2;
            } else {              /* odd line → B-Y */
                a = (rgb565_to_by(l0[x0]) + rgb565_to_by(l1[x0])) / 2;
                b = (rgb565_to_by(l0[x1]) + rgb565_to_by(l1[x1])) / 2;
            }
            chroma_row[x] = (uint8_t)((a + b) / 2);
        }

        /* sync + porch + Y */
        osc_freq_ms(&o, FREQ_SYNC,  MS_LINE_SYNC);
        osc_freq_ms(&o, FREQ_PORCH, MS_LINE_PORCH);
        osc_scanline(&o, y_row, SSTV_ROBOT36_WIDTH, MS_Y_SCAN);

        /* Chroma separator + porch + chroma scan.
         *
         * Per Robot36 standard (Dayton paper / classicsstv reference):
         *   even line carries R-Y, separator tone = 1500 Hz (BLACK)
         *   odd  line carries B-Y, separator tone = 2300 Hz (WHITE)
         *
         * Previously this was inverted (WHITE before R-Y), which made
         * spec-compliant mobile decoders interpret our R-Y data as
         * "B-Y" (since they key off the separator) and route it into
         * the wrong chroma channel — visible as swapped red/blue (a
         * blue sky decodes as red on phones). The Tab5 self-decode
         * mirrored the same swap on the RX side so it never saw the
         * problem internally. */
        if ((row & 1) == 0) {
            osc_freq_ms(&o, FREQ_BLACK, MS_CHROMA_SEP_EVEN);  /* R-Y separator = 1500 Hz */
        } else {
            osc_freq_ms(&o, FREQ_WHITE, MS_CHROMA_SEP_ODD);   /* B-Y separator = 2300 Hz */
        }
        osc_freq_ms(&o, 1900.0f, MS_CHROMA_PORCH);
        osc_scanline(&o, chroma_row, SSTV_ROBOT36_WIDTH / 2, MS_CHROMA_SCAN);
    }

    /* --- Trailing silence so mobile SSTV decoders auto-finalize / save. */
    {
        const size_t silence_n = (size_t)((MS_TRAILING_SILENCE * (float)o.fs) / 1000.0f + 0.5f);
        const size_t room = (o.n < o.cap) ? (o.cap - o.n) : 0;
        const size_t n_to_write = (silence_n < room) ? silence_n : room;
        for (size_t i = 0; i < n_to_write; ++i) {
            o.out[o.n++] = 0;
        }
    }

    ESP_LOGI(TAG, "encoded %u samples (%.2f s @ %u Hz, incl. %.1f ms tail silence)",
             (unsigned) o.n, (double) o.n / (double) sample_rate_hz,
             (unsigned) sample_rate_hz, (double) MS_TRAILING_SILENCE);

    if (out_samples) *out_samples = o.n;
    return ESP_OK;
}
