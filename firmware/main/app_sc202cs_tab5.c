/*
 * Tab5 demo: LVGL UI + LIVE SC202CS preview + button → Robot36 SSTV → 1W speaker.
 *
 *   Pipeline:
 *     1. bsp_display_start() — display+touch+LVGL on 1280×720 MIPI-DSI
 *     2. bsp_camera_start()  — esp_video brings up SC202CS via CSI+ISP
 *     3. open /dev/video0, set RGB565, REQBUFS, mmap, STREAMON
 *     4. cam_task continuously DQBUF → copies into preview buffer → QBUF
 *     5. lv_timer periodically refreshes the LVGL image widget
 *     6. button event → snapshot latest preview → downscale → Robot36 → speaker
 *
 *   Audio routed to onboard 1W speaker via
 *     bsp_audio_codec_speaker_init() → esp_codec_dev (ES8388 → PAM amp → speaker).
 */
#include "app.h"

#if defined(CONFIG_CAMERA_TARGET_SC202CS_TAB5)

#include <string.h>
#include <math.h>
#include <errno.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "sdkconfig.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "linux/videodev2.h"

#include "lvgl.h"
#include "bsp/m5stack_tab5.h"
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "esp_video_device.h"
#include "esp_video_init.h"
#include "driver/ppa.h"
#include "esp_dsp.h"

#include "sstv_robot36.h"

static const char *TAG = "app/tab5";

/* ---------------------------------------------------------------- *
 *  Configuration
 * ---------------------------------------------------------------- */

#define SAMPLE_RATE_HZ          16000
#define PCM_CAPACITY            (SAMPLE_RATE_HZ * 40)   /* 40 s: 36 SSTV + 1 silence + headroom */

#define CAM_PIX_FMT             V4L2_PIX_FMT_RGB565
#define CAM_BUFFER_COUNT        2

/* Preview canvas in the UI.
 *
 * The SC202CS streams 1280×720 LANDSCAPE. We rotate CW 90° so the scene
 * looks right-side-up when the user holds the Tab5 portrait. The UI
 * preview BOX is LANDSCAPE 4:3 (640×480) on the portrait screen, and the
 * rotated content FILLS it edge-to-edge (no pillarbox).
 *
 * To get a 4:3 landscape output from a CW90 rotation, the source crop
 * must be 3:4 PORTRAIT before rotation. Largest 3:4 portrait crop from
 * the 1280×720 sensor is 540×720 (full vertical FOV, ~42% of horizontal
 * FOV, centered). After CW90 → 720×540 landscape → uniform 8/9× scale
 * → 640×480 = preview box exactly.
 *
 * Trade-off: we keep 100% of the sensor's vertical FOV and crop ~58%
 * of horizontal — looks like a moderate "zoom in" vs. the full sensor
 * view. That's the only no-distortion way to fill a 4:3 landscape box
 * with content rotated 90° from a 16:9 landscape sensor.
 *
 * All rotation + scale is done in HARDWARE via PPA at ~1 ms/frame
 * (vs. ~85 ms for the original CPU column-wise path).
 *
 * The SSTV thumbnail (g_image_buf, 320×240 landscape) is built on
 * demand at button-press time, NOT every frame.
 */
#define PREVIEW_W               640
#define PREVIEW_H               480

/* PPA writes the entire preview buffer (no pillarbox bars). */
#define ROT_W                   PREVIEW_W
#define ROT_H                   PREVIEW_H
#define ROT_X_OFFSET            0
#define ROT_Y_OFFSET            0

/* Crop region in sensor (landscape) coordinates: 540 wide × 720 tall
 * (3:4 portrait), centered horizontally → x = (1280-540)/2 = 370. */
#define SRC_CROP_X              370
#define SRC_CROP_Y              0
#define SRC_CROP_W              540
#define SRC_CROP_H              720

/* ---------------------------------------------------------------- *
 *  Phase 2A — mic + FFT + spectrum waterfall (no decoder yet)
 * ---------------------------------------------------------------- */

/* Mic capture at 16 kHz / mono / int16, same as Robot36 audio rate. */
#define MIC_SR_HZ               16000

/* FFT window size. 1024 → 15.625 Hz/bin, ~64 ms/column at 16 kHz. */
#define FFT_N                   1024
#define FFT_HALF                (FFT_N / 2)         /* meaningful bins (0..N/2) */

/* Number of bins displayed vertically. Cover 0..~4 kHz of the spectrum
 * — enough to span the full SSTV signal band (1100..2300 Hz) plus
 * voice / general audio context. */
#define WF_BIN_COUNT            256                  /* bins 0..255 = 0..4 kHz */

/* Waterfall image size on screen. Slimmer (130 H) so the decoded SSTV
 * image directly below can be rendered at a near-4:3 aspect (640×440
 * display, native buffer 320×240). */
#define WF_W                    640                  /* time history          */
#define WF_H                    130                  /* frequency axis         */

/* ---------------------------------------------------------------- *
 *  Phase 2B — Robot36 RX decoder (zero-crossing freq → state machine)
 * ---------------------------------------------------------------- */

/* Robot36 image dimensions (transmitted spec). */
#define DEC_W                   320
#define DEC_H                   240

/* Audio-sample counts per Robot36 timing field at 16 kHz. */
#define SAMP_SYNC               (MIC_SR_HZ *  9 / 1000)   /* 9 ms   = 144 */
#define SAMP_PORCH              (MIC_SR_HZ *  3 / 1000)   /* 3 ms   =  48 */
#define SAMP_Y_LINE             (MIC_SR_HZ * 88 / 1000)   /* 88 ms  =1408 */
#define SAMP_SEP                (MIC_SR_HZ * 45 / 10000)  /* 4.5 ms =  72 */
#define SAMP_CPORCH             (MIC_SR_HZ * 15 / 10000)  /* 1.5 ms =  24 */
#define SAMP_CHROMA             (MIC_SR_HZ * 44 / 1000)   /* 44 ms  = 704 */

/* Robot36 FSK frequencies. */
#define DEC_FREQ_SYNC           1200
#define DEC_FREQ_BLACK          1500
#define DEC_FREQ_WHITE          2300

/* Sliding window for instantaneous-frequency estimation via zero-crossing
 * count. 32 samples @ 16 kHz = 2 ms window → crossings × 250 Hz/crossing.
 *
 * Resolution matters: the previous 16-sample window stepped at 500 Hz,
 * which means the only possible ZCR readings are multiples of 500. The
 * sync (1200 Hz) and porch (1500 Hz) frequencies BOTH read as either
 * 1000 or 1500, so the sync band `1050..1350` (which contains no
 * multiple of 500) literally never matched — sync detection never fired
 * and the decoder ran with no row alignment, producing strongly-green
 * frames because mis-averaged chroma converges to ~0 → BT.601 with
 * Cr=Cb=-128 maps to almost-pure green.
 *
 * With 32 samples we get 250 Hz steps and Robot36 tones cleanly separate:
 *   1100 Hz (VIS bit 1)  → 4/5 crossings → 1000 or 1250 Hz reading
 *   1200 Hz (line sync)  → 5 crossings    → 1250 Hz reading
 *   1300 Hz (VIS bit 0)  → 5 crossings    → 1250 Hz reading
 *   1500 Hz (porch)      → 6 crossings    → 1500 Hz reading
 *   1900 Hz (leader)     → 7/8 crossings  → 1750 or 2000 Hz reading
 *   2300 Hz (white)      → 9 crossings    → 2250 Hz reading
 *
 * Sync (1250) and porch (1500) are now distinguishable, separator
 * polarity (1500 vs 2250) is unambiguous, and the leader band
 * (1750..2250) is well separated from sync.
 *
 * Caveat: at 32 samples the readout lags by ~16 samples = 1 ms ≈ 3.6
 * Y pixels in the 88 ms scan. Per-pixel averaging absorbs this.
 *
 * Future upgrade if needed: Goertzel filter bank tuned to the six
 * Robot36 tones. ~6×3 muladds per sample = trivial at 16 kHz, gives
 * proper SNR weighting per tone. */
#define DEC_FWIN                32

/* ---------------------------------------------------------------- *
 *  Globals
 * ---------------------------------------------------------------- */

static lv_obj_t *g_status_label;
static lv_obj_t *g_preview_img;
static lv_obj_t *g_capture_btn;     /* left button: snapshot preview → g_image_buf */
static lv_obj_t *g_button;          /* right button: transmit Robot36 from snapshot */
static lv_obj_t *g_tx_bar;          /* slim progress bar shown during TX */
static volatile bool g_have_snapshot = false;   /* set by Capture, checked by Send */
static lv_image_dsc_t g_preview_dsc;        /* LVGL image desc, points at preview_buf */

static esp_codec_dev_handle_t g_spk = NULL;

static int16_t  *g_pcm_buf;                 /* SSTV PCM, PSRAM */
static uint16_t *g_image_buf;               /* 320×240 RGB565 snapshot, PSRAM */
static uint16_t *g_preview_buf;             /* 480×640 RGB565 portrait, rotated+cropped for UI */
static SemaphoreHandle_t g_preview_mutex;
static volatile bool g_have_frame = false;

/* Camera state. */
static int           g_cam_fd = -1;
static uint8_t      *g_cam_bufs[CAM_BUFFER_COUNT] = {0};
static uint32_t      g_cam_buf_len[CAM_BUFFER_COUNT] = {0};
static uint32_t      g_cam_width = 0, g_cam_height = 0;

/* PPA hardware rotator/scaler client. Initialized in cam_init(). */
static ppa_client_handle_t g_ppa = NULL;

/* Mic + FFT + waterfall state (Phase 2A). */
static esp_codec_dev_handle_t g_mic = NULL;
static lv_obj_t       *g_waterfall_img = NULL;
static lv_image_dsc_t  g_waterfall_dsc = {0};
static uint16_t       *g_waterfall_buf = NULL;       /* WF_W × WF_H RGB565 */
static int16_t        *g_mic_samples   = NULL;       /* FFT_N int16 PCM */
static float          *g_fft_data      = NULL;       /* FFT_N * 2 floats (cplx) */
static float          *g_fft_window    = NULL;       /* FFT_N Blackman-Harris */

/* Robot36 RX visualizer (Phase 2B — color raster, scrolls up from bottom).
 *
 * Each 2400-sample row of audio is interpreted using Robot36's known
 * within-line layout:
 *
 *   sample  0  ..  144  : 9 ms 1200 Hz sync
 *   sample 144 ..  192  : 3 ms 1500 Hz porch
 *   sample 192 .. 1600  : 88 ms Y luma scan  → 320 Y pixels
 *   sample 1600.. 1672  : 4.5 ms separator (1500 Hz / 2300 Hz polarity
 *                                            tells us R-Y vs B-Y)
 *   sample 1672.. 1696  : 1.5 ms 1900 Hz porch
 *   sample 1696.. 2400  : 44 ms chroma scan → 160 R-Y or B-Y samples
 *
 * Two consecutive rows form a "pair": one row carries Y₀ + R-Y, the next
 * carries Y₁ + B-Y. We classify each row's parity by reading the chroma
 * separator polarity at samples 1600..1672 — per Robot36 spec, BLACK
 * (1500 Hz) → next chroma is R-Y, WHITE (2300 Hz) → next chroma is B-Y —
 * so the decoder self-aligns parity per-row regardless of where we
 * tune in mid-frame. When we complete a B-Y row with a stashed R-Y row's
 * Y already on hand, we combine into RGB565 for both rows, memmove the
 * whole image buffer up by 2 rows, and paint the new pair at the bottom.
 *
 * Frame-start lock comes from the VIS leader detector (≈200 ms of
 * sustained 1900 Hz → leader confirmed → suppress sync triggers for
 * ~690 ms while the rest of leader + break + 11 VIS bits play out →
 * arm the very next 1200 Hz pulse as line 0). We don't decode the VIS
 * code byte itself; leader-presence alone is enough to align line-0,
 * and 16-sample ZCR can't reliably distinguish 1100 from 1300 Hz
 * anyway.
 */
#define SAMPLES_PER_ROW         (MIC_SR_HZ * 150 / 1000)   /* 2400 */
#define Y_START_SAMP            192                         /* 12 ms  */
#define Y_END_SAMP              1600                        /* 100 ms */
#define C_START_SAMP            1696                        /* 106 ms */
#define C_END_SAMP              2400                        /* 150 ms */
#define Y_LEN_SAMP              (Y_END_SAMP - Y_START_SAMP) /* 1408   */
#define C_LEN_SAMP              (C_END_SAMP - C_START_SAMP) /* 704    */

/* Chroma-separator averaging window.
 *
 * Critical timing: the ZCR estimator at `samples_in_row=N` reflects the
 * last 32 samples of audio, i.e. signal positions [N-31, N]. AND the
 * sync-lock snaps `samples_in_row=80` at the moment the score crosses
 * threshold, which (with +1/-1 dynamics on a 95%-in-band sync pulse) is
 * actually ~89 samples = 5.6 ms into the sync pulse. So a generic
 * `samples_in_row=N` reads signal position N+9 with a 32-sample tail.
 *
 * The real separator spans signal positions [1600, 1672]. For the ZCR
 * readout to be cleanly INSIDE the separator we need:
 *     (N+9) - 31 >= 1600  →  N >= 1622
 *     (N+9)      <= 1672  →  N <= 1663
 * → SEP_AVG = [1625, 1660] gives ~35 clean samples on each end.
 *
 * Earlier values (e.g. 1610..1662) pulled the average toward the
 * Y-region tail (1500..1599), and for bright scenes (Y high → freq
 * ~2000 Hz reading) every row's separator average came out above the
 * 1875 Hz threshold — BLACK separators were misclassified as WHITE,
 * `cb[]` never got refreshed from BSS-zero, and the decoded image
 * came out heavily green due to BT.601 with Cb≈0. */
#define SEP_AVG_START           1625
#define SEP_AVG_END             1660
#define SEP_FREQ_THRESH         1875

static lv_obj_t       *g_decoded_img = NULL;
static lv_image_dsc_t  g_decoded_dsc = {0};
static uint16_t       *g_decoded_buf = NULL;         /* DEC_W × DEC_H RGB565 */

/* Sync-lock parameters.
 *
 * The sync tone (1200 Hz, 9 ms = 144 samples) reads in the 32-sample
 * sliding ZCR as ALTERNATING 4 and 5 crossings — i.e. it flips between
 * 1000 Hz and 1250 Hz readings every ~7 samples as the window slides
 * through phase. So we widen the band to catch BOTH readings:
 *   950 ≤ freq ≤ 1300  →  in-band (1000 or 1250)
 *   freq = 1500 (porch)  →  out of band ✓
 *   freq = 1750 (1900 leader) → out of band ✓
 *
 * We score the run with a leaky integrator (+1 in-band, −1 out, cap
 * at SYNC_SCORE_MAX) and fire when it crosses SYNC_SCORE_THRESH. With
 * ~95% in-band during a real sync pulse, the score climbs at
 * +0.9/sample → reaches the 80-point threshold in ~89 samples = 5.6 ms,
 * comfortably inside the 9 ms sync window. After a lock, we lock out
 * sync detection for ~140 ms to prevent re-fire during the same row's
 * pixel data. */
#define SYNC_FREQ_LO            950
#define SYNC_FREQ_HI            1300
#define SYNC_SCORE_MAX          120
#define SYNC_SCORE_THRESH       80
#define SYNC_LOCKOUT_SAMPLES    (MIC_SR_HZ * 140 / 1000)   /* 2240 samples */

/* VIS leader detection — same score-based approach.
 *
 * Robot36 sends 300 ms of 1900 Hz before the VIS code byte. With
 * 32-sample ZCR, 1900 Hz reads as 7 or 8 crossings → 1750 / 2000 Hz,
 * flipping every few samples. Wide band catches both.
 *
 * Score climbs ~+0.9/sample → 200 ms of leader yields ~2880-point
 * score; threshold 2400 fires reliably ~150 ms into the leader. After
 * detection we suppress sync for ~690 ms (covers the remaining 150 ms
 * of leader, the 10 ms break, the 300 ms second leader half, and the
 * ~230 ms of VIS bits) and arm the next genuine sync as line 0.
 *
 * Hysteresis: re-arming requires the score to drop below MAX/4 first,
 * so a single VIS doesn't keep retriggering itself. */
#define VIS_LEADER_FREQ_LO      1600
#define VIS_LEADER_FREQ_HI      2100
#define VIS_LEADER_SCORE_MAX    4000
#define VIS_LEADER_SCORE_THRESH 2400
#define VIS_LEADER_RESET_LEVEL  (VIS_LEADER_SCORE_MAX / 4)
#define VIS_SUPPRESS_SAMPLES    (MIC_SR_HZ * 690 / 1000)   /* 11040 samples */

static struct {
    int16_t freq_win[DEC_FWIN];     /* sliding window for ZCR freq est. */
    int     freq_pos;

    int samples_in_row;             /* 0..SAMPLES_PER_ROW-1             */
    int sync_score;                 /* leaky integrator, 0..SYNC_SCORE_MAX */
    int sync_lockout;               /* >0 → suppress re-fire of sync lock  */

    /* Per-Y-pixel + per-chroma-pixel accumulators. */
    int y_last_pix;                 /* 0..DEC_W-1                       */
    int y_acc, y_n;
    int c_last_pix;                 /* 0..DEC_W/2-1                     */
    int c_acc, c_n;

    /* Separator-driven pair state.
     *   chroma_target:  0 → R-Y row (writes to cr[])
     *                   1 → B-Y row (writes to cb[])
     *                  -1 → not yet determined for this row
     *   sep_acc/sep_n:  freq accumulator over SEP_AVG_START..SEP_AVG_END
     *   y_prev_valid:   true once a R-Y row's Y has been stashed as
     *                   y_prev; cleared on pair emit, VIS lock, etc. */
    int  chroma_target;
    int  sep_acc, sep_n;
    bool y_prev_valid;

    /* Working line buffers — populated over a row, consumed at pair end. */
    uint8_t y_curr[DEC_W];          /* this row's Y                      */
    uint8_t y_prev[DEC_W];          /* R-Y row's Y (kept until pair emits) */
    uint8_t cr[DEC_W / 2];          /* R-Y chroma samples                 */
    uint8_t cb[DEC_W / 2];          /* B-Y chroma samples                 */

    /* VIS leader detection — see VIS_* constants above. */
    int  vis_leader_score;          /* leaky integrator, 0..VIS_LEADER_SCORE_MAX */
    bool vis_recently_fired;        /* hysteresis: score must dip below RESET before re-fire */
    int  vis_suppress_ctr;          /* >0 → ignore sync, count down       */
    bool vis_frame_armed;           /* true → next sync = line 0 of frame */

    /* Counters for the periodic decoder-status log in mic_task. */
    uint32_t pair_emit_count;       /* total pairs ever emitted */
    uint32_t sync_lock_count;       /* total sync locks ever fired */
    uint32_t vis_lock_count;        /* total VIS leader detections */
} g_dec;

/* ---------------------------------------------------------------- *
 *  PPA hardware-accelerated rotation + scale
 * ---------------------------------------------------------------- */

/*
 * ppa_rotate_cw90_into_buffer()
 *
 * Single PPA SRM op: read SRC_CROP_W × SRC_CROP_H from the sensor buffer
 * starting at (SRC_CROP_X, SRC_CROP_Y), rotate CW 90°, scale to ROT_W × ROT_H,
 * and write into the preview buffer at offset (ROT_X_OFFSET, ROT_Y_OFFSET).
 *
 * IDF v6.0 PPA rotation enum is COUNTER-CLOCKWISE, so to rotate the
 * camera image CW 90° we ask for CCW 270°.
 *
 * After 90/270° rotation the (cw, ch) crop becomes (ch, cw). PPA's
 * scale_x / scale_y are applied to the POST-rotation block dimensions:
 *     post_rot_w = SRC_CROP_H = 720
 *     post_rot_h = SRC_CROP_W = 540
 *     scale_x = ROT_W / post_rot_w = 640 / 720 ≈ 0.889
 *     scale_y = ROT_H / post_rot_h = 480 / 540 ≈ 0.889
 * (equal scale = no distortion)
 *
 * Returns ESP_OK on success.
 */
static esp_err_t ppa_rotate_cw90_into_buffer(const void *src_buf,
                                             int src_w, int src_h,
                                             void *dst_buf,
                                             size_t dst_buf_size)
{
    ppa_srm_oper_config_t op = {
        .in = {
            .buffer         = (void *)src_buf,
            .pic_w          = src_w,
            .pic_h          = src_h,
            .block_w        = SRC_CROP_W,
            .block_h        = SRC_CROP_H,
            .block_offset_x = SRC_CROP_X,
            .block_offset_y = SRC_CROP_Y,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .out = {
            .buffer         = dst_buf,
            .buffer_size    = dst_buf_size,
            .pic_w          = PREVIEW_W,
            .pic_h          = PREVIEW_H,
            .block_offset_x = ROT_X_OFFSET,
            .block_offset_y = ROT_Y_OFFSET,
            .srm_cm         = PPA_SRM_COLOR_MODE_RGB565,
        },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_270,   /* CCW 270° == CW 90° */
        .scale_x        = (float)ROT_W / (float)SRC_CROP_H,
        .scale_y        = (float)ROT_H / (float)SRC_CROP_W,
        .mirror_x       = false,
        .mirror_y       = false,
        .rgb_swap       = false,
        .byte_swap      = false,
        .mode           = PPA_TRANS_MODE_BLOCKING,
    };
    return ppa_do_scale_rotate_mirror(g_ppa, &op);
}

/*
 * crop_scale_landscape() — pure CPU center crop + nearest-neighbor
 * downscale, both landscape. Used ONCE at button press to derive the
 * 320×240 Robot36 thumbnail from the latest raw sensor frame. Not on the
 * preview hot path, so we don't bother accelerating it.
 */
static void crop_scale_landscape(const uint16_t *src, int src_w, int src_h,
                                 int cx, int cy, int cw, int ch,
                                 uint16_t *dst, int dst_w, int dst_h)
{
    (void)src_h;
    for (int dy = 0; dy < dst_h; ++dy) {
        int sy = cy + (dy * ch) / dst_h;
        const uint16_t *srow = src + sy * src_w + cx;
        uint16_t       *drow = dst + dy * dst_w;
        for (int dx = 0; dx < dst_w; ++dx) {
            int sx = (dx * cw) / dst_w;
            drow[dx] = srow[sx];
        }
    }
}

/* ---------------------------------------------------------------- *
 *  Mic + FFT + waterfall (Phase 2A — spectrum-only, no decoder yet)
 * ---------------------------------------------------------------- */

/*
 * Map a normalized intensity u ∈ [0, 1] to an RGB565 viridis-like color.
 * Used to color the waterfall by per-bin power.
 *
 *   u =   0.00  → black
 *   u =   0.25  → dark blue / purple
 *   u =   0.50  → green / cyan
 *   u =   0.75  → yellow
 *   u =   1.00  → red / white
 */
static inline uint16_t intensity_to_rgb565(float u)
{
    if (u < 0.0f) u = 0.0f;
    if (u > 1.0f) u = 1.0f;

    /* 5-stop gradient: black → indigo → cyan → yellow → red. */
    float r, g, b;
    if (u < 0.25f)       { float t = u / 0.25f;       r = 0.20f * t;        g = 0.0f;             b = 0.40f * t + 0.10f; }
    else if (u < 0.50f)  { float t = (u-0.25f)/0.25f; r = 0.20f - 0.20f*t;  g = 0.70f * t;        b = 0.50f + 0.20f*t;   }
    else if (u < 0.75f)  { float t = (u-0.50f)/0.25f; r = 0.80f * t;        g = 0.70f + 0.20f*t;  b = 0.70f - 0.70f*t;   }
    else                 { float t = (u-0.75f)/0.25f; r = 0.80f + 0.20f*t;  g = 0.90f - 0.40f*t;  b = 0.00f + 0.20f*t;   }

    uint8_t R = (uint8_t)(r * 255.0f + 0.5f);
    uint8_t G = (uint8_t)(g * 255.0f + 0.5f);
    uint8_t B = (uint8_t)(b * 255.0f + 0.5f);
    return (uint16_t)(((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3));
}

/*
 * Scroll the waterfall buffer one column to the LEFT, then write the
 * newest column on the RIGHT edge using `bin_power` (length WF_BIN_COUNT).
 *
 * bin_power is in dBFS, expected range roughly [-80, 0].
 */
static void waterfall_push_column(const float *bin_power_db)
{
    /* Map dy = 0   → highest bin (top of screen = high freq)
     *     dy = H-1 → bin 0       (bottom of screen = low freq)
     *
     * The naive implementation memmoves each row inside PSRAM (614 KB of
     * overlapping shifts), which at ~5 MB/s effective PSRAM overlap
     * bandwidth takes 100+ ms per frame and starves the FreeRTOS IDLE
     * task on CPU 1 (task WDT fires). Stage each row through internal
     * RAM (1.3 KB scratch) so the per-row PSRAM access pattern becomes
     * one sequential read + one sequential write. */
    static uint16_t row_tmp[WF_W];   /* internal RAM (~1.3 KB) */

    for (int dy = 0; dy < WF_H; ++dy) {
        uint16_t *row = g_waterfall_buf + dy * WF_W;

        /* Read the row's "kept" pixels (cols 1..WF_W-1) into internal RAM,
         * sliding them to row_tmp[0..WF_W-2]. */
        memcpy(row_tmp, row + 1, (WF_W - 1) * sizeof(uint16_t));

        /* Append the new rightmost pixel (this row's bin power). */
        int bin = ((WF_H - 1 - dy) * WF_BIN_COUNT) / WF_H;
        float dB = bin_power_db[bin];
        /* Map [-100 dB .. -20 dB] to [0..1]. Wider sensitivity than the
         * original -70..-10 so quiet room noise (typically -70 to -80 dB
         * on Tab5's mic at default gain) shows as a faint blue tint
         * instead of being clipped to pure black. */
        float u = (dB - (-100.0f)) / 80.0f;
        row_tmp[WF_W - 1] = intensity_to_rgb565(u);

        /* Bulk sequential write back to PSRAM. */
        memcpy(row, row_tmp, WF_W * sizeof(uint16_t));
    }
}

/*
 * Run one FFT over the latest 1024 mic samples and update the waterfall
 * with a new column.
 *
 * Steps:
 *   1. Window the int16 samples (Blackman-Harris) into the complex FFT
 *      buffer (real = sample * window, imag = 0).
 *   2. esp-dsp 1024-pt radix-2 FFT + bit reversal.
 *   3. For each meaningful bin i ∈ [0, FFT_N/2), compute magnitude² and
 *      convert to dB. Store into `bin_power_db[i]`.
 *   4. Push as new waterfall column.
 */
static void fft_and_render(void)
{
    /* Window + zero-imag pack. */
    for (int i = 0; i < FFT_N; ++i) {
        g_fft_data[2 * i + 0] = (float)g_mic_samples[i] * g_fft_window[i];
        g_fft_data[2 * i + 1] = 0.0f;
    }

    dsps_fft2r_fc32(g_fft_data, FFT_N);
    dsps_bit_rev_fc32(g_fft_data, FFT_N);

    /* Compute per-bin dB power. Max possible: (N/2 * 32768) for a full-scale
     * single-bin sine. log10 floor at 1.0 to avoid -inf. */
    float bin_db[WF_BIN_COUNT];
    static const float ref = (float)(FFT_N / 2) * 32768.0f;
    static const float ref2 = (float)(FFT_N / 2) * 32768.0f * (float)(FFT_N / 2) * 32768.0f;
    (void)ref;
    for (int b = 0; b < WF_BIN_COUNT && b < FFT_HALF; ++b) {
        float re = g_fft_data[2 * b + 0];
        float im = g_fft_data[2 * b + 1];
        float mag2 = re * re + im * im;
        if (mag2 < 1.0f) mag2 = 1.0f;
        /* 10*log10(mag2 / ref2) — full-scale single-bin sine is 0 dB. */
        bin_db[b] = 10.0f * log10f(mag2 / ref2);
    }

    /* Periodic diagnostic: every ~30 frames (~3 s at typical 10 fps),
     * dump raw mic min/max amplitude AND FFT min/max dB so we can
     * tell whether (a) the mic is delivering real audio and (b) the
     * FFT is producing energy worth coloring. */
    static uint32_t fft_log_ctr = 0;
    if ((++fft_log_ctr % 30) == 0) {
        int16_t s_mn = g_mic_samples[0], s_mx = g_mic_samples[0];
        for (int i = 1; i < FFT_N; ++i) {
            if (g_mic_samples[i] < s_mn) s_mn = g_mic_samples[i];
            if (g_mic_samples[i] > s_mx) s_mx = g_mic_samples[i];
        }
        float mn = bin_db[1], mx = bin_db[1];
        int   imx = 1;
        for (int b = 1; b < WF_BIN_COUNT; ++b) {
            if (bin_db[b] < mn) mn = bin_db[b];
            if (bin_db[b] > mx) { mx = bin_db[b]; imx = b; }
        }
        ESP_LOGI(TAG, "mic raw=[%d..%d]  fft=[%.1f..%.1f] dB @ bin %d (%d Hz)",
                 (int)s_mn, (int)s_mx, mn, mx, imx, imx * MIC_SR_HZ / FFT_N);
    }

    waterfall_push_column(bin_db);
}

/*
 * Robot36 RX visualizer ("raster waterfall")
 * ------------------------------------------
 *
 * For each mic sample: estimate instantaneous frequency via zero-crossing
 * count in a 16-sample sliding window, map to a 0..255 luma value via
 * Robot36's freq-to-pixel rule (1500 Hz=0, 2300 Hz=255), and paint as a
 * grayscale RGB565 pixel in the decoded image at (pix_idx, row).
 *
 * One image row = one Robot36 scanline duration (2400 samples = 150 ms).
 * No sync detection — if the timing happens to line up with an actual
 * Robot36 transmission, the picture will look correct; if it drifts you
 * get a sheared version of it; if there's no SSTV on the mic at all you
 * get noise/silence patterns. The point is that *something* is always
 * visible, so we can iterate on alignment from there.
 */

static inline int dec_estimate_freq(int16_t sample)
{
    /* Replace the oldest sample with the newest, then count sign changes
     * in the whole DEC_FWIN-sample window.
     *
     * Theory: a sinusoid at freq f Hz has 2*f zero crossings per second.
     * Over a window of N samples at sample rate Fs, expected crossings =
     * 2*f*N/Fs, so f = crossings * Fs / (2*N).
     *
     * For DEC_FWIN=32 @ Fs=16 kHz: f = crossings * 250 Hz.
     *
     * (Previously hard-coded as `crossings * 500` for the 16-sample
     * window; doubling DEC_FWIN to 32 without updating this multiplier
     * is what made every reading come back at 2× actual freq, so sync /
     * VIS / separator bands all missed during the first Tab5 run.) */
    g_dec.freq_win[g_dec.freq_pos] = sample;
    g_dec.freq_pos = (g_dec.freq_pos + 1) % DEC_FWIN;

    int crossings = 0;
    int idx_prev  = g_dec.freq_pos;            /* oldest after the write */
    int prev_sign = (g_dec.freq_win[idx_prev] >= 0);
    for (int i = 1; i < DEC_FWIN; ++i) {
        int idx = (g_dec.freq_pos + i) % DEC_FWIN;
        int sign = (g_dec.freq_win[idx] >= 0);
        if (sign != prev_sign) ++crossings;
        prev_sign = sign;
    }
    /* freq = crossings * Fs / (2 * DEC_FWIN). Computed at compile time
     * since MIC_SR_HZ and DEC_FWIN are both constants. */
    return crossings * (MIC_SR_HZ / (2 * DEC_FWIN));
}

/* Map a measured FSK frequency to an 8-bit luma/chroma sample.
 * Robot36: 1500 Hz = 0 (black), 2300 Hz = 255 (white). */
static inline uint8_t dec_freq_to_pixel(int freq)
{
    int v = ((freq - DEC_FREQ_BLACK) * 256) / (DEC_FREQ_WHITE - DEC_FREQ_BLACK);
    if (v < 0)   v = 0;
    if (v > 255) v = 255;
    return (uint8_t)v;
}

/* Convert one Y + Cr + Cb triplet (each 0..255, Cr/Cb centered at 128)
 * to RGB565 using BT.601 fixed-point coefficients. */
static inline uint16_t yuv_to_rgb565(int Y, int Cr, int Cb)
{
    int cr = Cr - 128;
    int cb = Cb - 128;
    int r = Y + ((359 * cr) >> 8);
    int g = Y - (( 88 * cb + 183 * cr) >> 8);
    int b = Y + ((454 * cb) >> 8);
    if (r < 0)   r = 0;
    if (r > 255) r = 255;
    if (g < 0)   g = 0;
    if (g > 255) g = 255;
    if (b < 0)   b = 0;
    if (b > 255) b = 255;
    return (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
}

/* End-of-row: scroll the whole image up by 2 rows and paint the new
 * pair. y_prev came from the R-Y row, y_curr from the just-completed
 * B-Y row; cr[] and cb[] hold the chroma samples for each. */
static void dec_emit_pair_at_bottom(void)
{
    memmove(g_decoded_buf,
            g_decoded_buf + 2 * DEC_W,
            (size_t)(DEC_H - 2) * DEC_W * sizeof(uint16_t));
    uint16_t *r_old = g_decoded_buf + (DEC_H - 2) * DEC_W;
    uint16_t *r_new = g_decoded_buf + (DEC_H - 1) * DEC_W;
    for (int x = 0; x < DEC_W; ++x) {
        int cx = x >> 1;
        int Cr = g_dec.cr[cx];
        int Cb = g_dec.cb[cx];
        r_old[x] = yuv_to_rgb565(g_dec.y_prev[x], Cr, Cb);
        r_new[x] = yuv_to_rgb565(g_dec.y_curr[x], Cr, Cb);
    }
}

/* Classify the chroma separator by averaging the freq readouts collected
 * during SEP_AVG_START..SEP_AVG_END.
 *
 * Per Robot36 standard:
 *   BLACK separator (1500 Hz)  → R-Y data follows  → store in cr[]  → 0
 *   WHITE separator (2300 Hz)  → B-Y data follows  → store in cb[]  → 1
 *
 * Returns -1 if no samples were collected (we tuned in past the
 * separator window). */
static inline int dec_classify_separator(void)
{
    if (g_dec.sep_n == 0) return -1;
    int avg = g_dec.sep_acc / g_dec.sep_n;
    return (avg > SEP_FREQ_THRESH) ? 1 : 0;
}

/* Feed one PCM sample. */
static void dec_feed_sample(int16_t sample)
{
    int freq = dec_estimate_freq(sample);

    /* ---------------------------------------------------------------- *
     *  VIS leader detection (frame-start lock)
     *
     *  Sliding-window ZCR is jittery — readings flip between adjacent
     *  crossing counts as the window slides through phase, so a strict
     *  "consecutive samples in band" counter never reaches a sane
     *  threshold. Instead we use a leaky integrator score: +1 in band,
     *  −1 out, capped at VIS_LEADER_SCORE_MAX. With ~95% in-band on a
     *  real leader the score climbs ~+0.9/sample and crosses
     *  VIS_LEADER_SCORE_THRESH well within the 300 ms leader.
     *
     *  Hysteresis (vis_recently_fired): after firing once, we wait for
     *  the score to drain back below VIS_LEADER_RESET_LEVEL before
     *  arming the detector again. This prevents the same leader (or a
     *  chroma porch run) from re-firing while we're still in the
     *  suppress window.
     * ---------------------------------------------------------------- */
    if (freq >= VIS_LEADER_FREQ_LO && freq <= VIS_LEADER_FREQ_HI) {
        if (g_dec.vis_leader_score < VIS_LEADER_SCORE_MAX) g_dec.vis_leader_score += 1;
    } else {
        if (g_dec.vis_leader_score > 0) g_dec.vis_leader_score -= 1;
    }
    if (!g_dec.vis_recently_fired &&
        g_dec.vis_leader_score >= VIS_LEADER_SCORE_THRESH) {
        g_dec.vis_suppress_ctr   = VIS_SUPPRESS_SAMPLES;
        g_dec.vis_frame_armed    = true;
        g_dec.vis_recently_fired = true;
        g_dec.sync_score         = 0;
        g_dec.sync_lockout       = 0;
        g_dec.y_prev_valid       = false;
        g_dec.vis_lock_count    += 1;
    } else if (g_dec.vis_leader_score < VIS_LEADER_RESET_LEVEL) {
        g_dec.vis_recently_fired = false;
    }

    if (g_dec.vis_suppress_ctr > 0) {
        g_dec.vis_suppress_ctr -= 1;
        g_dec.sync_score = 0;
        return;
    }

    /* ---------------------------------------------------------------- *
     *  1200 Hz line-sync lock — score-based, same as VIS.
     *
     *  Band 950..1300 catches BOTH 1000 Hz and 1250 Hz ZCR readings of
     *  a 1200 Hz tone (which alternate as the window slides). ~95%
     *  in-band during a real sync pulse → score climbs ~+0.9/sample →
     *  crosses SYNC_SCORE_THRESH (80) in ~89 samples = 5.6 ms,
     *  comfortably inside the 9 ms sync.
     *
     *  After firing we lock out re-detection for SYNC_LOCKOUT_SAMPLES
     *  (~140 ms = the rest of the row) so the score climbing during
     *  porch/Y/chroma noise can't relock mid-row.
     *
     *  Pair state (chroma_target / y_prev_valid) is NOT reset here —
     *  the chroma separator detector handles parity per-row so the
     *  decoder self-aligns regardless of where we tuned in. Only
     *  vis_frame_armed forces y_prev_valid=false (fresh frame).
     * ---------------------------------------------------------------- */
    if (freq > SYNC_FREQ_LO && freq < SYNC_FREQ_HI) {
        if (g_dec.sync_score < SYNC_SCORE_MAX) g_dec.sync_score += 1;
    } else {
        if (g_dec.sync_score > 0) g_dec.sync_score -= 1;
    }

    if (g_dec.sync_lockout > 0) {
        g_dec.sync_lockout -= 1;
    } else if (g_dec.sync_score >= SYNC_SCORE_THRESH) {
        /* Lock! Snap row offset to "we're 5.6 ms into the sync" — close
         * enough to the SYNC_SCORE_THRESH value in samples. */
        g_dec.samples_in_row = SYNC_SCORE_THRESH;
        g_dec.sync_lockout = SYNC_LOCKOUT_SAMPLES;
        g_dec.y_last_pix = g_dec.y_acc = g_dec.y_n = 0;
        g_dec.c_last_pix = g_dec.c_acc = g_dec.c_n = 0;
        g_dec.sep_acc = g_dec.sep_n = 0;
        g_dec.chroma_target = -1;
        g_dec.sync_lock_count += 1;
        if (g_dec.vis_frame_armed) {
            g_dec.vis_frame_armed = false;
            g_dec.y_prev_valid    = false;
        }
    }

    int s = g_dec.samples_in_row;

    /* === Y luma region (samples 192..1600 inside the row) ============ */
    if (s >= Y_START_SAMP && s < Y_END_SAMP) {
        int relpos = s - Y_START_SAMP;
        int pix    = (relpos * DEC_W) / Y_LEN_SAMP;          /* 0..319 */
        if (pix != g_dec.y_last_pix) {
            if (g_dec.y_n > 0 && g_dec.y_last_pix < DEC_W) {
                int avg = g_dec.y_acc / g_dec.y_n;
                g_dec.y_curr[g_dec.y_last_pix] = dec_freq_to_pixel(avg);
            }
            g_dec.y_last_pix = pix;
            g_dec.y_acc      = 0;
            g_dec.y_n        = 0;
        }
        g_dec.y_acc += freq;
        g_dec.y_n   += 1;
    }
    /* === Separator averaging window (mid of 1600..1672) ============== */
    else if (s >= SEP_AVG_START && s < SEP_AVG_END) {
        g_dec.sep_acc += freq;
        g_dec.sep_n   += 1;
    }
    /* === Chroma region (samples 1696..2400) ========================== */
    else if (s >= C_START_SAMP && s < C_END_SAMP) {
        if (s == C_START_SAMP) {
            /* First sample of the chroma region — lock in this row's
             * parity from the separator polarity we just measured. */
            g_dec.chroma_target = dec_classify_separator();
        }
        int relpos = s - C_START_SAMP;
        int pix    = (relpos * (DEC_W / 2)) / C_LEN_SAMP;    /* 0..159 */
        if (pix != g_dec.c_last_pix) {
            if (g_dec.c_n > 0 && g_dec.c_last_pix < DEC_W / 2) {
                int avg = g_dec.c_acc / g_dec.c_n;
                uint8_t v = dec_freq_to_pixel(avg);
                if (g_dec.chroma_target == 0) {
                    g_dec.cr[g_dec.c_last_pix] = v;
                } else if (g_dec.chroma_target == 1) {
                    g_dec.cb[g_dec.c_last_pix] = v;
                }
            }
            g_dec.c_last_pix = pix;
            g_dec.c_acc      = 0;
            g_dec.c_n        = 0;
        }
        g_dec.c_acc += freq;
        g_dec.c_n   += 1;
    }

    g_dec.samples_in_row += 1;

    /* === End of row =================================================== */
    if (g_dec.samples_in_row >= SAMPLES_PER_ROW) {
        /* Flush the last Y and chroma pixel accumulators. */
        if (g_dec.y_n > 0 && g_dec.y_last_pix < DEC_W) {
            g_dec.y_curr[g_dec.y_last_pix] = dec_freq_to_pixel(g_dec.y_acc / g_dec.y_n);
        }
        if (g_dec.c_n > 0 && g_dec.c_last_pix < DEC_W / 2) {
            uint8_t v = dec_freq_to_pixel(g_dec.c_acc / g_dec.c_n);
            if (g_dec.chroma_target == 0) {
                g_dec.cr[g_dec.c_last_pix] = v;
            } else if (g_dec.chroma_target == 1) {
                g_dec.cb[g_dec.c_last_pix] = v;
            }
        }

        /* Pair handling driven by the separator decision:
         *  • R-Y row (chroma_target=0): stash y_curr as y_prev, mark valid
         *  • B-Y row (chroma_target=1) with valid y_prev: emit the pair
         *  • undetermined (-1, e.g. we tuned in past the separator):
         *    silently drop the row */
        if (g_dec.chroma_target == 0) {
            memcpy(g_dec.y_prev, g_dec.y_curr, DEC_W);
            g_dec.y_prev_valid = true;
        } else if (g_dec.chroma_target == 1 && g_dec.y_prev_valid) {
            dec_emit_pair_at_bottom();
            g_dec.y_prev_valid = false;
            g_dec.pair_emit_count += 1;
        }

        /* Reset per-row accumulators. */
        g_dec.samples_in_row = 0;
        g_dec.y_last_pix = g_dec.y_acc = g_dec.y_n = 0;
        g_dec.c_last_pix = g_dec.c_acc = g_dec.c_n = 0;
        g_dec.sep_acc = g_dec.sep_n = 0;
        g_dec.chroma_target = -1;
    }
}

/* Feed a buffer of samples to the decoder (called from mic_task). */
static void dec_feed_buffer(const int16_t *samples, size_t n)
{
    for (size_t i = 0; i < n; ++i) {
        dec_feed_sample(samples[i]);
    }
}

static void mic_task(void *arg)
{
    (void)arg;

    g_mic = bsp_audio_codec_microphone_init();
    if (!g_mic) {
        ESP_LOGE(TAG, "mic init failed — waterfall will stay blank");
        vTaskDelete(NULL);
        return;
    }

    esp_codec_dev_sample_info_t fmt = {
        .sample_rate     = MIC_SR_HZ,
        .channel         = 1,
        .channel_mask    = 0,
        .bits_per_sample = 16,
    };
    esp_err_t r = esp_codec_dev_open(g_mic, &fmt);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "mic open: %s", esp_err_to_name(r));
        vTaskDelete(NULL);
        return;
    }
    /* ES7210 input gain — 26 dB is a sane default for room-level audio. */
    esp_codec_dev_set_in_gain(g_mic, 26.0f);
    ESP_LOGI(TAG, "✓ mic streaming %d Hz mono 16-bit", MIC_SR_HZ);

    /* WDT diagnostic: time the FIRST read and dump first few samples.
     * If the read hangs (codec/I²S misconfigured) we never reach the
     * post-call log; the watchdog will catch it. */
    ESP_LOGI(TAG, "mic: calling first esp_codec_dev_read (FFT_N=%d samples)", FFT_N);
    TickType_t t_before = xTaskGetTickCount();
    esp_err_t first_r = esp_codec_dev_read(g_mic, g_mic_samples, FFT_N * sizeof(int16_t));
    TickType_t t_after = xTaskGetTickCount();
    ESP_LOGI(TAG, "mic: first read r=%d in %u ms; samples[0..3]=%d,%d,%d,%d",
             (int)first_r, (unsigned)((t_after - t_before) * portTICK_PERIOD_MS),
             g_mic_samples[0], g_mic_samples[1], g_mic_samples[2], g_mic_samples[3]);

    while (1) {
        static uint32_t loop_iter = 0;
        loop_iter++;
        if (loop_iter <= 10 || (loop_iter % 100) == 0) {
            ESP_LOGI(TAG, "mic loop iter %u", (unsigned)loop_iter);
        }

        /* No mic pause during TX: ES8388 (out) and ES7210 (in) use
         * independent TX/RX channels of the same I²S peripheral, so the
         * speaker write and mic read don't conflict at the hardware
         * level. Leaving the mic running during TX lets the decoder
         * pick up the Tab5's own SSTV signal end-to-end. */

        size_t bytes = FFT_N * sizeof(int16_t);
        r = esp_codec_dev_read(g_mic, g_mic_samples, bytes);
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "mic read: %s", esp_err_to_name(r));
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        fft_and_render();

        /* Run the Robot36 RX decoder over the same block of samples. */
        dec_feed_buffer(g_mic_samples, FFT_N);

        /* Decoder status snapshot every ~30 iters (~2.5 s). Helpful
         * when bringing up sync — without this, a stuck score-based
         * detector looks identical to "decoder doing nothing". */
        if ((loop_iter % 30) == 0) {
            ESP_LOGI(TAG, "dec: sync_score=%d/%d (locks=%u) vis_score=%d/%d (locks=%u) "
                          "suppress=%d row=%d/%d chroma_target=%d y_prev_valid=%d pairs=%u",
                     g_dec.sync_score, SYNC_SCORE_THRESH,
                     (unsigned)g_dec.sync_lock_count,
                     g_dec.vis_leader_score, VIS_LEADER_SCORE_THRESH,
                     (unsigned)g_dec.vis_lock_count,
                     g_dec.vis_suppress_ctr,
                     g_dec.samples_in_row, SAMPLES_PER_ROW,
                     g_dec.chroma_target, (int)g_dec.y_prev_valid,
                     (unsigned)g_dec.pair_emit_count);

            /* Chroma + luma buffer stats: snapshot of the last emitted
             * pair's data. Neutral chroma should average ~128. If both
             * cr_avg and cb_avg are well below 128, we're decoding a
             * frequency lower than the encoder transmitted → ZCR
             * estimator bias or window misalignment. */
            int cr_sum = 0, cb_sum = 0, y_sum = 0;
            int cr_min = 255, cr_max = 0, cb_min = 255, cb_max = 0;
            for (int i = 0; i < DEC_W / 2; i++) {
                cr_sum += g_dec.cr[i];
                cb_sum += g_dec.cb[i];
                if (g_dec.cr[i] < cr_min) cr_min = g_dec.cr[i];
                if (g_dec.cr[i] > cr_max) cr_max = g_dec.cr[i];
                if (g_dec.cb[i] < cb_min) cb_min = g_dec.cb[i];
                if (g_dec.cb[i] > cb_max) cb_max = g_dec.cb[i];
            }
            for (int i = 0; i < DEC_W; i++) y_sum += g_dec.y_curr[i];
            ESP_LOGI(TAG, "dec: y_avg=%d cr=[%d..%d avg=%d] cb=[%d..%d avg=%d]",
                     y_sum / DEC_W,
                     cr_min, cr_max, cr_sum / (DEC_W / 2),
                     cb_min, cb_max, cb_sum / (DEC_W / 2));
        }

        /* No direct LVGL call from this task — would race against the
         * LVGL task on CPU 0 (we don't hold the LVGL mutex here).
         * preview_tick() inside the LVGL task invalidates the waterfall
         * widget at 30 Hz, which picks up the buffer we just wrote. */

        /* Explicit yield. esp_codec_dev_read often returns much faster
         * than the natural 64 ms data cadence (it pulls whatever's
         * already in DMA), so a hard yield is needed to keep IDLE1
         * alive and let cam_task / LVGL run unimpeded. */
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

/* ---------------------------------------------------------------- *
 *  Camera bring-up
 * ---------------------------------------------------------------- */

static esp_err_t cam_init(void)
{
    /* Step 0: register a PPA SRM client for hardware rotation. */
    ppa_client_config_t ppa_cfg = {
        .oper_type             = PPA_OPERATION_SRM,
        .max_pending_trans_num = 1,
    };
    esp_err_t r = ppa_register_client(&ppa_cfg, &g_ppa);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "ppa_register_client: %s", esp_err_to_name(r));
        return r;
    }
    ESP_LOGI(TAG, "PPA SRM client ready");

    /* Step 1: BSP brings up CSI + ISP + sensor (auto-detect SC202CS). */
    bsp_camera_cfg_t bsp_cam_cfg = {0};
    r = bsp_camera_start(&bsp_cam_cfg);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "bsp_camera_start: %s", esp_err_to_name(r));
        return r;
    }
    ESP_LOGI(TAG, "bsp_camera_start OK");

    /* Step 2: Open V4L2 device. */
    g_cam_fd = open(ESP_VIDEO_MIPI_CSI_DEVICE_NAME, O_RDONLY);
    if (g_cam_fd < 0) {
        ESP_LOGE(TAG, "open %s: %s", ESP_VIDEO_MIPI_CSI_DEVICE_NAME, strerror(errno));
        return ESP_FAIL;
    }

    struct v4l2_capability cap = {0};
    if (ioctl(g_cam_fd, VIDIOC_QUERYCAP, &cap) == 0) {
        ESP_LOGI(TAG, "cam: driver=%s card=%s", cap.driver, cap.card);
    }

    /* Step 3: Get default format, then set RGB565 keeping native resolution. */
    struct v4l2_format fmt = { .type = V4L2_BUF_TYPE_VIDEO_CAPTURE };
    if (ioctl(g_cam_fd, VIDIOC_G_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "G_FMT: %s", strerror(errno));
        goto fail;
    }
    ESP_LOGI(TAG, "cam default: %ux%u pixfmt=%c%c%c%c",
             fmt.fmt.pix.width, fmt.fmt.pix.height,
             (char)(fmt.fmt.pix.pixelformat),
             (char)(fmt.fmt.pix.pixelformat >> 8),
             (char)(fmt.fmt.pix.pixelformat >> 16),
             (char)(fmt.fmt.pix.pixelformat >> 24));

    g_cam_width  = fmt.fmt.pix.width;
    g_cam_height = fmt.fmt.pix.height;

    if (fmt.fmt.pix.pixelformat != CAM_PIX_FMT) {
        fmt.fmt.pix.pixelformat = CAM_PIX_FMT;
        if (ioctl(g_cam_fd, VIDIOC_S_FMT, &fmt) != 0) {
            ESP_LOGE(TAG, "S_FMT RGB565: %s", strerror(errno));
            goto fail;
        }
        g_cam_width  = fmt.fmt.pix.width;
        g_cam_height = fmt.fmt.pix.height;
        ESP_LOGI(TAG, "cam set: %ux%u RGB565 (sizeimage=%u)",
                 g_cam_width, g_cam_height, fmt.fmt.pix.sizeimage);
    }

    /* Step 4: Request mmap buffers. */
    struct v4l2_requestbuffers req = {
        .count  = CAM_BUFFER_COUNT,
        .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (ioctl(g_cam_fd, VIDIOC_REQBUFS, &req) != 0) {
        ESP_LOGE(TAG, "REQBUFS: %s", strerror(errno));
        goto fail;
    }

    for (int i = 0; i < CAM_BUFFER_COUNT; ++i) {
        struct v4l2_buffer buf = {
            .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
            .index  = i,
        };
        if (ioctl(g_cam_fd, VIDIOC_QUERYBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QUERYBUF[%d]: %s", i, strerror(errno));
            goto fail;
        }
        g_cam_bufs[i]    = mmap(NULL, buf.length, PROT_READ | PROT_WRITE,
                                 MAP_SHARED, g_cam_fd, buf.m.offset);
        g_cam_buf_len[i] = buf.length;
        if (!g_cam_bufs[i] || g_cam_bufs[i] == MAP_FAILED) {
            ESP_LOGE(TAG, "mmap[%d] failed", i);
            goto fail;
        }
        if (ioctl(g_cam_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QBUF[%d]: %s", i, strerror(errno));
            goto fail;
        }
    }

    /* Step 5: STREAMON. */
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl(g_cam_fd, VIDIOC_STREAMON, &type) != 0) {
        ESP_LOGE(TAG, "STREAMON: %s", strerror(errno));
        goto fail;
    }
    ESP_LOGI(TAG, "✓ camera streaming %ux%u RGB565 from /dev/video0",
             g_cam_width, g_cam_height);
    return ESP_OK;

fail:
    if (g_cam_fd >= 0) { close(g_cam_fd); g_cam_fd = -1; }
    return ESP_FAIL;
}

/* Background task: DQBUF → scale to preview → QBUF, forever. */
static void cam_task(void *arg)
{
    (void)arg;
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    while (1) {
        struct v4l2_buffer buf = {
            .type   = V4L2_BUF_TYPE_VIDEO_CAPTURE,
            .memory = V4L2_MEMORY_MMAP,
        };
        if (ioctl(g_cam_fd, VIDIOC_DQBUF, &buf) != 0) {
            ESP_LOGE(TAG, "DQBUF: %s", strerror(errno));
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }
        const uint16_t *src = (uint16_t *)g_cam_bufs[buf.index];

        /* Mutex protects g_preview_buf (LVGL reads from it during repaint)
         * and g_image_buf (the demo_task button handler reads from it
         * during a TX press). Both are PSRAM-backed. */
        if (xSemaphoreTake(g_preview_mutex, pdMS_TO_TICKS(50))) {
            /* Hardware rotate+scale into the center strip of the landscape
             * preview buffer. The black pillarbox bars were initialized
             * once in app_run() and never overwritten by PPA (block_offset
             * keeps PPA's writes inside the ROT_W × ROT_H window). */
            esp_err_t pr = ppa_rotate_cw90_into_buffer(
                src, g_cam_width, g_cam_height,
                g_preview_buf, PREVIEW_W * PREVIEW_H * 2);
            if (pr != ESP_OK) {
                ESP_LOGW(TAG, "PPA SRM failed: %s", esp_err_to_name(pr));
            }
            /* SSTV thumbnail (g_image_buf, 320×240) is now derived from
             * the preview at button-press time — same crop, same rotation
             * the user sees. Done lazily so cam_task stays minimal. */

            g_have_frame = true;
            xSemaphoreGive(g_preview_mutex);
        }

        if (ioctl(g_cam_fd, VIDIOC_QBUF, &buf) != 0) {
            ESP_LOGE(TAG, "QBUF: %s", strerror(errno));
        }
        /* No artificial delay — VIDIOC_DQBUF above blocks until the next
         * sensor frame is ready (sensor runs at 30 fps), which paces the
         * loop naturally. */
    }
}

/* LVGL timer: ~30 ms; invalidates preview AND waterfall so they repaint.
 *
 * Important: this runs INSIDE the LVGL task (which holds the LVGL
 * mutex), so it is the safe place to call lv_obj_invalidate. Tasks on
 * other cores (e.g. mic_task on CPU 1) must NOT call LVGL APIs without
 * bsp_display_lock() — doing so was deadlocking the LVGL task and
 * starving IDLE1 / tripping the task watchdog. */
static void preview_tick(lv_timer_t *t)
{
    (void)t;
    if (g_have_frame && g_preview_img) {
        lv_obj_invalidate(g_preview_img);
    }
    if (g_waterfall_img) {
        lv_obj_invalidate(g_waterfall_img);
    }
    if (g_decoded_img) {
        lv_obj_invalidate(g_decoded_img);
    }
}

/* ---------------------------------------------------------------- *
 *  Status updates (LVGL widgets must be touched under bsp lock)
 * ---------------------------------------------------------------- */

static void status_set(const char *msg)
{
    if (bsp_display_lock(200)) {
        lv_label_set_text(g_status_label, msg);
        bsp_display_unlock();
    }
}

/* ---------------------------------------------------------------- *
 *  Audio output
 * ---------------------------------------------------------------- */

static esp_err_t audio_open(void)
{
    /* Mic & speaker stay open simultaneously: I²S has independent TX/RX
     * channels so the decoder can keep listening (and even decode the
     * Tab5's own SSTV output). */
    if (!g_spk) {
        g_spk = bsp_audio_codec_speaker_init();
        if (!g_spk) return ESP_FAIL;
    }
    esp_codec_dev_sample_info_t fmt = {
        .sample_rate     = SAMPLE_RATE_HZ,
        .channel         = 1,
        .channel_mask    = 0,
        .bits_per_sample = 16,
    };
    esp_err_t r = esp_codec_dev_open(g_spk, &fmt);
    if (r != ESP_OK) return r;
    /* 30% is the sweet spot on Tab5's 1W speaker + PAM amp — loud
     * enough for a phone decoder to lock at ~1 m, soft enough that the
     * FM tones stay undistorted (40%+ starts to clip). */
    esp_codec_dev_set_out_vol(g_spk, 30);
    return ESP_OK;
}

/*
 * Play PCM in ~200 ms chunks so the progress bar can be updated between
 * writes. Each chunk = SAMPLE_RATE_HZ / 5 samples (3200 @ 16 kHz).
 * esp_codec_dev_write is blocking, so by the time it returns those
 * samples have been DMA'd into the codec; using fixed-size chunks lets
 * us extrapolate progress accurately.
 */
static void audio_play(const int16_t *pcm, size_t n_samples)
{
    if (!g_spk) return;
    const size_t chunk = SAMPLE_RATE_HZ / 5;     /* 200 ms */
    size_t played = 0;
    int last_pct = -1;
    while (played < n_samples) {
        size_t this_chunk = (n_samples - played < chunk) ? (n_samples - played) : chunk;
        esp_codec_dev_write(g_spk, (void *)(pcm + played), this_chunk * sizeof(int16_t));
        played += this_chunk;

        int pct = (int)((100ULL * played) / n_samples);
        if (pct != last_pct) {
            last_pct = pct;
            if (g_tx_bar && bsp_display_lock(20)) {
                lv_bar_set_value(g_tx_bar, pct, LV_ANIM_OFF);
                bsp_display_unlock();
            }
        }
    }
}

static void audio_close(void)
{
    if (g_spk) esp_codec_dev_close(g_spk);
}

/* ---------------------------------------------------------------- *
 *  Demo task — runs on button press
 * ---------------------------------------------------------------- */

/* ---------------------------------------------------------------- *
 *  Test pattern (HSV color wheel) — used when camera is unavailable
 * ---------------------------------------------------------------- */
static void make_test_pattern(uint16_t *rgb565)
{
    for (int y = 0; y < SSTV_ROBOT36_HEIGHT; ++y) {
        float hue = (float)y / SSTV_ROBOT36_HEIGHT * 6.0f;
        int   hi  = (int)hue;
        float f   = hue - hi;
        for (int x = 0; x < SSTV_ROBOT36_WIDTH; ++x) {
            float v = (float)x / SSTV_ROBOT36_WIDTH;
            float r = 0, g = 0, b = 0;
            switch (hi) {
                case 0: r = 1;     g = f;     b = 0;     break;
                case 1: r = 1 - f; g = 1;     b = 0;     break;
                case 2: r = 0;     g = 1;     b = f;     break;
                case 3: r = 0;     g = 1 - f; b = 1;     break;
                case 4: r = f;     g = 0;     b = 1;     break;
                default:r = 1;     g = 0;     b = 1 - f; break;
            }
            uint8_t R = (uint8_t)(r * v * 255);
            uint8_t G = (uint8_t)(g * v * 255);
            uint8_t B = (uint8_t)(b * v * 255);
            rgb565[y * SSTV_ROBOT36_WIDTH + x] =
                (uint16_t)(((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3));
        }
    }
}

static void demo_task(void *arg)
{
    (void)arg;

    /* 1. g_image_buf was populated by the Capture button (or, if the
     *    user pressed Send without capturing AND there's no camera,
     *    by make_test_pattern). Nothing to do here — just announce. */
    status_set("Encoding snapshot for SSTV TX...");

    /* 2. Encode Robot36. */
    status_set("Encoding Robot36 (~36 s of audio)...");
    size_t nsamp = 0;
    esp_err_t r = sstv_robot36_encode(g_image_buf, SAMPLE_RATE_HZ,
                                      g_pcm_buf, PCM_CAPACITY, &nsamp);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "encode: %s", esp_err_to_name(r));
        status_set("Encode failed!");
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "encoded %u samples (%.2f s @ %u Hz)",
             (unsigned) nsamp, (double) nsamp / SAMPLE_RATE_HZ, SAMPLE_RATE_HZ);

    /* 3. Play through ES8388 → onboard 1W speaker. */
    status_set("Transmitting Robot36 to speaker...");
    if (audio_open() == ESP_OK) {
        /* Reveal + reset the progress bar. */
        if (g_tx_bar && bsp_display_lock(100)) {
            lv_bar_set_value(g_tx_bar, 0, LV_ANIM_OFF);
            lv_obj_clear_flag(g_tx_bar, LV_OBJ_FLAG_HIDDEN);
            bsp_display_unlock();
        }
        audio_play(g_pcm_buf, nsamp);
        audio_close();
        /* Hide the bar after TX completes. */
        if (g_tx_bar && bsp_display_lock(100)) {
            lv_obj_add_flag(g_tx_bar, LV_OBJ_FLAG_HIDDEN);
            bsp_display_unlock();
        }
    } else {
        status_set("Audio open failed!");
        vTaskDelete(NULL);
        return;
    }
    status_set("Done. Press to send again.");
    vTaskDelete(NULL);
}

/* ---------------------------------------------------------------- *
 *  Button event
 * ---------------------------------------------------------------- */

/*
 * Capture button — snapshot the current preview into g_image_buf.
 *
 * g_image_buf is our "single-frame storage": one 320×240 RGB565 buffer
 * (same resolution as Robot36's wire format), filled by a fast 2× nearest-
 * neighbor downscale from the live preview. The snapshot persists until
 * the user captures again or reboots, so they can re-send the same frame
 * multiple times.
 */
static void on_capture_press(lv_event_t *e)
{
    (void)e;
    if (!g_have_frame) {
        status_set("No camera frame yet — preview is dark");
        return;
    }
    if (xSemaphoreTake(g_preview_mutex, pdMS_TO_TICKS(500))) {
        crop_scale_landscape(
            g_preview_buf, PREVIEW_W, PREVIEW_H,
            0, 0, PREVIEW_W, PREVIEW_H,
            g_image_buf, SSTV_ROBOT36_WIDTH, SSTV_ROBOT36_HEIGHT);
        g_have_snapshot = true;
        xSemaphoreGive(g_preview_mutex);
        status_set("Snapshot ready — tap Send to TX");
        ESP_LOGI(TAG, "snapshot captured (320x240 RGB565)");
    } else {
        status_set("Capture failed (preview busy)");
    }
}

/*
 * Send button — encode + transmit the captured snapshot via Robot36.
 *
 * Requires g_have_snapshot=true. If the user taps Send before Capture,
 * we refuse cleanly with a status message rather than transmit whatever
 * uninitialized garbage is in g_image_buf.
 */
static void on_send_press(lv_event_t *e)
{
    (void)e;
    if (!g_have_snapshot) {
        status_set("No snapshot — tap Capture first");
        return;
    }
    ESP_LOGI(TAG, "Send button → spawn demo task");
    xTaskCreate(demo_task, "sstv_demo", 8192, NULL, 5, NULL);
}

/* ---------------------------------------------------------------- *
 *  UI construction
 * ---------------------------------------------------------------- */

static void build_ui(void)
{
    /*
     * Layout on the 720×1280 portrait screen, reserving the bottom half
     * (y > 640) for the upcoming SSTV-RX waterfall:
     *
     *   y =   0..  40   Title bar
     *   y =  60.. 540   Preview (640W × 480H landscape, centered horiz.)
     *   y = 560.. 640   TX SSTV button (240W × 70H)
     *   y = 640..1200   [reserved for waterfall — added next phase]
     *   y =1240..1270   Status text
     */
    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_hex(0x101820), 0);

    /* Title */
    lv_obj_t *title = lv_label_create(scr);
    lv_label_set_text(title, "CubeSat Imager — Tab5 SSTV demo");
    lv_obj_set_style_text_color(title, lv_color_hex(0xeaeaea), 0);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_24, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 10);

    /* Live preview image — top of screen, leaves the bottom half free for
     * the future RX waterfall. */
    g_preview_dsc.header.cf       = LV_COLOR_FORMAT_RGB565;
    g_preview_dsc.header.w        = PREVIEW_W;
    g_preview_dsc.header.h        = PREVIEW_H;
    g_preview_dsc.data_size       = PREVIEW_W * PREVIEW_H * 2;
    g_preview_dsc.data            = (const uint8_t *) g_preview_buf;

    g_preview_img = lv_image_create(scr);
    lv_image_set_src(g_preview_img, &g_preview_dsc);
    lv_obj_align(g_preview_img, LV_ALIGN_TOP_MID, 0, 60);
    lv_obj_set_style_border_color(g_preview_img, lv_color_hex(0x5cc8ff), 0);
    lv_obj_set_style_border_width(g_preview_img, 2, 0);
    lv_obj_set_style_radius(g_preview_img, 0, 0);
    /* lv_image has nonzero default content padding which leaves a
     * visible screen-bg gap on the right/bottom of the image. Zero out
     * pad_all so the image touches the border on every side. */
    lv_obj_set_style_pad_all(g_preview_img, 0, 0);

    /* Slim TX progress bar — sits in the 15 px gap between preview and
     * button. Hidden until the user taps the button; demo_task makes it
     * visible and ticks it as the SSTV waveform plays out. */
    g_tx_bar = lv_bar_create(scr);
    lv_obj_set_size(g_tx_bar, 640, 6);
    lv_obj_align(g_tx_bar, LV_ALIGN_TOP_MID, 0, 545);
    lv_bar_set_range(g_tx_bar, 0, 100);
    lv_bar_set_value(g_tx_bar, 0, LV_ANIM_OFF);
    lv_obj_set_style_bg_color(g_tx_bar, lv_color_hex(0x2a3340), LV_PART_MAIN);
    lv_obj_set_style_bg_color(g_tx_bar, lv_color_hex(0xff8a2b), LV_PART_INDICATOR);
    lv_obj_set_style_radius(g_tx_bar, 3, LV_PART_MAIN);
    lv_obj_set_style_radius(g_tx_bar, 3, LV_PART_INDICATOR);
    lv_obj_add_flag(g_tx_bar, LV_OBJ_FLAG_HIDDEN);

    /* Two buttons side-by-side, each 300W × 70H with a 40 px gap,
     * total 640 (matches preview width).
     *
     * LEFT: Capture — snapshots preview into g_image_buf.
     * RIGHT: Send SSTV — transmits the snapshot via Robot36. */
    g_capture_btn = lv_btn_create(scr);
    lv_obj_set_size(g_capture_btn, 300, 70);
    lv_obj_align(g_capture_btn, LV_ALIGN_TOP_MID, -170, 555);
    lv_obj_set_style_bg_color(g_capture_btn, lv_color_hex(0x5cc8ff), 0);   /* cyan */
    lv_obj_set_style_radius(g_capture_btn, 14, 0);
    lv_obj_add_event_cb(g_capture_btn, on_capture_press, LV_EVENT_CLICKED, NULL);
    lv_obj_t *cap_label = lv_label_create(g_capture_btn);
    lv_label_set_text(cap_label, LV_SYMBOL_IMAGE "  Capture");
    lv_obj_set_style_text_font(cap_label, &lv_font_montserrat_24, 0);
    lv_obj_center(cap_label);

    g_button = lv_btn_create(scr);
    lv_obj_set_size(g_button, 300, 70);
    lv_obj_align(g_button, LV_ALIGN_TOP_MID, 170, 555);
    lv_obj_set_style_bg_color(g_button, lv_color_hex(0xff8a2b), 0);        /* orange */
    lv_obj_set_style_radius(g_button, 14, 0);
    lv_obj_add_event_cb(g_button, on_send_press, LV_EVENT_CLICKED, NULL);
    lv_obj_t *btn_label = lv_label_create(g_button);
    lv_label_set_text(btn_label, LV_SYMBOL_AUDIO "  Send SSTV");
    lv_obj_set_style_text_font(btn_label, &lv_font_montserrat_24, 0);
    lv_obj_center(btn_label);

    /* SSTV-RX waterfall — bottom half of the screen.
     *
     * Width 640 (matches preview), height 480 (same aspect = symmetric
     * layout). x axis = time (newest column at the right), y axis =
     * frequency (low at bottom, high at top), color = power per bin. */
    g_waterfall_dsc.header.cf       = LV_COLOR_FORMAT_RGB565;
    g_waterfall_dsc.header.w        = WF_W;
    g_waterfall_dsc.header.h        = WF_H;
    g_waterfall_dsc.data_size       = WF_W * WF_H * 2;
    g_waterfall_dsc.data            = (const uint8_t *) g_waterfall_buf;

    g_waterfall_img = lv_image_create(scr);
    lv_image_set_src(g_waterfall_img, &g_waterfall_dsc);
    lv_obj_align(g_waterfall_img, LV_ALIGN_TOP_MID, 0, 650);
    lv_obj_set_style_border_color(g_waterfall_img, lv_color_hex(0xff8a2b), 0);
    lv_obj_set_style_border_width(g_waterfall_img, 2, 0);
    lv_obj_set_style_radius(g_waterfall_img, 0, 0);
    lv_obj_set_style_pad_all(g_waterfall_img, 0, 0);

    /* Decoded SSTV image — below the waterfall. Native Robot36 wire
     * format is 320×240 (4:3); we display it at 640×480 on screen by
     * (a) setting the widget size explicitly, and (b) using
     * LV_IMAGE_ALIGN_STRETCH so LVGL stretches the 320×240 source
     * uniformly to fill the widget — proper 4:3 aspect, no overflow
     * past the bounding box. */
    g_decoded_dsc.header.cf       = LV_COLOR_FORMAT_RGB565;
    g_decoded_dsc.header.w        = DEC_W;
    g_decoded_dsc.header.h        = DEC_H;
    g_decoded_dsc.data_size       = DEC_W * DEC_H * 2;
    g_decoded_dsc.data            = (const uint8_t *) g_decoded_buf;

    g_decoded_img = lv_image_create(scr);
    lv_image_set_src(g_decoded_img, &g_decoded_dsc);
    lv_obj_set_size(g_decoded_img, 640, 440);                 /* ~4:3 */
    lv_image_set_inner_align(g_decoded_img, LV_IMAGE_ALIGN_STRETCH);
    lv_obj_align(g_decoded_img, LV_ALIGN_TOP_MID, 0, 800);
    lv_obj_set_style_border_color(g_decoded_img, lv_color_hex(0x9adf57), 0);
    lv_obj_set_style_border_width(g_decoded_img, 2, 0);
    lv_obj_set_style_radius(g_decoded_img, 0, 0);
    lv_obj_set_style_pad_all(g_decoded_img, 0, 0);

    /* Status — bottom of screen. */
    g_status_label = lv_label_create(scr);
    lv_label_set_text(g_status_label, "Tap Capture to snapshot, then Send to TX over SSTV.");
    lv_obj_set_style_text_color(g_status_label, lv_color_hex(0xcccccc), 0);
    lv_obj_align(g_status_label, LV_ALIGN_BOTTOM_MID, 0, -15);

    /* Tick the preview repaint at 30 Hz. */
    lv_timer_create(preview_tick, 33, NULL);
}

/* ---------------------------------------------------------------- *
 *  Entry
 * ---------------------------------------------------------------- */

void app_run(void)
{
    ESP_LOGI(TAG, "Tab5 SSTV demo — live SC202CS capture");

    /* PSRAM buffers.
     *
     * g_preview_buf is the PPA DMA output target — needs DMA-capable
     * PSRAM and L2-cache-aligned address/size. heap_caps with the DMA
     * flag and 8BIT alignment satisfies this. */
    g_pcm_buf       = heap_caps_malloc(PCM_CAPACITY * sizeof(int16_t),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_image_buf     = heap_caps_malloc(SSTV_ROBOT36_WIDTH * SSTV_ROBOT36_HEIGHT * 2,
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_preview_buf   = heap_caps_aligned_calloc(64, 1, PREVIEW_W * PREVIEW_H * 2,
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    g_waterfall_buf = heap_caps_calloc(WF_W * WF_H, 2,
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_decoded_buf   = heap_caps_calloc(DEC_W * DEC_H, 2,
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_mic_samples   = heap_caps_malloc(FFT_N * sizeof(int16_t),
                                       MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    g_fft_data      = heap_caps_malloc(FFT_N * 2 * sizeof(float),
                                       MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    g_fft_window    = heap_caps_malloc(FFT_N * sizeof(float),
                                       MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (!g_pcm_buf || !g_image_buf || !g_preview_buf
        || !g_waterfall_buf || !g_decoded_buf
        || !g_mic_samples || !g_fft_data || !g_fft_window) {
        ESP_LOGE(TAG, "PSRAM/DRAM alloc failed");
        return;
    }

    /* Initial decoder state — empty raster at row 0. */
    memset(&g_dec, 0, sizeof(g_dec));
    /* Start with a black preview (so the LVGL widget shows something
     * before the first PPA frame lands). Waterfall is already zeroed
     * by heap_caps_calloc, which appears as black on screen. */
    memset(g_preview_buf, 0, PREVIEW_W * PREVIEW_H * 2);

    /* esp-dsp FFT tables + window. Cheap one-time cost (~150 µs). */
    if (dsps_fft2r_init_fc32(NULL, FFT_N) != ESP_OK) {
        ESP_LOGE(TAG, "dsps_fft2r_init_fc32 failed");
        return;
    }
    dsps_wind_blackman_harris_f32(g_fft_window, FFT_N);
    ESP_LOGI(TAG, "esp-dsp FFT ready (N=%d, %d Hz/bin)",
             FFT_N, MIC_SR_HZ / FFT_N);

    g_preview_mutex = xSemaphoreCreateMutex();
    if (!g_preview_mutex) {
        ESP_LOGE(TAG, "mutex alloc failed");
        return;
    }

    /* Display + touch + LVGL. */
    if (bsp_display_start() == NULL) {
        ESP_LOGE(TAG, "bsp_display_start failed");
        return;
    }
    ESP_LOGI(TAG, "display + touch + LVGL up");

    /* Camera. If this fails, we still let the UI run (preview stays
     * dark; button press will report "no frame"). */
    bool cam_ok = (cam_init() == ESP_OK);
    if (cam_ok) {
        xTaskCreate(cam_task, "cam_dq", 6144, NULL, 6, NULL);
    } else {
        ESP_LOGW(TAG, "camera init failed — UI runs, preview will be dark");
    }

    /* SSTV-RX mic + waterfall. Lower priority than cam_task — preview
     * latency matters more than waterfall jitter. Pinned to CPU 1 so
     * its work can't compete with cam_task / LVGL on CPU 0. Stack
     * is bigger because the FFT renderer allocates a 256-float array
     * on stack. */
    xTaskCreatePinnedToCore(mic_task, "mic_fft", 8192, NULL, 4, NULL, 1);

    /* Build UI under LVGL lock. */
    if (bsp_display_lock(0)) {
        build_ui();
        bsp_display_unlock();
        ESP_LOGI(TAG, "UI built");
    }

    /* Parallel probe: MI1602 thermal camera on Tab5 back-header.
     * Use the bus already opened by the Tab5 BSP so we don't fight over
     * I²C port 1. */
    mi1602_try_probe(bsp_i2c_get_handle());

    /* Done — LVGL has its own task, button event fires demo_task. */
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}

#endif /* CONFIG_CAMERA_TARGET_SC202CS_TAB5 */
