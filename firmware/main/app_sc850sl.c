/*
 * Flight-target application: SC850SL camera bring-up.
 *
 * Phase 1 (working, on hardware):
 *   1. Open I²C master bus (50 kHz)
 *   2. Diagnostic bus scan
 *   3. Power up sensor (rails managed by carrier; we toggle XSHUTDN)
 *   4. Probe chip ID 0x9d1e
 *   5. Walk M5Stack's 4K15 RAW10 2-lane init table (191 entries)
 *   6. Enable test pattern + stream-on
 *
 * Phase 2 (this file — software path, hardware verification pending):
 *   7. Open I²S TX to the PCM5102A DAC (GPIO 23 LRCK, 22 DIN, 21 BCK,
 *      20 SCK) for SSTV audio output.
 *   8. Set up the ESP32-P4 CSI host + ISP + PPA pipeline so the sensor's
 *      4K RAW10 stream lands in PSRAM as 1920×1080 RGB565. THIS PART IS
 *      STILL STUBBED — the IDF v6.0 esp_cam_ctlr_csi + driver/isp APIs
 *      need an exploration pass. Until that's done, the capture function
 *      returns a synthetic HSV color wheel.
 *   9. Spawn an sstv_tx_task that fires every 60 s:
 *        - capture frame (stub for now)
 *        - downscale to Robot36 native 320×240
 *        - sstv_robot36_encode → PSRAM PCM buffer
 *        - i2s_channel_write to the PCM5102A
 *
 * Heartbeat blink at 1 Hz on the configured GPIO (or no-op if -1).
 */
#include "app.h"

#if defined(CONFIG_CAMERA_TARGET_SC850SL)

#include <string.h>
#include <math.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_check.h"
#include "esp_cache.h"
#include "esp_ldo_regulator.h"
#include "esp_ota_ops.h"        /* OTA confirm/rollback (anti-brick) */
#include "esp_timer.h"          /* esp_timer_get_time: per-stage frame timing */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "driver/isp.h"
#include "driver/isp_wbg.h"
#include "driver/isp_ae.h"
#include "driver/usb_serial_jtag_vfs.h"  /* set_tx_line_endings: binary-clean stdout */
#include "esp_cam_ctlr.h"
#include "esp_cam_ctlr_csi.h"
#include "soc/isp_struct.h"   /* IDF-9706: direct ISP register pokes for bypass mode */

#include "sc850sl.h"
#include "sstv_robot36.h"
#include "ground_wifi.h"   /* ground-test WiFi + web control (no-op in flight) */
#include "test_image.h"      /* 320×240 RGB565 fallback test pattern */

static const char *TAG = "app/sc850sl";

/* ---------------------------------------------------------------- *
 *  Capture & audio configuration
 * ---------------------------------------------------------------- */

/* CAPTURE resolution = what we tell CSI + ISP to expect.
 *
 * NATIVE 4K, no sensor crop. The ISP's 1920×1080 input limit only applies
 * when the ISP is actually *processing* pixels (demosaic/CCM/etc). We run
 * the ISP in pure bypass (cntl.isp_en=0, frame_cfg poked by hand — see
 * camera_isp_bypass_start), so RAW10 bytes flow CSI → DMA untouched and
 * the 4K limit doesn't bind. table_2 is calibrated for 3840×2160 output;
 * cropping rewrote only the output-window regs while leaving HTS/VTS and
 * the rest of the 4K timing intact, which likely produced a malformed
 * MIPI frame the CSI never saw a clean frame-end on (isr_count stuck at
 * 0). Emitting the sensor's native, fully-calibrated frame removes that
 * variable.
 *
 * 3840×2160 RAW10 = 10,368,000 B per frame; we allocate g_capture_buf at
 * the RGB565-equivalent 16.6 MB (CAPTURE_W*CAPTURE_H*2), a superset that
 * also satisfies the IDF CSI driver's buflen >= fb_size_in_bytes check.
 * Fits in 32 MB PSRAM alongside the ~1.3 MB PCM buffer. */
#define CAPTURE_W                   3840
#define CAPTURE_H                   2160

/* PCM5102A DAC pins on the user's Stamp-P4 carrier. */
#define PCM5102A_LRCK_GPIO          23      /* word select / frame sync   */
#define PCM5102A_DIN_GPIO           22      /* MCU DOUT → DAC DIN          */
#define PCM5102A_BCK_GPIO           21      /* bit clock                   */
#define PCM5102A_SCK_GPIO           20      /* DAC master clock (256× fs)  */

/* Audio path matches Robot36's 16 kHz mono int16 wire format. */
#define AUDIO_SR_HZ                 16000

/* SSTV transmit cycle — this is the *sleep between cycles*, not the
 * total cycle period (each TX takes ~38 s on its own, so the effective
 * wall-clock interval is roughly 38 + this). */
#define SSTV_INTERVAL_SECS          10

/* PCM working buffer (~38 s of audio: 36 s Robot36 + 1 s trailing
 * silence + 1 s headroom = 608 K int16 samples = 1.18 MB PSRAM). */
#define PCM_CAPACITY                (AUDIO_SR_HZ * 40)

/* ---- Camera capture pipeline (sensor-crop → CSI → ISP → PSRAM) ---- */

/* MIPI PHY LDO (P4 internal regulator) — chan 3 @ 2.5 V per IDF docs. */
#define MIPI_PHY_LDO_CHAN_ID        3
#define MIPI_PHY_LDO_VOLTAGE_MV     2500

/* MIPI CSI bit rate on each of 2 lanes (matches sc850sl_table_2). */
#define CSI_LANE_BIT_RATE_MBPS      1080

/* SC850SL output window: 1920×1080 centered in the 3840×2160 array.
 * Sensor outputs only this region; CSI receives 1920×1080 directly so
 * no PPA downscale is needed. Loses 50% horizontal + 50% vertical FOV
 * (digital-zoom-like crop) but keeps every other pixel at full
 * resolution. Trade-off documented in PROJECT_MEMORY / capture design. */
#define SENSOR_CROP_W               CAPTURE_W
#define SENSOR_CROP_H               CAPTURE_H
#define SENSOR_CROP_X_START         ((3840 - SENSOR_CROP_W) / 2)
#define SENSOR_CROP_Y_START         ((2160 - SENSOR_CROP_H) / 2)

/* Frame buffer size for one capture (RGB565 = 2 bytes/pixel). */
#define CAPTURE_FB_BYTES            ((size_t) CAPTURE_W * CAPTURE_H * 2)

/* ---------------------------------------------------------------- *
 *  Globals
 * ---------------------------------------------------------------- */

static i2s_chan_handle_t g_i2s_tx     = NULL;
static int16_t          *g_pcm_buf    = NULL;     /* PCM_CAPACITY × int16   */
static uint16_t         *g_sstv_image = NULL;     /* 320×240 RGB565 Robot36 */
static uint16_t         *g_capture_buf = NULL;    /* CAPTURE_W×CAPTURE_H RGB565 */
static sc850sl_handle_t  g_cam        = NULL;
static volatile bool     g_cam_ready  = false;

/* MIPI CSI + ISP pipeline state. */
static esp_ldo_channel_handle_t g_mipi_ldo = NULL;
static esp_cam_ctlr_handle_t    g_csi      = NULL;
static isp_proc_handle_t        g_isp      = NULL;
static isp_ae_ctlr_t            g_ae       = NULL;
static SemaphoreHandle_t        g_csi_frame_done = NULL;     /* given in on_trans_finished */
static volatile uint32_t        g_csi_isr_count  = 0;        /* incremented in ISR for diagnostics */
static volatile bool            g_pipeline_ready = false;

/* Auto-exposure runtime state.
 *
 * SC850SL exposure is a 20-bit value across three registers, units of
 * 1/16 line at the sensor's HTS clock:
 *   0x3e00[3:0]   = exposure[19:16]   (high nibble)
 *   0x3e01[7:0]   = exposure[15:8]    (mid byte) — what M5Stack's
 *                                       table_2 sets to 0x82
 *   0x3e02[7:4]   = exposure[7:4]     (low nibble — fractional)
 * For simplicity we treat exposure as a 16-bit value packed in
 * 0x3e00 (high) + 0x3e01 (low), leaving 0x3e02 = 0.  Default after
 * init = 0x0082 → boot exposure.
 *
 * Target avg AE luminance is 128 (mid-gray for 8-bit). Step up/down
 * by a small percentage each iteration so the loop converges without
 * flapping; deadband of ±8 around target. */
#define AE_TARGET_LUMA              128
#define AE_DEADBAND                 8
#define AE_EXPOSURE_MIN             0x0010
#define AE_EXPOSURE_MAX             0x0fff
#define AE_PERIOD_MS                500

static uint16_t g_exposure = 0x0020;   /* legacy ISP-AE var (unused in bypass path) */

/* ---- Host-side auto-exposure (SC850SL has NO internal AEC; datasheet
 *      2.3.1 — the host must drive exposure + gain) ---------------------
 *
 * Frame timing (table_2): VTS = 0x08ca = 2250 lines. Max exposure is
 * VTS-4 (table 2-4). Exposure is in line units, programmed across
 * {0x3e00[3:0],0x3e01[7:0],0x3e02[7:4]}. Analog gain is the piecewise
 * DCG+analog code on 0x3e06/0x3e08/0x3e09 (vendor recipe, task #22).
 * Writes take effect on a plain register write (sensor latches at the next
 * frame); there is NO group-hold, and we must NOT set 0x3e03 — doing either
 * was exactly what made our earlier exposure sweep do nothing. */
#define SC850SL_VTS_LINES   2250
#define SC850SL_EXP_MAX     (SC850SL_VTS_LINES - 4)   /* 2246 */
#define SC850SL_EXP_MIN     2
#define AE_TARGET_RAW       90      /* target raw-frame mean (8-bit). Lowered
                                     * 110->90 (2026-06): daytime LEO scenes are
                                     * high-contrast; a lower mean leaves headroom
                                     * so bright cloud/ice/glint does not clip.
                                     * Shadows are lifted back by the gamma. */
#define AE_DEAD_RAW         10

/* Highlight protection -- the daytime-overexposure fix. Mean metering alone
 * blows out a dark-dominant scene's highlights: it raises exposure to hit the
 * mean target and clips the clouds. So we also cap the fraction of near-full-
 * scale pixels. AE_SAT_LEVEL is in the 8-bit high-byte domain of the RAW10
 * sample (255 = full scale); 250 ~= raw 1000/1023. AE_SAT_BUDGET_PPM is the
 * share of pixels at/above that level tolerated before the highlight guard in
 * ae_step() forces brightness down regardless of the mean. */
#define AE_SAT_LEVEL        250u    /* ~raw 1000/1023 in the 8-bit sample */
#define AE_SAT_BUDGET_PPM   8000u   /* 0.8% of pixels may clip before pull-down */

/* Gain is expressed in 1024 = 1.0x units (the vendor driver's convention);
 * sc850sl_set_exp_gain() maps it to the SC850SL's piecewise DCG+analog code
 * per the recipe recovered from sensor_sc850sl_mipi.c (task #22), cross-checked
 * register-for-register against the CVITEK cv183x driver
 * (reference/sophgo_cv183x_sc850sl/, 2026-06). */
#define SC850SL_GAIN_UNITY  1024u
#define SC850SL_AGAIN_MAX   50790u   /* 49.6x - datasheet analog SENSOR_MAXGAIN */
#define SC850SL_GAIN_MAX    (SC850SL_AGAIN_MAX * 8u)  /* + up to 8x digital ->
                                     * ~397x total. Digital is NIGHT-only head-
                                     * room past the analog ceiling; it steps in
                                     * 6 dB (2x/4x/8x), so expect brightness
                                     * stepping at the very top. UNTESTED on HW. */

/* Anti-flicker: snap exposure to whole light-ripple periods so AC-lit scenes
 * don't band. Line period for our 2-lane 4K15 mode ≈ frame(66.67ms @15fps) /
 * VTS(2250) = 29.63 µs; 50 Hz mains ripples at 100 Hz → 10 ms, 60 Hz → 8.33 ms.
 * g_antiflicker_hz = 0 disables it. (If banding persists the true line period
 * differs from the estimate — just retune SC850SL_LINE_PERIOD_NS.) */
#define SC850SL_LINE_PERIOD_NS    29630u
#define SC850SL_FLICKER_LINES_50  ((10000000u + SC850SL_LINE_PERIOD_NS/2) / SC850SL_LINE_PERIOD_NS) /* ~338 */
#define SC850SL_FLICKER_LINES_60  (( 8333333u + SC850SL_LINE_PERIOD_NS/2) / SC850SL_LINE_PERIOD_NS) /* ~281 */
static int g_antiflicker_hz = 50;   /* 0 = off, 50, or 60 */

static uint32_t g_ae_exp_lines  = 676;                 /* current exposure (lines), ~20 ms */
static uint32_t g_ae_gain_x1024 = SC850SL_GAIN_UNITY;  /* current gain, 1024 = 1.0x */

/* Gray-world AWB per-channel gains (green normalized to 1.0), updated
 * each frame in the demosaic. Declared here so usb_stream_send can log
 * them. */
static float g_wb_r = 1.6f;
static float g_wb_b = 1.3f;

/* Black-level pedestal, in the 8-bit space of the high-8-of-10 raw sample.
 * The sensor biases every pixel by an optical-black pedestal, so the
 * darkest scene reads ~this value instead of 0. Gray-world's multiplicative
 * gains can NOT remove an additive offset, so it survives as the milky grey
 * "haze" / crushed-contrast look. We subtract it (then rescale the range
 * back to full-scale) before WB.
 *
 * 16 (= 64 LSB in 10-bit) is the typical Smartsens pedestal and a safe
 * default. To set it PRECISELY: cap the lens, capture one dark frame, and
 * read the 'raw stats: ... mean=' line (now sampled over real pixels only)
 * — that mean IS the pedestal. Set this to it. */
static int g_black_level = 16;

/* Digital exposure gain, applied in linear space (folded into the black-level
 * range-restore scale). 1.0 = no-op. The SC850SL ignores our exposure/gain
 * register writes (see ae_step / task #22), so until that's cracked this is
 * the only brightness lever. Caveat: it amplifies a signal that may only span
 * a few dozen raw codes in dim light, so values much above ~2.0 start to
 * posterize (banding in smooth areas). More light is always better. */
static float g_dig_gain = 1.0f;

/* Colour Correction Matrix: sensor-RGB -> sRGB, applied in LINEAR space
 * AFTER white-balance gain and BEFORE gamma. WB only scales the three
 * channels independently; it can't undo the sensor's spectral cross-talk,
 * which is what leaves colours muddy / low-saturation. The off-diagonal
 * terms fix that. Rows sum to 1.0 so a neutral (R=G=B) input stays neutral
 * — the matrix changes saturation/hue, not the white point (that's WB's
 * job).
 *
 * THESE ARE GENERIC DAYLIGHT-ISH COEFFS, a starting point only. For an
 * accurate result, shoot a 24-patch ColorChecker under your actual lighting
 * and least-squares solve sensor-RGB -> sRGB (optionally with an offset
 * term). Set to identity (diag 1, rest 0) to disable. */
static float g_ccm[3][3] = {
    { 1.40f, -0.28f, -0.12f },
    {-0.22f,  1.42f, -0.20f },
    {-0.05f, -0.35f,  1.40f },
};

/* ---- USB live-preview stream ---- *
 *
 * Every ~2 s, downscale the latest captured frame to 320×240 RGB565 and
 * push it out over USB-Serial-JTAG with a framing header so a host-side
 * Python script can decode + display in real time. Lets us verify scene
 * content without waiting 38 s for an SSTV cycle.
 *
 * Frame format on the wire:
 *   magic[8]   = "\x00\x00\xff\xffFRM\x00"  (8 bytes — nulls + non-printable
 *                so it never collides with ESP_LOGx output, which is pure
 *                ASCII text)
 *   width:  u16 LE   (always 320 for now)
 *   height: u16 LE   (always 240 for now)
 *   data:   width × height × 2 bytes RGB565 little-endian
 */
/* USB preview is a side-by-side composite: RGB (visible) on the LEFT,
 * thermal (MI1602, false-colored) on the RIGHT, each scaled to the same
 * pane size. The host viewer reads w/h from the header so it shows the
 * whole composite as one image.
 *
 *   composite = [ RGB pane | thermal pane ]
 *   width  = 2 × USB_PANE_W,  height = USB_PANE_H
 *
 * 320×240 per pane → 640×240 composite = 307 KB/frame. The USB-Serial-
 * JTAG link is USB 2.0 Full-Speed (~0.8-1 MB/s), so the blocking fwrite
 * paces this at ~1.5-2 fps. */
#define USB_PANE_W                  320
#define USB_PANE_H                  240
#define USB_STREAM_W                (USB_PANE_W * 2)   /* composite width */
#define USB_STREAM_H                (USB_PANE_H)       /* composite height */
#define USB_STREAM_BYTES            (USB_STREAM_W * USB_STREAM_H * 2)
#define USB_STREAM_PERIOD_MS        100   /* loop interval; real rate is USB-bound */

/* Wire framing (format v1). The whole frame lives in ONE contiguous buffer
 * (g_frame_buf) so it can be pushed with a SINGLE fwrite — newlib holds the
 * stdout file lock for the entire call, which makes the frame atomic against
 * any other task's ESP_LOGx. (The old 4-separate-fwrite scheme let log text
 * splice into the middle of the binary payload, shifting every following
 * byte; because RGB565 is 2 bytes/pixel an odd-byte shift swaps hi/lo of
 * every pixel — that was the "smear + false-colour stripes".)
 *
 *   [0]  magic[8]      00 00 ff ff 'F' 'R' 'M' 01   (01 = format version)
 *   [8]  u16 width,  u16 height
 *   [12] u16 raw_min, u16 raw_max, u16 raw_mean      (per-frame exposure stats)
 *   [18] u16 black_level, u16 wb_r*256, u16 wb_b*256
 *        ^-- header = USB_HDR_BYTES (24)
 *   [24] payload[USB_STREAM_BYTES]                   (RGB565 LE)
 *   [..] trailer[4] = DE AD BE EF                    (fixed sentinel)
 *
 * The trailer at a FIXED offset lets the host validate alignment without a
 * CRC (USB CDC has its own link-layer CRC, so corruption here is byte
 * drop/insert, not bit-flip — a fixed sentinel catches all of those). If the
 * trailer is wrong the host drops the frame and re-syncs on the next magic,
 * so a bad frame is skipped rather than displayed as a smear. */
#define USB_HDR_BYTES               24
#define USB_TRAILER_BYTES           4
#define USB_FRAME_TOTAL             (USB_HDR_BYTES + USB_STREAM_BYTES + USB_TRAILER_BYTES)

static const uint8_t USB_STREAM_MAGIC[8] = {
    0x00, 0x00, 0xff, 0xff, 'F', 'R', 'M', 0x01,   /* 0x01 = wire format v1 */
};
static const uint8_t USB_STREAM_TRAILER[USB_TRAILER_BYTES] = { 0xDE, 0xAD, 0xBE, 0xEF };

/* Little-endian u16 store, alignment-safe (header offsets are byte-addressed). */
static inline void put_u16le(uint8_t *p, uint16_t v) { p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); }

static SemaphoreHandle_t g_capture_mutex = NULL;
/* Contiguous frame buffer: [header][payload][trailer]. The pixel renderers
 * write into g_usb_stream_buf, which points at the payload region inside it. */
static uint8_t           *g_frame_buf = NULL;
static uint16_t          *g_usb_stream_buf = NULL;   /* = g_frame_buf + USB_HDR_BYTES, USB_STREAM_W×H RGB565 */

/* Forward decls (definitions further down). */
static void downscale_to_robot36(const uint16_t *src, int sw, int sh,
                                 uint16_t *dst, int dw, int dh);
static void downscale_raw10_bggr_to_rgb565(const uint8_t *src, int sw, int sh,
                                           uint16_t *dst, int dw, int dh,
                                           int dst_stride);
static esp_err_t camera_capture_frame(uint16_t *out_rgb565);

/* ---------------------------------------------------------------- *
 *  I²C bus
 * ---------------------------------------------------------------- */

static i2c_master_bus_handle_t i2c_bus_open(void)
{
    i2c_master_bus_config_t cfg = {
        .i2c_port          = I2C_NUM_0,
        .sda_io_num        = CONFIG_SC850SL_I2C_SDA_GPIO,
        .scl_io_num        = CONFIG_SC850SL_I2C_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,   /* external pull-ups to 1.8 V */
    };
    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(i2c_new_master_bus(&cfg, &bus));
    ESP_LOGI(TAG, "I²C master open: SDA=%d SCL=%d", cfg.sda_io_num, cfg.scl_io_num);
    return bus;
}

/* ---------------------------------------------------------------- *
 *  Heartbeat LED
 * ---------------------------------------------------------------- */

static void heartbeat_setup(gpio_num_t pin)
{
    if (pin < 0) return;
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&cfg);
}

static void heartbeat_blip(gpio_num_t pin, int level)
{
    if (pin < 0) return;
    gpio_set_level(pin, level);
}

/* ---------------------------------------------------------------- *
 *  PCM5102A DAC — I²S TX setup
 * ---------------------------------------------------------------- */

/*
 * The PCM5102APWR is a stereo 32-bit/384 kHz DAC with no I²C control —
 * it auto-configures based on the BCK/SCK ratio. We feed it standard
 * Philips I²S (data MSB-first, 1 BCK after LRCK edge) at 16 kHz / mono
 * 16-bit. With MCLK_MULTIPLE_256, BCK = 16k × 32 = 512 kHz and
 * SCK = 16k × 256 = 4.096 MHz — within the DAC's 4× internal PLL window.
 */
static esp_err_t pcm5102a_init(void)
{
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan_cfg.dma_desc_num  = 6;
    chan_cfg.dma_frame_num = 1024;
    ESP_RETURN_ON_ERROR(i2s_new_channel(&chan_cfg, &g_i2s_tx, NULL),
                        TAG, "i2s_new_channel");

    i2s_std_config_t std_cfg = {
        .clk_cfg = {
            .sample_rate_hz = AUDIO_SR_HZ,
            .clk_src        = I2S_CLK_SRC_DEFAULT,
            .mclk_multiple  = I2S_MCLK_MULTIPLE_256,
        },
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
            I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = PCM5102A_SCK_GPIO,
            .bclk = PCM5102A_BCK_GPIO,
            .ws   = PCM5102A_LRCK_GPIO,
            .dout = PCM5102A_DIN_GPIO,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {0},
        },
    };
    /* Our carrier wires only the PCM5102A's OUT_R to the speaker.
     * I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG() with SLOT_MODE_MONO defaults
     * the slot mask to LEFT (OUT_L), which would land on the unconnected
     * channel — flip to RIGHT so the mono Robot36 stream comes out of
     * the speaker. (Alternative: SLOT_BOTH would copy to both channels;
     * RIGHT-only saves a tiny bit of bus bandwidth.) */
    std_cfg.slot_cfg.slot_mask = I2S_STD_SLOT_RIGHT;
    ESP_RETURN_ON_ERROR(i2s_channel_init_std_mode(g_i2s_tx, &std_cfg),
                        TAG, "i2s_channel_init_std_mode");
    ESP_RETURN_ON_ERROR(i2s_channel_enable(g_i2s_tx),
                        TAG, "i2s_channel_enable");

    ESP_LOGI(TAG, "✓ PCM5102A I²S: %d Hz mono 16-bit "
                  "(MCLK=GPIO%d BCK=GPIO%d LRCK=GPIO%d DIN=GPIO%d)",
             AUDIO_SR_HZ, PCM5102A_SCK_GPIO, PCM5102A_BCK_GPIO,
             PCM5102A_LRCK_GPIO, PCM5102A_DIN_GPIO);
    return ESP_OK;
}

/* ---------------------------------------------------------------- *
 *  Camera capture pipeline (STUBBED — replace in next session)
 * ---------------------------------------------------------------- */

/*
 * Real CSI + ISP capture pipeline — design notes after IDF v6.0.1 API
 * exploration. Implementation lands in tasks #13/#14/#15.
 *
 * Memory budget (32 MB PSRAM, ~5.5 MB already taken by other allocs):
 *   - 4K RGB565 frame  = 3840·2160·2 = 16.6 MB    → too tight for dual-buf
 *   - 1080p RGB565     = 1920·1080·2 =  4.0 MB    → dual-buf = 8 MB, fine
 *
 * Decision: output 1920×1080 from the pipeline directly (no full-4K buffer).
 * Two ways to get there from a sensor running 4K15:
 *   (a) ISP crop — sensor runs 4K, ISP defines a 1920×1080 window
 *       (esp_isp_crop_configure). Loses 50% horiz + 50% vert FOV.
 *   (b) Sensor crop — call sc850sl_set_window(1920,1080,...) so the
 *       sensor outputs only the 1920×1080 region. CSI receives that
 *       directly. Same FOV loss but lower MIPI bandwidth (1/4).
 * Going with (b) — simpler config, lower power, no ISP crop module to
 * tune.  PPA SRM is then a no-op (no further scaling needed); if the
 * user later wants the full FOV preserved, add (c) sensor 2×2 binning
 * (requires a new init table) and remove the sensor crop.
 *
 * Sequence to bring the pipeline up after sc850sl_load_init_table():
 *
 *  1. sc850sl_set_window(g_cam, 1920, 1080, x_center, y_center)
 *
 *  2. esp_cam_new_csi_ctlr(&csi_cfg)
 *       .h_res = 1920, .v_res = 1080
 *       .data_lane_num = 2, .lane_bit_rate_mbps = 1080
 *       .input_data_color_type  = CAM_CTLR_COLOR_RAW10
 *       .output_data_color_type = CAM_CTLR_COLOR_RGB565
 *       .queue_items = 1
 *     esp_cam_ctlr_register_event_callbacks(...)   — supply our buffers
 *     esp_cam_ctlr_enable(...)
 *
 *  3. esp_isp_new_processor(&isp_cfg)
 *       .input_data_source     = ISP_INPUT_DATA_SOURCE_CSI
 *       .input_data_color_type = ISP_COLOR_RAW10
 *       .output_data_color_type= ISP_COLOR_RGB565
 *       .h_res = 1920, .v_res = 1080
 *       .bayer_order = (verify from sc850sl_table_2 — likely BGGR)
 *       .clk_hz = 80 MHz
 *     esp_isp_enable(...)
 *
 *  4. Enable per-module pipelines (each has its own header in
 *     driver/isp_*.h):
 *       esp_isp_blc_configure + enable        — black level
 *       esp_isp_demosaic_configure + enable   — Bayer → RGB
 *       esp_isp_ccm_configure + enable        — colour matrix
 *       esp_isp_gamma_configure + enable      — γ ≈ 0.7
 *       esp_isp_sharpen_configure + enable
 *       (P4 rev v0.3+ also: esp_isp_awb + esp_isp_ae for closed loops)
 *
 *  5. sc850sl_stream_on(g_cam)
 *  6. esp_cam_ctlr_start(handle)
 *
 *  Per-frame in sstv_tx_task:
 *     esp_cam_ctlr_trans_t t = {.buffer = g_capture_buf,
 *                               .buflen = CAPTURE_W*CAPTURE_H*2};
 *     esp_cam_ctlr_receive(handle, &t, ESP_CAM_CTLR_MAX_DELAY);
 *     esp_cache_msync(g_capture_buf, ..., CACHE_MSYNC_FLAG_DIR_M2C);
 *     downscale_to_robot36(g_capture_buf, ..., g_sstv_image, 320, 240);
 *
 * Until the above is implemented, camera_capture_frame() is bypassed and
 * sstv_tx_task memcpy's the static test image directly into g_sstv_image.
 */

/* HSV color-wheel test pattern, same generator as the Tab5 demo but
 * sized for our 1920×1080 capture buffer. */
static void make_test_pattern(uint16_t *rgb565, int w, int h)
{
    for (int y = 0; y < h; ++y) {
        float hue = (float)y / h * 6.0f;
        int   hi  = (int)hue;
        float f   = hue - hi;
        for (int x = 0; x < w; ++x) {
            float v = (float)x / w;
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
            rgb565[y * w + x] =
                (uint16_t)(((R >> 3) << 11) | ((G >> 2) << 5) | (B >> 3));
        }
    }
}

/* Default gamma curve: y = (x/255)^0.7 × 255. Slight brightness lift in
 * shadows, decent contrast — matches the ESP-IDF multi_pipelines default. */
static uint32_t isp_gamma_default_curve(uint32_t x)
{
    return (uint32_t)(pow((double)x / 256.0, 0.7) * 256.0);
}

/* Enable all ISP modules we want active for the SC850SL bring-up:
 *   demosaic → CCM (identity) → gamma → sharpen → color adjust.
 * Each module here has been turned into a one-shot enabler so failures
 * don't cascade — we log the warning and continue, since most modules
 * are independent. */
static esp_err_t isp_pipeline_enable_all(isp_proc_handle_t isp)
{
    esp_err_t r;

    /* --- (Note: WBG hardware module isn't available on ESP32-P4 silicon
     *     prior to rev v3.0 — our Stamp-P4 is rev v1.3 — so the white
     *     balance gain is folded into the CCM matrix diagonal below.) */

    /* --- Demosaic (Bayer → RGB). Without this the CSI/ISP path emits
     *     raw Bayer bytes interpreted as RGB565, which looks black/garbled. */
    esp_isp_demosaic_config_t demosaic_cfg = {
        .grad_ratio = { .integer = 2, .decimal = 5 },
    };
    r = esp_isp_demosaic_configure(isp, &demosaic_cfg);
    if (r == ESP_OK) r = esp_isp_demosaic_enable(isp);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "demosaic: %s", esp_err_to_name(r));
        return r;       /* hard fail — without demosaic nothing else helps */
    }
    ESP_LOGI(TAG, "  ISP demosaic enabled");

    /* --- CCM (Color Correction Matrix). Diagonal carries the WB gains
     *     (R≈1.80×, G=1.00×, B≈1.56×) since the hardware WBG module is
     *     v3.0+ only. ESP32-P4 v1.x CCM coefficient range is ±3.999, so
     *     these values fit comfortably. Off-diagonal stays zero — no
     *     cross-channel mixing for now; tune once the bench has a known
     *     colour reference. */
    esp_isp_ccm_config_t ccm_cfg = {
        .matrix = {
            {1.80f, 0.00f, 0.00f},
            {0.00f, 1.00f, 0.00f},
            {0.00f, 0.00f, 1.56f},
        },
        .saturation = false,
    };
    r = esp_isp_ccm_configure(isp, &ccm_cfg);
    if (r == ESP_OK) r = esp_isp_ccm_enable(isp);
    if (r != ESP_OK) ESP_LOGW(TAG, "  CCM not enabled: %s", esp_err_to_name(r));
    else             ESP_LOGI(TAG, "  ISP CCM enabled (identity matrix)");

    /* --- Gamma (per-channel, same curve on R/G/B). */
    isp_gamma_curve_points_t pts = {0};
    r = esp_isp_gamma_fill_curve_points(isp_gamma_default_curve, &pts);
    if (r == ESP_OK) r = esp_isp_gamma_configure(isp, COLOR_COMPONENT_R, &pts);
    if (r == ESP_OK) r = esp_isp_gamma_configure(isp, COLOR_COMPONENT_G, &pts);
    if (r == ESP_OK) r = esp_isp_gamma_configure(isp, COLOR_COMPONENT_B, &pts);
    if (r == ESP_OK) r = esp_isp_gamma_enable(isp);
    if (r != ESP_OK) ESP_LOGW(TAG, "  gamma not enabled: %s", esp_err_to_name(r));
    else             ESP_LOGI(TAG, "  ISP gamma enabled (γ ≈ 0.7)");

    /* --- Sharpen */
    esp_isp_sharpen_config_t sharpen_cfg = {
        .h_freq_coeff  = { .integer = 2, .decimal = 0 },
        .m_freq_coeff  = { .integer = 2, .decimal = 0 },
        .h_thresh      = 255,
        .l_thresh      = 0,
        .padding_mode  = ISP_SHARPEN_EDGE_PADDING_MODE_SRND_DATA,
        .sharpen_template = {
            {1, 2, 1},
            {2, 4, 2},
            {1, 2, 1},
        },
    };
    r = esp_isp_sharpen_configure(isp, &sharpen_cfg);
    if (r == ESP_OK) r = esp_isp_sharpen_enable(isp);
    if (r != ESP_OK) ESP_LOGW(TAG, "  sharpen not enabled: %s", esp_err_to_name(r));
    else             ESP_LOGI(TAG, "  ISP sharpen enabled");

    /* --- Color adjustment (contrast/saturation/hue/brightness). */
    esp_isp_color_config_t color_cfg = {
        .color_contrast   = { .integer = 1, .decimal = 0 },
        .color_saturation = { .integer = 1, .decimal = 0 },
        .color_hue        = 0,
        .color_brightness = 0,
    };
    r = esp_isp_color_configure(isp, &color_cfg);
    if (r == ESP_OK) r = esp_isp_color_enable(isp);
    if (r != ESP_OK) ESP_LOGW(TAG, "  color adjust not enabled: %s", esp_err_to_name(r));
    else             ESP_LOGI(TAG, "  ISP color-adjust enabled");

    return ESP_OK;
}

/* CSI buffer-request callback: tell the driver where to write the next
 * frame. We always return the single shared g_capture_buf — the CSI
 * runs continuously and keeps overwriting it; consumers wait on the
 * frame-done semaphore and read whichever frame is current.
 *
 * One-buffer-only design: occasional snapshots, no double-buffering.
 * Tearing is theoretically possible if a consumer reads while DMA writes
 * the same buffer; in practice the frame period (≈67 ms) is much longer
 * than a downscale + memcpy so we never collide in the same scanline. */
static bool csi_on_get_new_trans(esp_cam_ctlr_handle_t handle,
                                 esp_cam_ctlr_trans_t *trans,
                                 void *user_data)
{
    (void)handle; (void)user_data;
    trans->buffer = g_capture_buf;
    trans->buflen = CAPTURE_FB_BYTES;
    return false;
}

/* CSI completion callback (fires from ISR when a DMA finished). Signals
 * the binary semaphore so a parked camera_capture_frame() can wake up
 * and use the freshly written buffer. */
static bool csi_on_trans_finished(esp_cam_ctlr_handle_t handle,
                                   esp_cam_ctlr_trans_t *trans,
                                   void *user_data)
{
    (void)handle; (void)trans; (void)user_data;
    BaseType_t high_task_woken = pdFALSE;
    g_csi_isr_count++;
    if (g_csi_frame_done) {
        xSemaphoreGiveFromISR(g_csi_frame_done, &high_task_woken);
    }
    return high_task_woken == pdTRUE;
}

static esp_err_t camera_pipeline_init(void)
{
    esp_err_t r;

    /* 1) Allocate the single capture buffer (PSRAM, DMA-able, 64-byte
     *    aligned for L2 cache line on P4). */
    g_capture_buf = heap_caps_aligned_calloc(
        64, 1, CAPTURE_FB_BYTES,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
    if (!g_capture_buf) {
        ESP_LOGE(TAG, "capture fb alloc failed (%u bytes)",
                 (unsigned) CAPTURE_FB_BYTES);
        return ESP_ERR_NO_MEM;
    }

    /* 2) Power the MIPI CSI PHY rail (internal LDO ch3 @ 2.5 V). */
    esp_ldo_channel_config_t ldo_cfg = {
        .chan_id    = MIPI_PHY_LDO_CHAN_ID,
        .voltage_mv = MIPI_PHY_LDO_VOLTAGE_MV,
    };
    r = esp_ldo_acquire_channel(&ldo_cfg, &g_mipi_ldo);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "MIPI PHY LDO: %s", esp_err_to_name(r));
        return r;
    }
    ESP_LOGI(TAG, "MIPI PHY LDO ch%d @ %d mV", MIPI_PHY_LDO_CHAN_ID,
             MIPI_PHY_LDO_VOLTAGE_MV);

    /* 3) Create the CSI controller.
     *    The actual sensor data on the MIPI wire is RAW10. The actual
     *    bytes the DMA writes to PSRAM are RGB565 — because the on-chip
     *    ISP sits inline between the CSI bridge and the GDMA and does
     *    the RAW10 → debayer → CCM → gamma → RGB565 conversion.
     *
     *    The CSI driver's "input_data_color_type" / "output_data_color_type"
     *    don't refer to the wire format vs. ISP output — they describe
     *    what the CSI BRIDGE's internal format-conversion module sees on
     *    its source and destination sides. With the ISP in the loop, the
     *    bridge sees RGB565 on both sides (the ISP feeds it pre-converted
     *    RGB565, and the bridge just transports it to DMA). This matches
     *    the v6.0+ convention used by esp_video's CSI device driver
     *    (espressif__esp_video/.../esp_video_csi_device.c line 478-485:
     *    "Bypass CSI convert function" sets input = output = out_color).
     *
     *    Crucially, fb_size_in_bytes and csi_transfer_size are computed
     *    from h_res × v_res × out_bpp. With out_color=RGB565 (16 bpp),
     *    those land at CAPTURE_W × CAPTURE_H × 2 = our buffer size, and
     *    the DMA fires "trans-done" after that many bytes — which is
     *    exactly when one full RGB565 frame from the ISP has arrived.
     *
     *    With RAW10 → RAW10 (the previous setting) the DMA was sized
     *    for 10-bpp data but received 16-bpp from the ISP, so the byte
     *    count was wrong and the trans-done ISR never fired. */
    /* RAW10 → RAW10 pure bypass. The CSI bridge transports MIPI RAW10
     * bytes straight to PSRAM via DMA, ISP is NOT in the path. The CPU
     * debayers in software when we downscale for the USB preview and
     * SSTV. This is slower than letting the ISP do the work, but the
     * ISP-in-the-loop config has been a moving target (intermittent
     * fault, FIFO overflow on some clock settings, silent no-frames on
     * others) — and we want an image today. Total raw frame size:
     * 1920×1080×10/8 = 2,592,000 bytes. Our g_capture_buf is 4.1 MB
     * (sized for the eventual RGB565 path) — buflen > fb_size_in_bytes
     * is fine per the IDF API contract. */
    esp_cam_ctlr_csi_config_t csi_cfg = {
        .ctlr_id                = 0,
        .h_res                  = CAPTURE_W,
        .v_res                  = CAPTURE_H,
        .lane_bit_rate_mbps     = CSI_LANE_BIT_RATE_MBPS,
        .input_data_color_type  = CAM_CTLR_COLOR_RAW10,
        .output_data_color_type = CAM_CTLR_COLOR_RAW10,
        .data_lane_num          = 2,
        .byte_swap_en           = false,
        .queue_items            = 1,
        .bk_buffer_dis          = true,
    };
    r = esp_cam_new_csi_ctlr(&csi_cfg, &g_csi);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "esp_cam_new_csi_ctlr: %s", esp_err_to_name(r));
        return r;
    }

    /* We register ONLY on_trans_finished (not on_get_new_trans). The
     * callback signals a binary semaphore when DMA completes, so
     * camera_capture_frame() can block on the semaphore instead of
     * polling. We deliberately leave on_get_new_trans unset so the
     * CSI doesn't auto-pull buffers in the background — that path
     * fills the queue with stale frames during our long idle windows
     * (38 s SSTV TX, 10 s sleep). */
    g_csi_frame_done = xSemaphoreCreateBinary();
    if (!g_csi_frame_done) {
        ESP_LOGE(TAG, "frame-done semaphore alloc failed");
        return ESP_ERR_NO_MEM;
    }
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_trans  = csi_on_get_new_trans,
        .on_trans_finished = csi_on_trans_finished,
    };
    r = esp_cam_ctlr_register_event_callbacks(g_csi, &cbs, NULL);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "csi register cbs: %s", esp_err_to_name(r));
        return r;
    }
    r = esp_cam_ctlr_enable(g_csi);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "esp_cam_ctlr_enable: %s", esp_err_to_name(r));
        return r;
    }
    /* ISP creation and frame_cfg pokes happen LATER, after we've started
     * the CSI receiver — see camera_isp_bypass_start() called from the
     * stream-on sequence in app_run. esp_video does it in this order
     * (esp_video_csi_device.c line 428 → 437). */

#if 0
    r = esp_isp_enable(g_isp);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "esp_isp_enable: %s", esp_err_to_name(r));
        return r;
    }

    /* 5) Configure + enable all ISP pipeline modules. Demosaic is the
     *    one we can't live without (raw Bayer → RGB); the others are
     *    best-effort and log warnings on failure. */
    r = isp_pipeline_enable_all(g_isp);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "ISP pipeline modules failed: %s", esp_err_to_name(r));
        return r;
    }

    /* 6) AE controller — collects 5×5 luminance stats per frame so an
     *    ae_task can drive the sensor exposure in closed loop. Window
     *    covers the entire frame. Sample point AFTER_DEMOSAIC gives the
     *    cleanest signal for measuring scene brightness. */
    esp_isp_ae_config_t ae_cfg = {
        .sample_point = ISP_AE_SAMPLE_POINT_0,
        .window = {
            .top_left  = { .x = 0,           .y = 0           },
            .btm_right = { .x = CAPTURE_W - 1, .y = CAPTURE_H - 1 },
        },
        .intr_priority = 0,
    };
    r = esp_isp_new_ae_controller(g_isp, &ae_cfg, &g_ae);
    if (r == ESP_OK) r = esp_isp_ae_controller_enable(g_ae);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "AE controller not enabled: %s — exposure will be fixed",
                 esp_err_to_name(r));
        g_ae = NULL;       /* don't spawn the AE task */
    } else {
        ESP_LOGI(TAG, "  ISP AE controller enabled (%dx%d luminance blocks)",
                 ISP_AE_BLOCK_X_NUM, ISP_AE_BLOCK_Y_NUM);
    }
#endif  /* ISP creation disabled */

    /* Disable AE controller spawn — depends on ISP which we skipped. */
    g_ae = NULL;
    g_isp = NULL;

    ESP_LOGI(TAG, "✓ CSI up (no ISP): %d×%d RAW10 @ %d Mbps/lane × 2 lanes",
             CAPTURE_W, CAPTURE_H, CSI_LANE_BIT_RATE_MBPS);
    g_pipeline_ready = true;
    return ESP_OK;
}

/* IDF-9706 ISP-bypass setup, called AFTER esp_cam_ctlr_start() per
 * esp_video's csi_video_start() ordering. The ISP module sits between
 * the CSI host and the GDMA on ESP32-P4; even when we want pure RAW
 * pass-through, the ISP's frame_cfg registers must reflect the frame
 * geometry, otherwise the CSI→DMA pipeline never sees a frame-end
 * event and on_trans_finished never fires.
 *
 *   hadr_num = ceil(h_res * out_bpp / 32) - 1
 *   vadr_num = v_res - 1
 *   cntl.isp_en = 0 (ensure ISP processing is OFF — we want raw bytes)
 *
 * For 1920×1080 RAW10: hadr=599 vadr=1079. */
static esp_err_t camera_isp_bypass_start(void)
{
    esp_isp_processor_cfg_t isp_cfg = {
        .clk_hz                 = 80 * 1000 * 1000,
        .input_data_source      = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type  = ISP_COLOR_RAW8,         /* placeholder */
        .output_data_color_type = ISP_COLOR_RGB565,       /* placeholder */
        .h_res                  = CAPTURE_W,
        .v_res                  = CAPTURE_H,
        .bayer_order            = COLOR_RAW_ELEMENT_ORDER_BGGR,
        .has_line_start_packet  = false,
        .has_line_end_packet    = false,
    };
    esp_err_t r = esp_isp_new_processor(&isp_cfg, &g_isp);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "esp_isp_new_processor: %s", esp_err_to_name(r));
        return r;
    }
    const uint8_t out_bpp = 10;
    ISP.frame_cfg.hadr_num = ((CAPTURE_W * out_bpp + 31) / 32) - 1;
    ISP.frame_cfg.vadr_num = CAPTURE_H - 1;
    ISP.cntl.isp_en = 0;
    ESP_LOGI(TAG, "ISP frame_cfg: hadr_num=%u vadr_num=%u (bypass, isp_en=0)",
             (unsigned) ISP.frame_cfg.hadr_num,
             (unsigned) ISP.frame_cfg.vadr_num);
    return ESP_OK;
}

/* Program exposure (lines) + analog gain (gain_x1024, 1024 = 1.0x) using the
 * recipe recovered from the vendor/OpenIPC SC850SL driver
 * (sensor_sc850sl_mipi.c: pCus_SetAEUSecs / pCus_SetAEGain — see task #22).
 *
 * Corrections vs the old code, which is why manual exposure/gain did NOTHING:
 *   - NO group-hold register. The SC850SL has none at 0x3800 (that is not a
 *     group-hold address); the vendor just writes the registers plainly and
 *     the sensor latches them at its next frame boundary. The old 0x3800
 *     pack/commit poking + the boot-time 0x3e03=0x0b "AGC enable" were what
 *     stopped our writes from taking effect.
 *   - Gain is the piecewise DCG + analog code, not a raw ladder value.
 * Exposure field = {0x3e00[3:0],0x3e01[7:0],0x3e02[7:4]}, value = lines << 4. */
static void sc850sl_set_exp_gain(uint32_t lines, uint32_t gain_x1024)
{
    if (!g_cam) return;
    if (lines < SC850SL_EXP_MIN) lines = SC850SL_EXP_MIN;
    if (lines > SC850SL_EXP_MAX) lines = SC850SL_EXP_MAX;

    /* Exposure: 20-bit field carries lines<<4 (low nibble = 1/16-line frac).
     * Use the FAST (no-bus-reset) write and bail on the first failure: this is
     * a per-frame hot path, and a stuck SC850SL I2C bus must not block the
     * stream task — the normal write's retry+bus-reset storm did exactly that
     * and tripped the task watchdog into a reboot loop. AE just skips this
     * update and tries again next cycle. */
    uint32_t v = lines << 4;
    if (sc850sl_write_reg_fast(g_cam, 0x3e00, (v >> 16) & 0x0f) != ESP_OK) return;
    sc850sl_write_reg_fast(g_cam, 0x3e01, (v >>  8) & 0xff);
    sc850sl_write_reg_fast(g_cam, 0x3e02, (v >>  0) & 0xf0);

    /* Gain: choose the coarse DCG+analog code and DCG base, then the fine
     * mantissa = 64*gain/(dcg*coarse), which lands in [0x40,0x7f].
     * Breakpoints + 0x3e08 codes verified register-for-register against the
     * CVITEK cv183x driver's AgainInfo[] table (reference/sophgo_cv183x_sc850sl). */
    uint32_t g = gain_x1024;
    if (g < SC850SL_GAIN_UNITY) g = SC850SL_GAIN_UNITY;
    if (g > SC850SL_GAIN_MAX)   g = SC850SL_GAIN_MAX;

    /* Digital-gain relay (night-only headroom). The analog ladder tops out at
     * 49.6x (SC850SL_AGAIN_MAX). Past that, rail analog and make up the rest
     * with the sensor's digital gain on 0x3e06 (CVITEK cmos_dgain_calc_table:
     * 0x00/0x01/0x03/0x07 = 1x/2x/4x/8x, coarse 6 dB steps). dig is x1024. */
    uint32_t dig = 1024;
    if (g > SC850SL_AGAIN_MAX) {
        dig = (uint32_t)((uint64_t)g * 1024u / SC850SL_AGAIN_MAX);
        g   = SC850SL_AGAIN_MAX;
    }

    uint8_t  coarse_reg;
    uint32_t dcg, coarse;
    if      (g < 2048)  { coarse = 1; dcg = 1024; coarse_reg = 0x03; }
    else if (g < 3200)  { coarse = 2; dcg = 1024; coarse_reg = 0x07; }
    else if (g < 6400)  { coarse = 1; dcg = 3200; coarse_reg = 0x23; }
    else if (g < 12800) { coarse = 2; dcg = 3200; coarse_reg = 0x27; }
    else if (g < 25600) { coarse = 4; dcg = 3200; coarse_reg = 0x2f; }
    else                { coarse = 8; dcg = 3200; coarse_reg = 0x3f; }
    uint32_t fine = (64u * g) / (dcg * coarse);
    if (fine < 0x40) fine = 0x40;
    if (fine > 0x7f) fine = 0x7f;

    /* Digital gain register (coarse 2^n, per CVITEK). 0x3e07 is the fine
     * digital mantissa; pin it to unity (0x80) whenever digital is engaged so
     * we don't ride an unknown power-on value (the M5Stack init never writes
     * 0x3e07; the CVITEK init sets it 0x80). */
    uint8_t dgain_reg = (dig < 2048) ? 0x00 :
                        (dig < 4096) ? 0x01 :
                        (dig < 8192) ? 0x03 : 0x07;

    /* DPC strength vs gain (mirror CVITEK cmos_gains_update): the init table
     * leaves 0x363c=0x07 (aggressive defect-pixel correction, right for high
     * gain); below 2.0x that can over-soften, so drop to 0x04. Write only on
     * change to spare the flaky SC850SL I2C bus. */
    static uint8_t s_dpc_last = 0x07;  /* = init-table value */
    uint8_t dpc = (g >= 2048u) ? 0x07 : 0x04;
    if (dpc != s_dpc_last && sc850sl_write_reg_fast(g_cam, 0x363c, dpc) == ESP_OK)
        s_dpc_last = dpc;

    sc850sl_write_reg_fast(g_cam, 0x3e06, dgain_reg);
    if (dgain_reg != 0x00) sc850sl_write_reg_fast(g_cam, 0x3e07, 0x80);
    sc850sl_write_reg_fast(g_cam, 0x3e08, coarse_reg);
    sc850sl_write_reg_fast(g_cam, 0x3e09, (uint8_t)fine);
}

/* One host-AE step, exposure-priority. Brighten by raising exposure first
 * (best SNR), then gain once exposure is railed; darken by dropping gain to
 * 1x first, then shortening exposure. Per-step ratio is clamped for smooth
 * convergence; with anti-flicker on, exposure snaps to whole light-ripple
 * periods. Called once per captured frame with the raw-frame mean (8-bit).
 *
 * Re-enabled June 2026: exposure/gain writes finally take effect now that
 * sc850sl_set_exp_gain() dropped the bogus 0x3800 group-hold and we removed
 * the boot 0x3e03=0x0b write (see task #22). */
static void ae_step(int raw_mean, int sat_ppm)
{
    if (raw_mean < 2) return;                         /* no signal yet */

    /* Highlight guard (daytime overexposure fix), highest priority. If too
     * many pixels sit at/near full scale, pull brightness DOWN regardless of
     * the mean -- clipped highlights (cloud/ice/sun-glint in a LEO daytime
     * scene) are unrecoverable, while the darker midtones we trade away are
     * lifted by the 1/2.2 gamma downstream. Shed gain first (it only amplifies
     * the clip), then shorten exposure; step harder than the symmetric AE ratio
     * so we escape saturation within a couple of (throttled) cycles. */
    if (sat_ppm > (int)AE_SAT_BUDGET_PPM) {
        uint32_t exp = g_ae_exp_lines, gain = g_ae_gain_x1024;
        if (gain > SC850SL_GAIN_UNITY) {
            gain = (gain * 80u) / 100u;               /* -20% gain */
            if (gain < SC850SL_GAIN_UNITY) gain = SC850SL_GAIN_UNITY;
        } else {
            exp = (exp * 80u) / 100u;                 /* -20% exposure */
            if (exp < SC850SL_EXP_MIN) exp = SC850SL_EXP_MIN;
        }
        g_ae_exp_lines = exp; g_ae_gain_x1024 = gain;
        sc850sl_set_exp_gain(exp, gain);
        return;
    }

    int err = AE_TARGET_RAW - raw_mean;
    if (err > -AE_DEAD_RAW && err < AE_DEAD_RAW) return;   /* close enough */

    float ratio = (float)AE_TARGET_RAW / (float)raw_mean;
    if (ratio > 2.0f) ratio = 2.0f;                   /* clamp per-step change */
    if (ratio < 0.5f) ratio = 0.5f;

    uint32_t exp  = g_ae_exp_lines;
    uint32_t gain = g_ae_gain_x1024;

    if (ratio > 1.0f) {                               /* need brighter */
        if (exp < SC850SL_EXP_MAX) {                  /* raise exposure first */
            uint32_t ne = (uint32_t)(exp * ratio + 0.5f);
            if (ne > SC850SL_EXP_MAX) ne = SC850SL_EXP_MAX;
            exp = ne;
        } else {                                      /* exposure railed → gain */
            uint32_t ng = (uint32_t)(gain * ratio + 0.5f);
            if (ng > SC850SL_GAIN_MAX) ng = SC850SL_GAIN_MAX;
            gain = ng;
        }
    } else {                                          /* need darker */
        if (gain > SC850SL_GAIN_UNITY) {              /* drop gain first */
            uint32_t ng = (uint32_t)(gain * ratio + 0.5f);
            if (ng < SC850SL_GAIN_UNITY) ng = SC850SL_GAIN_UNITY;
            gain = ng;
        } else {                                      /* gain at 1x → shorten exp */
            uint32_t ne = (uint32_t)(exp * ratio + 0.5f);
            if (ne < SC850SL_EXP_MIN) ne = SC850SL_EXP_MIN;
            exp = ne;
        }
    }

    /* Anti-flicker: snap exposure to whole ripple periods, but only once it's
     * at least one period; below that (very bright AC scene) let it run
     * continuous and accept some banding rather than overexpose. */
    if (g_antiflicker_hz) {
        uint32_t fl = (g_antiflicker_hz == 60) ? SC850SL_FLICKER_LINES_60
                                               : SC850SL_FLICKER_LINES_50;
        if (exp >= fl) {
            uint32_t m = (exp + fl / 2) / fl;
            if (m < 1) m = 1;
            exp = m * fl;
            if (exp > SC850SL_EXP_MAX) exp = (SC850SL_EXP_MAX / fl) * fl;
        }
    }

    g_ae_exp_lines  = exp;
    g_ae_gain_x1024 = gain;
    sc850sl_set_exp_gain(exp, gain);
}

/* OLD ISP-AE task — disabled. We bypass the ISP, so there are no ISP AE
 * statistics; auto-exposure now runs in usb_stream_send via ae_step()
 * using the raw-frame mean. Kept behind #if 0 for reference. */
#if 0
static void ae_task(void *arg)
{
    (void)arg;
    /* Settling delay — let CSI / ISP produce a few frames first. */
    vTaskDelay(pdMS_TO_TICKS(500));

    /* Apply the boot exposure so we're starting from a known state. */
    sc850sl_set_exposure(g_exposure);

    while (1) {
        isp_ae_result_t res;
        esp_err_t r = esp_isp_ae_controller_get_oneshot_statistics(
            g_ae, AE_PERIOD_MS, &res);
        if (r != ESP_OK) {
            /* Stats not ready (no CSI activity yet?) — back off. */
            vTaskDelay(pdMS_TO_TICKS(AE_PERIOD_MS));
            continue;
        }

        /* Average luminance across all 5×5 blocks. */
        int64_t sum = 0;
        int     n   = 0;
        for (int y = 0; y < ISP_AE_BLOCK_Y_NUM; ++y) {
            for (int x = 0; x < ISP_AE_BLOCK_X_NUM; ++x) {
                sum += res.luminance[x][y];
                ++n;
            }
        }
        int avg = (n > 0) ? (int)(sum / n) : 0;

        /* Decide direction. Deadband of ±AE_DEADBAND around target
         * prevents oscillation when we're "close enough". */
        int delta = AE_TARGET_LUMA - avg;
        if (delta > AE_DEADBAND || delta < -AE_DEADBAND) {
            /* Proportional step. Avoid divide-by-zero / huge ratios. */
            float ratio = (avg > 4) ? (float)AE_TARGET_LUMA / (float)avg
                                    : 4.0f;
            /* Clamp the per-step ratio. */
            if (ratio > 1.25f) ratio = 1.25f;
            if (ratio < 0.80f) ratio = 0.80f;
            int next = (int)((float)g_exposure * ratio + 0.5f);
            if (next < AE_EXPOSURE_MIN) next = AE_EXPOSURE_MIN;
            if (next > AE_EXPOSURE_MAX) next = AE_EXPOSURE_MAX;
            if (next != g_exposure) {
                g_exposure = (uint16_t) next;
                sc850sl_set_exposure(g_exposure);
                ESP_LOGI(TAG, "AE: avg=%3d (target %d), exposure → 0x%03x",
                         avg, AE_TARGET_LUMA, g_exposure);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(AE_PERIOD_MS));
    }
}
#endif  /* disabled ISP-AE task */

/* Crop the SC850SL output to CAPTURE_W × CAPTURE_H (centered) so the
 * sensor delivers exactly the resolution the CSI/ISP pipeline is
 * configured for. Must be called after the init table has loaded. */
static esp_err_t camera_sensor_set_crop(void)
{
    if (!g_cam) return ESP_ERR_INVALID_STATE;
    esp_err_t r = sc850sl_set_window(g_cam, SENSOR_CROP_W, SENSOR_CROP_H,
                                     SENSOR_CROP_X_START, SENSOR_CROP_Y_START);
    if (r == ESP_OK) {
        ESP_LOGI(TAG, "sensor cropped to %d×%d @ (%d, %d)",
                 SENSOR_CROP_W, SENSOR_CROP_H,
                 SENSOR_CROP_X_START, SENSOR_CROP_Y_START);
    }
    return r;
}

/* Block until the next frame is captured into g_capture_buf, then
 * invalidate cache so the CPU sees the fresh DMA data. */
static esp_err_t camera_capture_frame(uint16_t *out_rgb565)
{
    if (!g_pipeline_ready) {
        return ESP_ERR_INVALID_STATE;
    }
    /* Serialize CSI access — sstv_tx_task and usb_stream_task both call
     * this, so the receive() chain has to be exclusive. */
    if (g_capture_mutex && xSemaphoreTake(g_capture_mutex, portMAX_DELAY) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }
    /* The CSI controller is running continuously, writing every frame
     * to g_capture_buf (returned by csi_on_get_new_trans). We just
     * wait for the next on_trans_finished signal, then the latest
     * frame is in the buffer — no receive() call needed. */
    /* Drain any stale signal from a previous frame we didn't consume. */
    xSemaphoreTake(g_csi_frame_done, 0);
    if (xSemaphoreTake(g_csi_frame_done, pdMS_TO_TICKS(2000)) != pdTRUE) {
        ESP_LOGW(TAG, "csi: no frame-done within 2000 ms (isr_count=%u)",
                 (unsigned) g_csi_isr_count);
        if (g_capture_mutex) xSemaphoreGive(g_capture_mutex);
        return ESP_ERR_TIMEOUT;
    }
    /* DMA already invalidated cache for us inside the ISR before
     * calling on_trans_finished (see csi_dma_trans_done_callback),
     * but issue an explicit invalidate on our consumer-side address
     * to be defensive against caller buffer aliases. */
    esp_cache_msync(out_rgb565, CAPTURE_FB_BYTES,
                    ESP_CACHE_MSYNC_FLAG_DIR_M2C);
    if (g_capture_mutex) xSemaphoreGive(g_capture_mutex);
    return ESP_OK;
}

/* Push one captured frame out the USB-Serial-JTAG CDC channel for live
 * host-side viewing. Uses a small magic prefix so a Python decoder can
 * locate frames in the byte stream regardless of interleaved ESP_LOGx
 * output (logs are ASCII, our magic includes nulls so no collision). */
static void usb_stream_send(int cycle)
{
    if (!g_pipeline_ready || !g_usb_stream_buf) {
        ESP_LOGW(TAG, "usb_stream: pipeline not ready");
        return;
    }

    /* Per-stage timing (esp_timer, microseconds): splits the frame budget
     * across sensor-wait / software-ISP / USB. Cheap (~1 us/call); only the
     * deltas get logged, on the 1/30 throttle below. Useful for sizing the
     * multicore flight schedule and as a baseline to compare any future
     * hardware-ISP path against. */
    int64_t t_cap0 = esp_timer_get_time();

    /* Capture a fresh frame, then downscale to 320×240 for the stream. */
    esp_err_t r = camera_capture_frame(g_capture_buf);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "usb_stream[%d]: capture failed: %s",
                 cycle, esp_err_to_name(r));
        return;
    }
    int64_t t_cap1 = esp_timer_get_time();   /* sensor-wait + DMA transfer */
    /* Diagnostic: scan the raw capture buffer for min/max/mean so we can
     * tell underexposure (all near 0) from a data-extraction bug (exactly
     * 0 everywhere) from a healthy signal (spread of values). Sample the
     * high bytes across the whole RAW10-packed frame (every 5th byte =
     * the LSB-packing byte is skipped; we step by a stride to keep it
     * cheap). */
    int raw_mean = 0, raw_min = 255, raw_max = 0, raw_sat_ppm = 0;
    {
        const uint8_t *raw = (const uint8_t *)g_capture_buf;
        size_t raw_bytes = (size_t)CAPTURE_W * CAPTURE_H * 10 / 8;
        uint8_t mn = 255, mx = 0;
        uint64_t sum = 0; uint32_t cnt = 0, sat = 0;
        for (size_t i = 0; i < raw_bytes; i += 97) {  /* prime stride */
            if ((i % 5) == 4) continue;   /* skip RAW10 LSB-packing byte, so
                                           * min/mean reflect real pixel
                                           * values — this is what you read
                                           * off a lens-capped dark frame to
                                           * calibrate g_black_level */
            uint8_t v = raw[i];
            if (v < mn) mn = v;
            if (v > mx) mx = v;
            if (v >= AE_SAT_LEVEL) ++sat;   /* near-full-scale (clipping) pixel */
            sum += v; ++cnt;
        }
        raw_mean = cnt ? (int)(sum / cnt) : 0;
        raw_min  = mn;
        raw_max  = mx;
        raw_sat_ppm = cnt ? (int)((uint64_t)sat * 1000000u / cnt) : 0;
        /* min/max/mean now ride in the frame header (host overlays them), so
         * there's no per-frame log line here — that keeps the wire clean
         * between frames. A throttled summary still prints 1/30 below. */
    }

    /* Host auto-exposure: drive the sensor's exposure + analog gain toward the
     * target raw mean (clean 10-bit analog domain, vs the old software-gain
     * multiply that posterized). Throttled to every 4th frame — the scene
     * doesn't change fast, and it bounds the per-frame I2C write load on the
     * flaky SC850SL bus (which, hammered every frame, was stalling and
     * watchdog-resetting the board). */
    if ((cycle % 4) == 0) ae_step(raw_mean, raw_sat_ppm);

    /* Composite layout: RGB visible (left pane) | thermal (right pane).
     * Both panes are USB_PANE_W × USB_PANE_H inside a USB_STREAM_W-wide
     * buffer, so each renderer writes with dst_stride = USB_STREAM_W. */

    /* LEFT pane: RGB. g_capture_buf is RAW10 Bayer (RGGB); demosaic +
     * box-filter + AWB + gamma straight into the left half. This call IS the
     * "software ISP" whose per-frame cost we're timing (t_ds1 - t_ds0). */
    int64_t t_ds0 = esp_timer_get_time();
    downscale_raw10_bggr_to_rgb565((const uint8_t *)g_capture_buf,
                                   CAPTURE_W, CAPTURE_H,
                                   g_usb_stream_buf,            /* left pane @ x=0 */
                                   USB_PANE_W, USB_PANE_H,
                                   USB_STREAM_W);
    int64_t t_ds1 = esp_timer_get_time();

    /* RIGHT pane: thermal. Capture an MI1602 frame, false-color + scale
     * into the right half. If the IR module isn't present / a capture
     * fails, paint the pane dark so the composite still streams. */
    uint16_t *thermal_pane = g_usb_stream_buf + USB_PANE_W;    /* right pane @ x=PANE_W */
    esp_err_t tr = mi1602_aux_capture_rgb565(thermal_pane,
                                             USB_PANE_W, USB_PANE_H,
                                             USB_STREAM_W);
    if (tr != ESP_OK) {
        for (int y = 0; y < USB_PANE_H; ++y) {
            uint16_t *row = thermal_pane + (size_t)y * USB_STREAM_W;
            for (int x = 0; x < USB_PANE_W; ++x) row[x] = 0x0000;  /* black */
        }
    }

    /* Build the per-frame header in-place at the front of g_frame_buf. The
     * pixels are already rendered into the payload region and the trailer
     * sentinel was written once at init, so the buffer is a complete frame.
     * Push it with a SINGLE fwrite: newlib holds the stdout file lock for the
     * whole call, making the frame atomic against any other task's ESP_LOGx
     * (that mid-frame interleaving was the smear / false-colour-stripe cause). */
    uint8_t *H = g_frame_buf;
    memcpy(H, USB_STREAM_MAGIC, sizeof(USB_STREAM_MAGIC));
    put_u16le(H + 8,  USB_STREAM_W);
    put_u16le(H + 10, USB_STREAM_H);
    put_u16le(H + 12, (uint16_t)raw_min);
    put_u16le(H + 14, (uint16_t)raw_max);
    put_u16le(H + 16, (uint16_t)raw_mean);
    put_u16le(H + 18, (uint16_t)g_black_level);
    put_u16le(H + 20, (uint16_t)(g_wb_r * 256.0f + 0.5f));
    put_u16le(H + 22, (uint16_t)(g_wb_b * 256.0f + 0.5f));

    int64_t t_usb0 = esp_timer_get_time();
    size_t n = fwrite(g_frame_buf, 1, USB_FRAME_TOTAL, stdout);
    fflush(stdout);
    int64_t t_usb1 = esp_timer_get_time();

    /* Throttled summary (1/30 frames). It lands in the gap AFTER a frame's
     * trailer and BEFORE the next magic, so the host's magic search skips it
     * cleanly — keeps headless serial-capture debugging possible. */
    if ((cycle % 30) == 0) {
        ESP_LOGI(TAG, "usb_stream[%d]: %u B wrote=%zu  raw min=%d max=%d mean=%d sat=%dppm "
                 "exp=%u gainx1024=%u bl=%d wb r=%.2f b=%.2f thermal=%s "
                 "| t cap=%lldus isp=%lldus usb=%lldus",
                 cycle, (unsigned)USB_FRAME_TOTAL, n, raw_min, raw_max, raw_mean, raw_sat_ppm,
                 (unsigned)g_ae_exp_lines, (unsigned)g_ae_gain_x1024,
                 g_black_level, g_wb_r, g_wb_b, tr == ESP_OK ? "ok" : "none",
                 (long long)(t_cap1 - t_cap0), (long long)(t_ds1 - t_ds0),
                 (long long)(t_usb1 - t_usb0));
    }
}

static void usb_stream_task(void *arg)
{
    (void)arg;
    /* Make stdout binary-clean for the frame stream:
     *   1. setvbuf(_IONBF): no stdio buffering — writes flush immediately.
     *   2. set_tx_line_endings(LF): DISABLE the console's default LF->CRLF
     *      translation. That default expands every 0x0A byte in our binary
     *      payload to 0x0D 0x0A, inserting a byte; the inserted count depends
     *      on pixel content, so bright/varied frames drift out of sync (the
     *      host's trailer check then lands on pixel data) while an all-black
     *      covered frame — which has no 0x0A bytes — streams fine. THAT was
     *      the brightness-correlated frame loss; setvbuf alone never fixed it
     *      because line-ending translation lives in the VFS, not stdio. */
    setvbuf(stdout, NULL, _IONBF, 0);
    usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_LF);

    /* Settling delay so first frame isn't from a half-initialized ISP. */
    vTaskDelay(pdMS_TO_TICKS(2000));
    int cycle = 0;
    while (1) {
        usb_stream_send(++cycle);
        vTaskDelay(pdMS_TO_TICKS(USB_STREAM_PERIOD_MS));
    }
}

/* ---------------------------------------------------------------- *
 *  Frame → Robot36 downscale (1920×1080 → 320×240)
 * ---------------------------------------------------------------- */

/* Old: RGB565 → RGB565 nearest-neighbor (used when ISP outputs RGB565). */
static void downscale_to_robot36(const uint16_t *src, int sw, int sh,
                                 uint16_t *dst, int dw, int dh)
{
    for (int y = 0; y < dh; ++y) {
        int sy = (y * sh) / dh;
        const uint16_t *srow = src + sy * sw;
        uint16_t       *drow = dst + y * dw;
        for (int x = 0; x < dw; ++x) {
            int sx = (x * sw) / dw;
            drow[x] = srow[sx];
        }
    }
}

/* RAW10 (BGGR Bayer, packed: 5 bytes per 4 pixels) → RGB565 with downscale.
 * Uses the 8 most-significant bits of each 10-bit sample (drops the 2 LSBs
 * stored in byte 4 of each 5-byte group — they don't add much for a 320×240
 * preview and skipping them keeps the inner loop tight). Demosaic is the
 * simplest possible: for each output pixel, find the 2x2 Bayer cell at the
 * source position and read B/G/R from its four samples.
 *
 *   Source BGGR layout (each pair of rows):
 *     row N (even)   : B G B G B G ...
 *     row N+1 (odd)  : G R G R G R ...
 */
static inline uint8_t raw10_get8(const uint8_t *row, int x)
{
    // 5-byte block per 4 pixels. High byte of pixel x is at:
    //   row[(x/4)*5 + (x%4)]
    return row[(x >> 2) * 5 + (x & 3)];
}

/* ---- Software ISP-lite: black-level + gray-world AWB + CCM + gamma ---
 * The hardware ISP would do these, but its memory-input (DWGDMA) mode is
 * "not supported yet" in IDF v6.0.1 and its CSI path needs a ≤1080p
 * sensor mode we don't have. So we do a minimal version in software.
 *
 * Auto white balance (gray-world): each frame we measure the mean R/G/B
 * and set per-channel gains so the means equalize (green = reference).
 * Self-corrects to any illuminant — no hand-tuned constants. Applied
 * with one frame of lag (this frame's measurement drives next frame's
 * gains), IIR-smoothed so it doesn't pulse. g_wb_r/g_wb_b are declared
 * up in the globals block (usb_stream_send logs them). */

/* sRGB-ish gamma encode (1/2.2). Raw sensor data is linear and looks
 * dark/wrong on a display; gamma lifts the mid-tones to a natural look
 * (also brightens our ~mean-65 frames perceptually). */
static uint8_t g_gamma_lut[256];
static bool    g_gamma_ready = false;

static void gamma_lut_init(void)
{
    for (int i = 0; i < 256; ++i) {
        float c = powf(i / 255.0f, 1.0f / 2.2f);
        int v = (int)(c * 255.0f + 0.5f);
        g_gamma_lut[i] = (uint8_t)(v > 255 ? 255 : v);
    }
    g_gamma_ready = true;
}

/* RAW10 (RGGB Bayer, packed 5 bytes / 4 pixels) -> RGB565 with a
 * BOX-FILTER downscale: each output pixel averages ALL source samples in
 * its footprint (separately per Bayer color), which low-passes away the
 * rainbow aliasing the old nearest-neighbor picker produced on detailed
 * rows. Uses the high 8 bits of each 10-bit sample.
 *
 * Per-pixel colour pipeline (linear light until the very last step):
 *   demosaic -> black-level subtract + range-restore -> WB gain (diagonal)
 *   -> CCM (3x3) -> gamma (sRGB-ish 1/2.2) -> pack RGB565
 * Gray-world AWB is updated once per frame from the BL-corrected means. */
static void downscale_raw10_bggr_to_rgb565(const uint8_t *src, int sw, int sh,
                                           uint16_t *dst, int dw, int dh,
                                           int dst_stride)
{
    if (!g_gamma_ready) gamma_lut_init();
    const int src_row_bytes = (sw * 10) / 8;
    /* Frame-wide channel accumulators for next-frame gray-world AWB. */
    uint64_t fr = 0, fg = 0, fb = 0;
    const float wb_r = g_wb_r, wb_b = g_wb_b;   /* gains from prev frame */

    /* Black-level pedestal + the scale that restores [bl..255] -> [0..255]
     * after subtracting it (recovers the contrast the pedestal was eating),
     * with the digital-exposure gain folded in (g_dig_gain = 1.0 -> no-op). */
    const float bl       = (float)g_black_level;
    const float bl_den   = (255.0f - bl) > 1.0f ? (255.0f - bl) : 1.0f;
    const float bl_scale = (255.0f / bl_den) * g_dig_gain;
    /* Cache the CCM rows in locals for the inner loop. */
    const float m00 = g_ccm[0][0], m01 = g_ccm[0][1], m02 = g_ccm[0][2];
    const float m10 = g_ccm[1][0], m11 = g_ccm[1][1], m12 = g_ccm[1][2];
    const float m20 = g_ccm[2][0], m21 = g_ccm[2][1], m22 = g_ccm[2][2];

    for (int dy = 0; dy < dh; ++dy) {
        int sy0 = (dy * sh) / dh;
        int sy1 = ((dy + 1) * sh) / dh;
        if (sy1 < sy0 + 2) sy1 = sy0 + 2;          /* ≥1 full Bayer cell */
        if (sy1 > sh) sy1 = sh;
        uint16_t *drow = dst + (size_t)dy * dst_stride;
        for (int dx = 0; dx < dw; ++dx) {
            int sx0 = (dx * sw) / dw;
            int sx1 = ((dx + 1) * sw) / dw;
            if (sx1 < sx0 + 2) sx1 = sx0 + 2;
            if (sx1 > sw) sx1 = sw;

            uint32_t sr = 0, sg = 0, sb = 0;
            uint32_t cr = 0, cg = 0, cb = 0;
            for (int sy = sy0; sy < sy1; ++sy) {
                const uint8_t *row = src + (size_t)sy * src_row_bytes;
                int yodd = sy & 1;
                for (int sx = sx0; sx < sx1; ++sx) {
                    uint8_t v = raw10_get8(row, sx);
                    int xodd = sx & 1;
                    /* RGGB: (even,even)=R (odd,even)=G (even,odd)=G (odd,odd)=B.
                     * (Was BGGR — but that rendered a blue mask as yellow and
                     * skin as blue, the classic R/B-swap signature, so the
                     * sensor's actual order at our readout is RGGB.) */
                    if (!yodd && !xodd)      { sr += v; ++cr; }
                    else if (yodd && xodd)   { sb += v; ++cb; }
                    else                     { sg += v; ++cg; }
                }
            }
            uint32_t ar = cr ? sr / cr : 0;
            uint32_t ag = cg ? sg / cg : 0;
            uint32_t ab = cb ? sb / cb : 0;

            /* (1) Black-level subtract + range-restore, in linear space. */
            float lr = ((float)ar - bl) * bl_scale;
            float lg = ((float)ag - bl) * bl_scale;
            float lb = ((float)ab - bl) * bl_scale;
            if (lr < 0.0f) lr = 0.0f;
            if (lg < 0.0f) lg = 0.0f;
            if (lb < 0.0f) lb = 0.0f;

            /* Gray-world AWB measures the BL-corrected linear means. A
             * uniform rescale doesn't change channel ratios, but removing
             * the additive pedestal does, so WB converges cleaner now. */
            fr += (uint32_t)lr; fg += (uint32_t)lg; fb += (uint32_t)lb;

            /* (2) White-balance gain (diagonal), still linear. Green is the
             * reference channel (gain 1.0), so lg is left untouched. */
            lr *= wb_r;
            lb *= wb_b;

            /* (3) CCM: sensor-RGB -> sRGB. Off-diagonal terms undo the
             * spectral cross-talk WB gain can't, restoring saturation. */
            float xr = m00 * lr + m01 * lg + m02 * lb;
            float xg = m10 * lr + m11 * lg + m12 * lb;
            float xb = m20 * lr + m21 * lg + m22 * lb;
            int ir = (int)(xr + 0.5f);
            int ig = (int)(xg + 0.5f);
            int ib = (int)(xb + 0.5f);
            if (ir < 0) ir = 0;
            if (ig < 0) ig = 0;
            if (ib < 0) ib = 0;
            if (ir > 255) ir = 255;
            if (ig > 255) ig = 255;
            if (ib > 255) ib = 255;

            /* (4) Gamma encode LAST, on the CCM output (so CCM operated on
             * linear light), then pack to RGB565. */
            uint8_t r = g_gamma_lut[ir];
            uint8_t g = g_gamma_lut[ig];
            uint8_t b = g_gamma_lut[ib];
            drow[dx] = (uint16_t)(((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3));
        }
    }

    /* Gray-world update for the NEXT frame: drive channel means together
     * (green = reference). Clamp to a sane range and IIR-smooth. */
    if (fr > 0 && fb > 0 && fg > 0) {
        float gr = (float)fg / (float)fr;
        float gb = (float)fg / (float)fb;
        if (gr < 0.5f) gr = 0.5f;
        if (gr > 4.0f) gr = 4.0f;
        if (gb < 0.5f) gb = 0.5f;
        if (gb > 4.0f) gb = 4.0f;
        g_wb_r = g_wb_r * 0.7f + gr * 0.3f;
        g_wb_b = g_wb_b * 0.7f + gb * 0.3f;
    }
}

/* ---------------------------------------------------------------- *
 *  Periodic SSTV TX task
 * ---------------------------------------------------------------- */

static void sstv_tx_task(void *arg)
{
    (void)arg;
    /* Initial settling delay so the sensor + audio chain finish
     * spinning up before the first transmission. */
    vTaskDelay(pdMS_TO_TICKS(3000));

    int cycle = 0;
    while (1) {
        ++cycle;
        ESP_LOGI(TAG, "── SSTV TX cycle %d ──", cycle);

        /* 1. Capture: pull the latest frame from the CSI/ISP DMA path
         *    (1920×1080 RGB565). If the pipeline isn't ready (init
         *    failed for some reason), fall back to the embedded
         *    static test image so we can still verify the audio path. */
        esp_err_t r;
        if (g_pipeline_ready) {
            r = camera_capture_frame(g_capture_buf);
            if (r == ESP_OK) {
                downscale_raw10_bggr_to_rgb565((const uint8_t *)g_capture_buf,
                                               CAPTURE_W, CAPTURE_H,
                                               g_sstv_image,
                                               SSTV_ROBOT36_WIDTH,
                                               SSTV_ROBOT36_HEIGHT,
                                               SSTV_ROBOT36_WIDTH);
            } else {
                ESP_LOGW(TAG, "capture failed (%s) — using static test image",
                         esp_err_to_name(r));
                memcpy(g_sstv_image, test_image_rgb565,
                       sizeof(test_image_rgb565));
            }
        } else {
            memcpy(g_sstv_image, test_image_rgb565,
                   sizeof(test_image_rgb565));
            r = ESP_OK;
        }

        /* 3. Encode. */
        size_t nsamp = 0;
        r = sstv_robot36_encode(g_sstv_image, AUDIO_SR_HZ,
                                g_pcm_buf, PCM_CAPACITY, &nsamp);
        if (r != ESP_OK) {
            ESP_LOGE(TAG, "encode failed: %s", esp_err_to_name(r));
            goto sleep;
        }
        ESP_LOGI(TAG, "encoded %u samples (%.2f s)",
                 (unsigned) nsamp, (double) nsamp / AUDIO_SR_HZ);

        /* 4. Push through I²S to the PCM5102A. */
        size_t total_written = 0;
        const size_t bytes_total = nsamp * sizeof(int16_t);
        while (total_written < bytes_total) {
            size_t written = 0;
            r = i2s_channel_write(g_i2s_tx,
                                  ((uint8_t *) g_pcm_buf) + total_written,
                                  bytes_total - total_written,
                                  &written, portMAX_DELAY);
            if (r != ESP_OK) {
                ESP_LOGE(TAG, "i2s_channel_write: %s", esp_err_to_name(r));
                break;
            }
            total_written += written;
        }
        ESP_LOGI(TAG, "✓ TX done (%zu bytes); sleeping %d s",
                 total_written, SSTV_INTERVAL_SECS);

sleep:
        vTaskDelay(pdMS_TO_TICKS(SSTV_INTERVAL_SECS * 1000));
    }
}

/* ---------------------------------------------------------------- *
 *  Entry
 * ---------------------------------------------------------------- */

/* OTA anti-brick: a freshly OTA-updated image boots in PENDING_VERIFY. We
 * confirm it valid only after a short burn-in (it reached steady state without
 * crashing); if it crashes/hangs before that it never confirms, and the
 * bootloader rolls back to the previous slot on the next reset. See
 * docs/FLIGHT_ARCHITECTURE.md §12. No-op on a normally (esptool-)flashed image. */
static void ota_confirm_pending(void)
{
    const esp_partition_t *run = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (run && esp_ota_get_state_partition(run, &st) == ESP_OK
        && st == ESP_OTA_IMG_PENDING_VERIFY) {
        esp_err_t e = esp_ota_mark_app_valid_cancel_rollback();
        ESP_LOGI(TAG, "OTA: running image confirmed valid (burn-in passed): %s",
                 esp_err_to_name(e));
    }
}

void app_run(void)
{
    ESP_LOGI(TAG, "CubeSat imager booting (SC850SL flight target)");
    heartbeat_setup(CONFIG_SC850SL_HEARTBEAT_LED_GPIO);

    /* ---- PSRAM allocations ---- */
    g_pcm_buf = heap_caps_malloc(PCM_CAPACITY * sizeof(int16_t),
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    g_sstv_image = heap_caps_malloc(SSTV_ROBOT36_WIDTH * SSTV_ROBOT36_HEIGHT * 2,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    /* One contiguous buffer holds [header][payload][trailer] so the frame can
     * be pushed atomically with a single fwrite. The pixel renderers only see
     * the payload region, via g_usb_stream_buf. The trailer never changes, so
     * write it once here. */
    g_frame_buf = heap_caps_malloc(USB_FRAME_TOTAL,
                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (g_frame_buf) {
        g_usb_stream_buf = (uint16_t *)(g_frame_buf + USB_HDR_BYTES);
        memcpy(g_frame_buf + USB_HDR_BYTES + USB_STREAM_BYTES,
               USB_STREAM_TRAILER, USB_TRAILER_BYTES);
    }
    g_capture_mutex = xSemaphoreCreateMutex();
    if (!g_pcm_buf || !g_sstv_image || !g_frame_buf || !g_capture_mutex) {
        ESP_LOGE(TAG, "PSRAM alloc for audio/sstv/stream buffers failed");
        goto idle;
    }

    /* ---- I²C + sensor bring-up ---- */
    i2c_master_bus_handle_t bus = i2c_bus_open();

    /* Defensive: kick the I²C peripheral out of any stuck state left over
     * from a previous power cycle (slave holding SCL low waiting for the
     * next clock pulse, etc.). i2c_master_bus_reset() pulses SCL 9 times
     * with a STOP condition afterwards — standard I²C bus recovery. */
    esp_err_t rrst = i2c_master_bus_reset(bus);
    if (rrst != ESP_OK) {
        ESP_LOGW(TAG, "i2c_master_bus_reset: %s (continuing anyway)",
                 esp_err_to_name(rrst));
    } else {
        ESP_LOGI(TAG, "I²C bus reset OK (9-clock recovery sequence)");
    }

    sc850sl_config_t cfg = {
        .i2c_bus       = bus,
        .i2c_addr      = SC850SL_I2C_ADDR_DEFAULT,
        .i2c_freq_hz   = CONFIG_SC850SL_I2C_FREQ_HZ,
        .xshutdn_gpio  = CONFIG_SC850SL_XSHUTDN_GPIO,
        .extclk_gpio   = CONFIG_SC850SL_EXTCLK_GPIO,
        .extclk_hz     = CONFIG_SC850SL_EXTCLK_HZ,
        .rail_en_dovdd = -1,
        .rail_en_dvdd  = -1,
        .rail_en_avdd  = -1,
        .mode          = (sc850sl_mode_t) CONFIG_SC850SL_DEFAULT_MODE,
    };
    /* Init the sensor BEFORE the diagnostic bus scan so XSHUTDN is high
     * and the SC850SL is actually ready to respond on its I²C address
     * when the scan runs. Previously the scan ran first (sensor still
     * in reset) which gave us either ghost-ACKs (when the rail leakage
     * happened to be permissive) or all-timeouts (now). */
    esp_err_t r = sc850sl_init(&cfg, &g_cam);
    if (r != ESP_OK) {
        ESP_LOGE(TAG, "sc850sl_init early: %s", esp_err_to_name(r));
    }

    /* Diagnostic bus scan (cheap; helps debug carrier wiring). */
    {
        ESP_LOGI(TAG, "I²C bus scan (this should find 0x30 = SC850SL):");
        int found = 0;
        for (uint8_t a = 0x08; a < 0x78; ++a) {
            if (i2c_master_probe(bus, a, 50) == ESP_OK) {
                ESP_LOGI(TAG, "  device found at 0x%02x", a);
                ++found;
            }
        }
        if (found == 0) {
            ESP_LOGW(TAG, "  no devices ACK'd — check pull-ups, rails, XSHUTDN");
        }
    }

    if (r != ESP_OK) {
        ESP_LOGE(TAG, "sc850sl_init failed: %s", esp_err_to_name(r));
        goto idle;
    }

    uint16_t chip_id = 0;
    r = sc850sl_probe(g_cam, &chip_id);
    if (r == ESP_OK) {
        ESP_LOGI(TAG, "probe OK — sensor is alive (id=0x%04x)", chip_id);
        /* Init-table walk with up to 3 attempts. The 191-entry walk has
         * been intermittently flaky on this carrier (sensor's internal
         * I²C state machine occasionally wedges mid-walk). If it fails,
         * drop XSHUTDN to fully reset the sensor, then try again from a
         * clean state. */
        ESP_LOGI(TAG, "loading 4K15 2-lane RAW10 init table (mode %d)…",
                 SC850SL_MODE_4K15_2LANE_RAW10);
        for (int attempt = 0; attempt < 3; ++attempt) {
            r = sc850sl_load_init_table(g_cam, SC850SL_MODE_4K15_2LANE_RAW10);
            if (r == ESP_OK) break;
            ESP_LOGW(TAG, "init table attempt %d failed (%s) — power-cycling sensor",
                     attempt, esp_err_to_name(r));
            gpio_set_level(CONFIG_SC850SL_XSHUTDN_GPIO, 0);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(CONFIG_SC850SL_XSHUTDN_GPIO, 1);
            vTaskDelay(pdMS_TO_TICKS(100));
        }
        if (r == ESP_OK) {
            ESP_LOGI(TAG, "init table OK");

            /* NO crop — emit the sensor's native 3840×2160 frame exactly
             * as table_2 calibrates it. Cropping only rewrites the output
             * window without adjusting HTS/VTS, which can malform the MIPI
             * frame (suspected cause of the stuck isr_count=0). In bypass
             * mode the ISP's 4K-input limit doesn't apply. */
            /* (void) camera_sensor_set_crop(); */

            /* Bring up CSI + ISP BEFORE stream-on so the receiver is
             * armed when the sensor starts pushing pixels. */
            if (camera_pipeline_init() != ESP_OK) {
                ESP_LOGW(TAG, "pipeline init failed — capture will fall back to test image");
            }

            /* Test pattern OFF — the capture path is now confirmed
             * working end-to-end (DMA completes, frames stream over USB),
             * so switch to real lens output. Flip back to true if you
             * need to re-verify the digital path independent of optics. */
            (void) sc850sl_set_test_pattern(g_cam, false);

            /* Set a known-healthy exposure + analog gain explicitly. With
             * test pattern OFF the analog readout path is what feeds MIPI;
             * if exposure/gain are at a degenerate value (0, or so long the
             * frame readout stalls past our capture timeout) the sensor can
             * fail to emit complete frames. Pick a mid exposure (~0x0800
             * lines) and modest gain to guarantee a sane, fast readout.
             *   0x3e00/01/02 = 20-bit exposure (we use the mid 16 bits)
             *   0x3e08/09    = analog gain coarse/fine */
            /* Moderate exposure + minimum gain. Near-max exposure + high
             * gain blew the scene out to white (raw mean ~200, lots of
             * 255s). Pull exposure down ~4x and gain back to 1x; we'll
             * fine-tune from the raw-stats readout. */
            /* Prime a moderate initial exposure + 1x gain using the vendor
             * recipe — NO 0x3e03 "AGC enable" and NO group-hold (those were
             * exactly why manual exposure/gain did nothing). The host-AE loop
             * in usb_stream_send converges from here. */
            g_ae_exp_lines  = 676;        /* ~20 ms at our line period */
            g_ae_gain_x1024 = SC850SL_GAIN_UNITY;
            sc850sl_set_exp_gain(g_ae_exp_lines, g_ae_gain_x1024);
            ESP_LOGI(TAG, "exp/gain primed: %u lines @1.0x (vendor recipe; host-AE active)",
                     (unsigned) g_ae_exp_lines);

            /* Start the CSI receiver FIRST, then turn the sensor stream on.
             * If we did it the other way around (stream_on → start), the
             * sensor would already be ~67 ms into its first frame by the
             * time the CSI bridge armed, so the bridge would try to sync
             * mid-frame and the first DMA would never complete. Arming
             * first means the CSI is waiting for a clean SOF when stream
             * lifts. */
            if (g_pipeline_ready) {
                esp_err_t cs = esp_cam_ctlr_start(g_csi);
                if (cs != ESP_OK) {
                    ESP_LOGW(TAG, "esp_cam_ctlr_start: %s — falling back to test image",
                             esp_err_to_name(cs));
                    g_pipeline_ready = false;
                } else {
                    ESP_LOGI(TAG, "CSI receiver armed (pre-stream)");
                    /* IDF-9706: ISP frame_cfg pokes MUST happen AFTER
                     * esp_cam_ctlr_start (matches esp_video's order). */
                    if (camera_isp_bypass_start() != ESP_OK) {
                        ESP_LOGW(TAG, "ISP bypass setup failed");
                        g_pipeline_ready = false;
                    }
                    /* Seed the queue (defensive, matches IDF test pattern). */
                    esp_cam_ctlr_trans_t seed = {
                        .buffer = g_capture_buf,
                        .buflen = CAPTURE_FB_BYTES,
                    };
                    esp_err_t rr = esp_cam_ctlr_receive(g_csi, &seed, 100);
                    if (rr != ESP_OK) {
                        ESP_LOGW(TAG, "esp_cam_ctlr_receive seed: %s",
                                 esp_err_to_name(rr));
                    } else {
                        ESP_LOGI(TAG, "CSI transaction seeded via receive()");
                    }
                }
            }

            /* Stream-on, with a few app-level retries on top of the
             * per-write retry. The carrier's I²C occasionally NACKs the
             * stream-on writes hard enough that even write_reg's internal
             * retries give up; a short pause + another go almost always
             * succeeds, and it's far cheaper than losing the camera for
             * the whole boot. */
            for (int attempt = 0; attempt < 5; ++attempt) {
                r = sc850sl_stream_on(g_cam);
                if (r == ESP_OK) break;
                ESP_LOGW(TAG, "stream-on attempt %d failed (%s) — retrying",
                         attempt, esp_err_to_name(r));
                vTaskDelay(pdMS_TO_TICKS(20));
            }
            if (r == ESP_OK) {
                ESP_LOGI(TAG, "stream-on — sensor is emitting MIPI data");
                g_cam_ready = true;
                /* Give the sensor a couple of frame periods (~67 ms each)
                 * to stabilize MIPI output before the first capture
                 * attempt. The first frame after stream_on is often
                 * corrupted (sensor still settling AE/internal state). */
                vTaskDelay(pdMS_TO_TICKS(200));
            } else {
                ESP_LOGE(TAG, "stream-on failed after retries: %s", esp_err_to_name(r));
            }
        } else {
            ESP_LOGE(TAG, "init table walk failed: %s", esp_err_to_name(r));
        }
    } else if (r == ESP_ERR_NOT_FOUND) {
        ESP_LOGE(TAG, "wrong chip ID (got 0x%04x, want 0x%04x) — check the FPC",
                 chip_id, SC850SL_CHIP_ID);
    } else {
        ESP_LOGW(TAG, "probe failed: %s — running without sensor",
                 esp_err_to_name(r));
    }

    /* MI1602 probe is disabled in this build (CONFIG_MI1602_ENABLED=n):
     * GPIO 53 / 54 are taken by the SC850SL XVCLK and XSHUTDN. */
    mi1602_try_probe(NULL);

    /* ---- Audio out ---- */
    if (pcm5102a_init() != ESP_OK) {
        ESP_LOGW(TAG, "PCM5102A init failed — periodic TX will skip");
        goto idle;
    }

    /* Auto-exposure runs inside usb_stream_send (ae_step) using the raw
     * frame mean — no separate task, and no ISP AE stats (ISP bypassed). */

    /* ---- Spawn the USB live-stream task (only if pipeline is up) ---- */
    if (g_pipeline_ready) {
        xTaskCreate(usb_stream_task, "usb_stream", 4096, NULL, 4, NULL);
        ESP_LOGI(TAG, "USB live-stream running (%d×%d every %d ms)",
                 USB_STREAM_W, USB_STREAM_H, USB_STREAM_PERIOD_MS);
    }

    /* ---- Periodic SSTV TX task ---- *
     * DISABLED while bringing up the live preview — the TX task holds the
     * capture mutex for ~38 s per cycle, which freezes the USB preview and
     * makes image analysis painful. Re-enable once the image looks right. */
#if 0
    xTaskCreate(sstv_tx_task, "sstv_tx", 6144, NULL, 5, NULL);
    ESP_LOGI(TAG, "periodic SSTV TX task running (interval %d s)",
             SSTV_INTERVAL_SECS);
#else
    (void) sstv_tx_task;   /* silence unused-function warning */
    ESP_LOGI(TAG, "SSTV TX task DISABLED (live-preview bring-up mode)");
#endif

idle:
    /* Ground-test WiFi + web control — started HERE, only after the camera
     * bring-up has fully run (success OR early-failure goto). Bringing the
     * C6/SDIO + WiFi up before app_run() collided with the SC850SL stream-on:
     * the WiFi-connect current/noise transient wedged the sensor's port-0 I²C
     * bus mid-stream-on (while the port-1 MI1602 bus, probed later, survived).
     * Deferring WiFi until the sensor is already streaming decouples the two.
     * No-op stub in the flight build (CONFIG_GROUND_WIFI_ENABLE off). */
    ground_wifi_start();

    /* Heartbeat: 1 Hz blink so we can see at a glance the firmware is alive.
     * After a ~5 s burn-in (firmware reached steady state without crashing),
     * confirm a pending OTA image so the bootloader won't roll it back. */
    {
        int level = 0, ticks = 0;
        bool ota_confirmed = false;
        while (1) {
            heartbeat_blip(CONFIG_SC850SL_HEARTBEAT_LED_GPIO, level);
            level ^= 1;
            if (!ota_confirmed && ++ticks >= 10) {   /* 10 × 500 ms ≈ 5 s */
                ota_confirm_pending();
                ota_confirmed = true;
            }
            vTaskDelay(pdMS_TO_TICKS(500));
        }
    }
}

#endif /* CONFIG_CAMERA_TARGET_SC850SL */
