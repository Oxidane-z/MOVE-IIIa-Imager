/*
 * Shared MI1602 probe helper. Both app_sc850sl.c (flight) and
 * app_sc202cs_tab5.c (bench) call mi1602_try_probe() to verify the
 * thermal camera's I²C link is alive.
 *
 * Phase 1 scope: probe-only (open I²C bus → mi1602_init → mi1602_probe →
 * leak the handle since we never need to deinit before reboot). Frame
 * capture comes in Phase 4.
 *
 * Failure modes are non-fatal: a missing or unresponsive IR module just
 * logs a warning and lets the visible-camera path continue.
 */
#include "app.h"
#include "sdkconfig.h"

#if CONFIG_MI1602_ENABLED

#include <math.h>                 /* fabsf */
#include "esp_log.h"
#include "esp_rom_sys.h"          /* esp_rom_delay_us */
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "mi1602.h"

static const char *TAG = "mi1602_probe";

/* Live handle, kept after a successful probe so the USB-stream path can
 * pull thermal frames. NULL until mi1602_try_probe() succeeds. */
static mi1602_handle_t g_mi1602 = NULL;

/* Board glue: the carrier routes the MI48 MODE strap to a GPIO. MODE
 * must be LOW (GND) to select the I²C-control + SPI-data host interface.
 *
 * IMPORTANT CAVEAT: the MI48 samples MODE at its OWN power-on reset
 * (t≈0, when the rail comes up). Our firmware can't drive GPIO10 until
 * app_main runs (~1.2 s later) — far too late. If MODE is left to the
 * ESP GPIO (high-Z during ESP boot), the module latches an undefined /
 * wrong mode at power-on and then drives the SDA/SCL pins, which shows
 * up as the I²C bus being HELD LOW (every address times out, as we see).
 * Driving GPIO10 low afterwards does NOT re-latch the mode — the MI48
 * needs a reset for that, and we have no RESET line wired (rst=-1).
 *
 * => MODE needs a HARDWARE pull-down to GND so it's already low when the
 *    module powers up. The firmware drive below is then just redundant
 *    insurance. Alternatively, wire the MI48 RESET_N to a GPIO and set
 *    CONFIG_MI1602_RESET_GPIO so we can: drive MODE low → pulse reset →
 *    module re-latches the correct mode. */
#define MI1602_MODE_GPIO    10
#define MI1602_MODE_LEVEL   0      /* LOW = I²C + SPI mode */

void mi1602_try_probe(i2c_master_bus_handle_t shared_bus)
{
    i2c_master_bus_handle_t bus = shared_bus;
    bool created_bus = false;

#if MI1602_MODE_GPIO >= 0
    gpio_config_t mode_cfg = {
        .pin_bit_mask = 1ULL << MI1602_MODE_GPIO,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&mode_cfg);
    gpio_set_level(MI1602_MODE_GPIO, MI1602_MODE_LEVEL);
    esp_rom_delay_us(2000);   /* let the strap settle before talking I²C */
    ESP_LOGI(TAG, "MI1602 MODE strap: GPIO%d driven %d",
             MI1602_MODE_GPIO, MI1602_MODE_LEVEL);
#endif

    if (!bus) {
        ESP_LOGI(TAG, "MI1602 aux probe (own bus): port=%d SDA=%d SCL=%d",
                 CONFIG_MI1602_I2C_PORT,
                 CONFIG_MI1602_I2C_SDA_GPIO,
                 CONFIG_MI1602_I2C_SCL_GPIO);
        i2c_master_bus_config_t bus_cfg = {
            .i2c_port          = CONFIG_MI1602_I2C_PORT,
            .sda_io_num        = CONFIG_MI1602_I2C_SDA_GPIO,
            .scl_io_num        = CONFIG_MI1602_I2C_SCL_GPIO,
            .clk_source        = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        esp_err_t r = i2c_new_master_bus(&bus_cfg, &bus);
        if (r != ESP_OK) {
            ESP_LOGW(TAG, "i2c_new_master_bus failed (%s) — IR aux unavailable",
                     esp_err_to_name(r));
            return;
        }
        created_bus = true;

        /* Diagnostic scan of the MI1602 bus — tells us whether the module
         * ACKs at all (and at which address: 0x40 vs 0x41 ADDR strap). */
        i2c_master_bus_reset(bus);
        ESP_LOGI(TAG, "MI1602 I²C scan (port %d, expect 0x40 or 0x41):",
                 CONFIG_MI1602_I2C_PORT);
        int found = 0;
        for (uint8_t a = 0x08; a < 0x78; ++a) {
            if (i2c_master_probe(bus, a, 50) == ESP_OK) {
                ESP_LOGI(TAG, "  device found at 0x%02x", a);
                ++found;
            }
        }
        if (!found) {
            ESP_LOGW(TAG, "  no devices ACK'd on MI1602 bus — check pull-ups,"
                     " power, MODE strap timing, SDA/SCL swap");
        }
    } else {
        ESP_LOGI(TAG, "MI1602 aux probe (shared bus from BSP)");
    }

    mi1602_config_t cfg = {
        .i2c_bus         = bus,
#ifdef CONFIG_MI1602_I2C_ADDR_HIGH
        .i2c_addr        = MI1602_I2C_ADDR_ALT,    /* 0x41 */
#else
        .i2c_addr        = MI1602_I2C_ADDR_DEFAULT, /* 0x40 */
#endif
        .i2c_freq_hz     = CONFIG_MI1602_I2C_FREQ_HZ,

        .spi_host        = CONFIG_MI1602_SPI_HOST,
        .spi_bus_already_initialised = false,
        .sclk_gpio       = CONFIG_MI1602_SPI_SCLK_GPIO,
        .miso_gpio       = CONFIG_MI1602_SPI_MISO_GPIO,
        .mosi_gpio       = CONFIG_MI1602_SPI_MOSI_GPIO,
        .cs_n_gpio       = CONFIG_MI1602_SPI_CS_GPIO,
        .spi_freq_hz     = CONFIG_MI1602_SPI_FREQ_HZ,

        .reset_n_gpio    = CONFIG_MI1602_RESET_GPIO,
        .data_ready_gpio = CONFIG_MI1602_DATA_READY_GPIO,
        .sysclk_gpio     = CONFIG_MI1602_SYSCLK_GPIO,
        .sysclk_hz       = CONFIG_MI1602_SYSCLK_HZ,
    };

    mi1602_handle_t cam = NULL;
    esp_err_t r = mi1602_init(&cfg, &cam);
    if (r != ESP_OK) {
        ESP_LOGW(TAG, "mi1602_init failed (%s) — module not present or wired wrong",
                 esp_err_to_name(r));
        if (created_bus) i2c_del_master_bus(bus);
        return;
    }

    r = mi1602_probe(cam);
    if (r == ESP_OK) {
        ESP_LOGI(TAG, "✓ MI1602 probe OK — I²C link live at 0x%02x",
                 cfg.i2c_addr);
        /* Bring the module out of boot so capture works, then keep the
         * handle for the USB-stream path. */
        if (mi1602_bootup(cam) != ESP_OK) {
            ESP_LOGW(TAG, "mi1602_bootup failed — capture may not work");
        }
        g_mi1602 = cam;
    } else {
        ESP_LOGW(TAG, "MI1602 probe NACK at 0x%02x (%s) — IR sensor offline",
                 cfg.i2c_addr, esp_err_to_name(r));
    }
}

bool mi1602_aux_available(void)
{
    return g_mi1602 != NULL;
}

/* Matplotlib "twilight" colormap — cyclic: light lavender at both ends,
 * darkest in the middle (cold → light-blue → dark indigo → maroon → hot
 * light-pink). 17 anchor points (Δ = 1/16), linearly interpolated. */
static const uint8_t TWILIGHT[17][3] = {
    {225, 218, 226},  /* 0.000 */
    {183, 199, 224},  /* 0.062 */
    {138, 173, 215},  /* 0.125 */
    {105, 142, 197},  /* 0.188 */
    { 90, 114, 177},  /* 0.250 */
    { 80,  92, 152},  /* 0.312 */
    { 72,  73, 122},  /* 0.375 */
    { 62,  53,  89},  /* 0.438 */
    { 48,  30,  58},  /* 0.500 (darkest) */
    { 80,  31,  57},  /* 0.562 */
    {116,  41,  62},  /* 0.625 */
    {152,  56,  67},  /* 0.688 */
    {188,  80,  79},  /* 0.750 */
    {210, 119, 101},  /* 0.812 */
    {220, 159, 143},  /* 0.875 */
    {224, 193, 186},  /* 0.938 */
    {225, 218, 226},  /* 1.000 */
};

static inline uint16_t twilight_rgb565(float t)
{
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;
    float f = t * 16.0f;               /* 16 segments between 17 anchors */
    int i = (int)f;
    if (i > 15) i = 15;
    float frac = f - (float)i;
    uint8_t R = (uint8_t)(TWILIGHT[i][0] + frac * (TWILIGHT[i+1][0] - TWILIGHT[i][0]));
    uint8_t G = (uint8_t)(TWILIGHT[i][1] + frac * (TWILIGHT[i+1][1] - TWILIGHT[i][1]));
    uint8_t B = (uint8_t)(TWILIGHT[i][2] + frac * (TWILIGHT[i+1][2] - TWILIGHT[i][2]));
    return (uint16_t)(((R & 0xF8) << 8) | ((G & 0xFC) << 3) | (B >> 3));
}

esp_err_t mi1602_aux_capture_rgb565(uint16_t *dst, int dw, int dh, int dst_stride)
{
    if (!g_mi1602) return ESP_ERR_INVALID_STATE;

    /* One 160×120 16-bit dK frame (no header → saves 320 bytes/frame). */
    static uint16_t px[MI1602_FPA_PIXELS];
    esp_err_t r = mi1602_capture_single(g_mi1602, px, NULL);
    if (r != ESP_OK) return r;

    /* Auto-range over the frame for the colormap. */
    uint16_t mn = 0xffff, mx = 0;
    for (int i = 0; i < MI1602_FPA_PIXELS; ++i) {
        if (px[i] < mn) mn = px[i];
        if (px[i] > mx) mx = px[i];
    }
    float span = (mx > mn) ? (float)(mx - mn) : 1.0f;

    /* Nearest-neighbor scale 160×120 → dw×dh with the twilight colormap.
     * Mirror horizontally (sample column cols-1-sx) — the module is
     * physically oriented so its native readout comes out flipped vs the
     * visible camera. */
    for (int y = 0; y < dh; ++y) {
        int sy = (y * MI1602_FPA_ROWS) / dh;
        const uint16_t *srow = px + sy * MI1602_FPA_COLS;
        uint16_t *drow = dst + (size_t)y * dst_stride;
        for (int x = 0; x < dw; ++x) {
            int sx = (x * MI1602_FPA_COLS) / dw;
            sx = (MI1602_FPA_COLS - 1) - sx;       /* horizontal mirror */
            float t = (float)(srow[sx] - mn) / span;
            drow[x] = twilight_rgb565(t);
        }
    }
    return ESP_OK;
}

#else  /* !CONFIG_MI1602_ENABLED */

void mi1602_try_probe(i2c_master_bus_handle_t shared_bus)
{
    (void)shared_bus;
}
bool mi1602_aux_available(void) { return false; }
esp_err_t mi1602_aux_capture_rgb565(uint16_t *dst, int dw, int dh, int dst_stride)
{
    (void)dst; (void)dw; (void)dh; (void)dst_stride;
    return ESP_ERR_NOT_SUPPORTED;
}

#endif
