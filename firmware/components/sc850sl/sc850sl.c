/*
 * SC850SL driver — implementation. ESP-IDF v6.0+.
 *
 * Phase 1 scope (what compiles & runs today):
 *   - Power-on sequencing (rail ENs + XSHUTDN + EXTCLK)
 *   - I²C reg16/data8 read & write
 *   - Chip ID probe
 *   - Init-table walker (table 0..3)
 *   - Window / mirror-flip / test pattern / stream on-off / sleep
 *
 * Phase 2 (later): CSI host wiring, frame buffer hand-off to the ISP.
 *   Those live in `components/isp_pipeline/`, not here — this component
 *   is sensor-only.
 */

#include "sc850sl.h"
#include "sc850sl_regs.h"

#include <string.h>
#include "esp_check.h"
#include "esp_log.h"
#include "esp_rom_sys.h"        // esp_rom_delay_us
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

static const char *TAG = "sc850sl";

struct sc850sl_t {
    sc850sl_config_t        cfg;
    i2c_master_dev_handle_t i2c_dev;
    bool                    powered;
    bool                    extclk_running;
};

/* ------------------------------------------------------------------ *
 *  Helpers
 * ------------------------------------------------------------------ */

static esp_err_t cfg_gpio_output(gpio_num_t pin, int level)
{
    if (pin < 0) return ESP_OK;
    gpio_config_t io = {
        .pin_bit_mask = 1ULL << pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io), TAG, "gpio_config(%d)", pin);
    return gpio_set_level(pin, level);
}

static esp_err_t extclk_start(gpio_num_t pin, uint32_t freq_hz)
{
    /* LEDC 50%-duty square wave to drive the sensor EXTCLK pin.
     * Stamp-P4 has no on-board XO for the camera; the carrier
     * pulls EXTCLK from this GPIO. */
    if (pin < 0) {
        ESP_LOGW(TAG, "EXTCLK pin not set — assuming external clock source");
        return ESP_OK;
    }

    /* For 24 MHz output, LEDC needs duty_resolution=1 (i.e. counter wraps
     * at 2). At higher resolutions the required source clock (24 MHz ×
     * 2^N × divider) exceeds the P4 LEDC's 80 MHz cap. 1-bit gives us
     * the standard 50% square wave the sensor needs.
     * TODO(phase 2): consider esp_clock_output_start() for an exact 24
     * MHz with zero jitter — LEDC uses a fractional divider so eye is
     * good but not glass-clean. */
    ledc_timer_config_t timer = {
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_1_BIT,    /* counter range 0..1 */
        .freq_hz         = freq_hz,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&timer), TAG, "ledc_timer_config");

    ledc_channel_config_t ch = {
        .gpio_num   = pin,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 1,           /* 1 of 2 → 50% */
        .hpoint     = 0,
        .intr_type  = LEDC_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ch), TAG, "ledc_channel_config");

    ESP_LOGI(TAG, "EXTCLK %lu Hz on GPIO%d (LEDC ch0)",
             (unsigned long) freq_hz, pin);
    return ESP_OK;
}

static esp_err_t extclk_stop(gpio_num_t pin)
{
    if (pin < 0) return ESP_OK;
    return ledc_stop(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
}

/* Datasheet section 1.4.1 power-on:
 *   1) DOVDD up, 2) DVDD + AVDD up, 3) XSHUTDN released, 4) wait ≥4 ms,
 *   5) start EXTCLK (or 1) start EXTCLK earlier — both are valid),
 *   6) first I²C transaction.
 */
static esp_err_t power_on(sc850sl_handle_t h)
{
    /* Hold sensor in reset while we sequence the rails. */
    ESP_RETURN_ON_ERROR(cfg_gpio_output(h->cfg.xshutdn_gpio, 0), TAG, "XSHUTDN low");

    /* Rail enables — if not used (always-on carrier), the cfg_gpio_output
     * call no-ops for pin == -1. */
    ESP_RETURN_ON_ERROR(cfg_gpio_output(h->cfg.rail_en_dovdd, 1), TAG, "DOVDD en");
    esp_rom_delay_us(500);
    ESP_RETURN_ON_ERROR(cfg_gpio_output(h->cfg.rail_en_dvdd,  1), TAG, "DVDD en");
    ESP_RETURN_ON_ERROR(cfg_gpio_output(h->cfg.rail_en_avdd,  1), TAG, "AVDD en");
    esp_rom_delay_us(1000);

    /* Start EXTCLK so the sensor PLL has a reference before XSHUTDN release. */
    ESP_RETURN_ON_ERROR(extclk_start(h->cfg.extclk_gpio, h->cfg.extclk_hz),
                        TAG, "EXTCLK start");
    h->extclk_running = true;
    esp_rom_delay_us(1000);

    /* Release reset. Datasheet T4 ≥ 4 ms before first I²C transaction.
     * In practice on this carrier (Stamp-P4 + custom FPC) the sensor's
     * internal sub-blocks need significantly longer than 4 ms to
     * stabilize, otherwise register writes during the mode-2 init walk
     * sporadically NACK. 100 ms is well past datasheet but cheap and
     * eliminates the flakiness. */
    if (h->cfg.xshutdn_gpio >= 0) {
        gpio_set_level(h->cfg.xshutdn_gpio, 1);
    }
    vTaskDelay(pdMS_TO_TICKS(100));
    h->powered = true;
    return ESP_OK;
}

static void power_off(sc850sl_handle_t h)
{
    if (h->cfg.xshutdn_gpio >= 0) gpio_set_level(h->cfg.xshutdn_gpio, 0);
    if (h->extclk_running) { extclk_stop(h->cfg.extclk_gpio); h->extclk_running = false; }
    cfg_gpio_output(h->cfg.rail_en_avdd,  0);
    cfg_gpio_output(h->cfg.rail_en_dvdd,  0);
    cfg_gpio_output(h->cfg.rail_en_dovdd, 0);
    h->powered = false;
}

/* ------------------------------------------------------------------ *
 *  I²C register access
 * ------------------------------------------------------------------ */

/* This carrier's I²C is intermittently flaky (occasional NACK/timeout on
 * an otherwise-healthy bus). The init-table walker has its own retry, but
 * one-off writes like stream-on / exposure used to fail on a single
 * transient NACK. Retry every write a few times (reset the bus between
 * tries to clear any stuck state) so the whole driver is robust. */
esp_err_t sc850sl_write_reg(sc850sl_handle_t h, uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff), val };
    esp_err_t r = ESP_FAIL;
    for (int attempt = 0; attempt < 4; ++attempt) {
        r = i2c_master_transmit(h->i2c_dev, buf, sizeof(buf), 50 /*ms*/);
        if (r == ESP_OK) return ESP_OK;
        if (h->cfg.i2c_bus) i2c_master_bus_reset(h->cfg.i2c_bus);
        esp_rom_delay_us(500);
    }
    return r;
}

/* Fast single-shot write for per-frame hot paths (host AE): ONE attempt,
 * short timeout, and crucially NO i2c_master_bus_reset on failure. The full
 * sc850sl_write_reg() resets the bus on every failed attempt, and when the
 * bus is genuinely stuck mid-stream that reset itself hits a hardware-reset
 * timeout and blocks the caller for seconds — which tripped the task watchdog
 * and put the board in a reboot loop. The AE loop would rather skip an update
 * than hang, so it uses this and bails on the first failure. */
esp_err_t sc850sl_write_reg_fast(sc850sl_handle_t h, uint16_t reg, uint8_t val)
{
    uint8_t buf[3] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff), val };
    return i2c_master_transmit(h->i2c_dev, buf, sizeof(buf), 20 /*ms*/);
}

esp_err_t sc850sl_read_reg(sc850sl_handle_t h, uint16_t reg, uint8_t *val)
{
    uint8_t addr[2] = { (uint8_t)(reg >> 8), (uint8_t)(reg & 0xff) };
    esp_err_t r = ESP_FAIL;
    for (int attempt = 0; attempt < 4; ++attempt) {
        r = i2c_master_transmit_receive(h->i2c_dev, addr, 2, val, 1, 50);
        if (r == ESP_OK) return ESP_OK;
        if (h->cfg.i2c_bus) i2c_master_bus_reset(h->cfg.i2c_bus);
        esp_rom_delay_us(500);
    }
    return r;
}

static esp_err_t write_table(sc850sl_handle_t h,
                             const sc850sl_reg_pair_t *tbl, size_t len)
{
    for (size_t i = 0; i < len; ++i) {
        esp_err_t r = sc850sl_write_reg(h, tbl[i].reg, tbl[i].val);
        if (r != ESP_OK) {
            /* NACK: aggressive recovery. We saw that without this the
             * init table walks intermittently fails at different points
             * (table[14], [32], [33] — varies run-to-run). Reset the
             * I²C bus (9-clock + STOP) to clear any stuck slave state,
             * wait, then retry. */
            int retry;
            for (retry = 0; retry < 5 && r != ESP_OK; ++retry) {
                i2c_master_bus_reset(h->cfg.i2c_bus);
                vTaskDelay(pdMS_TO_TICKS(20));
                r = sc850sl_write_reg(h, tbl[i].reg, tbl[i].val);
            }
            if (r != ESP_OK) {
                ESP_LOGE(TAG, "table[%u] 0x%04x=0x%02x failed after %d retries: %s",
                         (unsigned)i, tbl[i].reg, tbl[i].val, retry, esp_err_to_name(r));
                return r;
            }
            ESP_LOGW(TAG, "table[%u] 0x%04x=0x%02x retried OK after %d attempts",
                     (unsigned)i, tbl[i].reg, tbl[i].val, retry);
        }
        /* Settling delay after PLL-control writes (registers 0x36e9 and
         * 0x36f9). When these are written with non-reset values, the
         * sensor's internal clock domain re-locks and the I²C interface
         * is briefly unresponsive — the very next register write would
         * otherwise hit the bus mid-relock and NACK. 20 ms is generous;
         * datasheet T_pll_lock ~2 ms. */
        if (tbl[i].reg == 0x36e9 || tbl[i].reg == 0x36f9) {
            vTaskDelay(pdMS_TO_TICKS(20));
        }
        /* Soft reset needs ≥150 ns; the I²C frame itself is far longer,
         * so no explicit delay needed for other writes. */
    }
    return ESP_OK;
}

/* ------------------------------------------------------------------ *
 *  Public API
 * ------------------------------------------------------------------ */

esp_err_t sc850sl_init(const sc850sl_config_t *cfg, sc850sl_handle_t *out)
{
    ESP_RETURN_ON_FALSE(cfg && out && cfg->i2c_bus, ESP_ERR_INVALID_ARG,
                        TAG, "null arg");

    sc850sl_handle_t h = calloc(1, sizeof(*h));
    if (!h) return ESP_ERR_NO_MEM;
    h->cfg = *cfg;
    if (h->cfg.i2c_addr     == 0) h->cfg.i2c_addr     = SC850SL_I2C_ADDR_DEFAULT;
    if (h->cfg.i2c_freq_hz  == 0) h->cfg.i2c_freq_hz  = 100000;
    if (h->cfg.extclk_hz    == 0) h->cfg.extclk_hz    = 24000000;

    /* Attach a device handle to the bus.
     *
     * scl_wait_us tells the HW how long to tolerate the slave holding SCL
     * low (clock-stretching). The default reg value is short (~10 us) and
     * Smartsens sensors can stretch noticeably during the first few writes
     * after stream-on / reset while their internal state machines settle.
     * 50 ms is far more than the sensor needs but well below our overall
     * xfer_timeout_ms, so it just prevents a spurious "timeout" diagnostic
     * from the HW when the sensor is being slow. */
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = h->cfg.i2c_addr,
        .scl_speed_hz    = h->cfg.i2c_freq_hz,
        .scl_wait_us     = 50 * 1000,
    };
    esp_err_t r = i2c_master_bus_add_device(h->cfg.i2c_bus, &dev_cfg, &h->i2c_dev);
    if (r != ESP_OK) { free(h); return r; }

    r = power_on(h);
    if (r != ESP_OK) { i2c_master_bus_rm_device(h->i2c_dev); free(h); return r; }

    *out = h;
    ESP_LOGI(TAG, "powered, addr=0x%02x, freq=%lu Hz, mode=%d",
             h->cfg.i2c_addr, (unsigned long) h->cfg.i2c_freq_hz, (int) h->cfg.mode);
    return ESP_OK;
}

esp_err_t sc850sl_probe(sc850sl_handle_t h, uint16_t *chip_id)
{
    uint8_t hi = 0, lo = 0;
    ESP_RETURN_ON_ERROR(sc850sl_read_reg(h, SC850SL_REG_CHIP_ID_HI, &hi), TAG, "id hi");
    ESP_RETURN_ON_ERROR(sc850sl_read_reg(h, SC850SL_REG_CHIP_ID_LO, &lo), TAG, "id lo");
    uint16_t id = ((uint16_t) hi << 8) | lo;
    if (chip_id) *chip_id = id;
    ESP_LOGI(TAG, "chip ID = 0x%04x (%s)", id,
             id == SC850SL_CHIP_ID ? "expected ✓" : "UNEXPECTED");
    return id == SC850SL_CHIP_ID ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t sc850sl_load_init_table(sc850sl_handle_t h, sc850sl_mode_t mode)
{
    const sc850sl_reg_pair_t *tbl;
    size_t len;
    switch (mode) {
        case SC850SL_MODE_4K30_4LANE_RAW10:
            tbl = sc850sl_table_0; len = sc850sl_table_0_len; break;
        case SC850SL_MODE_4K30_4LANE_HDR_DOL:
            tbl = sc850sl_table_1; len = sc850sl_table_1_len; break;
        case SC850SL_MODE_4K15_2LANE_RAW10:
            tbl = sc850sl_table_2; len = sc850sl_table_2_len; break;
        case SC850SL_MODE_4K15_2LANE_RAW10_ALT:
            tbl = sc850sl_table_3; len = sc850sl_table_3_len; break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    ESP_LOGI(TAG, "loading mode %d table (%u entries)", (int) mode, (unsigned) len);
    h->cfg.mode = mode;
    return write_table(h, tbl, len);
}

esp_err_t sc850sl_set_window(sc850sl_handle_t h,
                             uint16_t w, uint16_t hgt, uint16_t x, uint16_t y)
{
    ESP_RETURN_ON_ERROR(sc850sl_write_reg(h, SC850SL_REG_OUT_W_HI, w >> 8),   TAG, "W hi");
    ESP_RETURN_ON_ERROR(sc850sl_write_reg(h, SC850SL_REG_OUT_W_LO, w & 0xff), TAG, "W lo");
    ESP_RETURN_ON_ERROR(sc850sl_write_reg(h, SC850SL_REG_OUT_H_HI, hgt >> 8), TAG, "H hi");
    ESP_RETURN_ON_ERROR(sc850sl_write_reg(h, SC850SL_REG_OUT_H_LO, hgt & 0xff),TAG, "H lo");
    ESP_RETURN_ON_ERROR(sc850sl_write_reg(h, SC850SL_REG_OUT_X_HI, x >> 8),   TAG, "X hi");
    ESP_RETURN_ON_ERROR(sc850sl_write_reg(h, SC850SL_REG_OUT_X_LO, x & 0xff), TAG, "X lo");
    ESP_RETURN_ON_ERROR(sc850sl_write_reg(h, SC850SL_REG_OUT_Y_HI, y >> 8),   TAG, "Y hi");
    ESP_RETURN_ON_ERROR(sc850sl_write_reg(h, SC850SL_REG_OUT_Y_LO, y & 0xff), TAG, "Y lo");
    return ESP_OK;
}

esp_err_t sc850sl_set_test_pattern(sc850sl_handle_t h, bool enable)
{
    /* Read-modify-write the test_pattern register (bit 3). */
    uint8_t v = 0;
    ESP_RETURN_ON_ERROR(sc850sl_read_reg(h, SC850SL_REG_TEST_PATTERN, &v), TAG, "read 0x4501");
    v = enable ? (v | (1u << 3)) : (v & ~(1u << 3));
    return sc850sl_write_reg(h, SC850SL_REG_TEST_PATTERN, v);
}

esp_err_t sc850sl_set_mirror_flip(sc850sl_handle_t h, bool mirror, bool flip)
{
    uint8_t v = 0;
    v |= mirror ? (0x3 << 1) : 0;     /* bit[2:1] = 11 = mirror on */
    v |= flip   ? (0x3 << 5) : 0;     /* bit[6:5] = 11 = flip on   */
    return sc850sl_write_reg(h, SC850SL_REG_MIRROR_FLIP, v);
}

esp_err_t sc850sl_stream_on(sc850sl_handle_t h)
{
    /* Order matters: exit sleep first, then enable streaming. */
    ESP_RETURN_ON_ERROR(sc850sl_write_reg(h, SC850SL_REG_SLEEP_AUX, 0x00), TAG, "sleep off");
    return sc850sl_write_reg(h, SC850SL_REG_STREAM, 0x01);
}

esp_err_t sc850sl_stream_off(sc850sl_handle_t h)
{
    return sc850sl_write_reg(h, SC850SL_REG_STREAM, 0x00);
}

esp_err_t sc850sl_sleep(sc850sl_handle_t h)
{
    /* Datasheet 1.4.3 — order is enforced. */
    ESP_RETURN_ON_ERROR(sc850sl_write_reg(h, SC850SL_REG_STREAM,    0x00), TAG, "stream off");
    return sc850sl_write_reg(h, SC850SL_REG_SLEEP_AUX, 0x0f);
}

esp_err_t sc850sl_wake(sc850sl_handle_t h)
{
    ESP_RETURN_ON_ERROR(sc850sl_write_reg(h, SC850SL_REG_SLEEP_AUX, 0x00), TAG, "sleep aux");
    return sc850sl_write_reg(h, SC850SL_REG_STREAM, 0x01);
}

void sc850sl_deinit(sc850sl_handle_t h)
{
    if (!h) return;
    if (h->powered) sc850sl_stream_off(h);
    power_off(h);
    if (h->i2c_dev) i2c_master_bus_rm_device(h->i2c_dev);
    free(h);
}
