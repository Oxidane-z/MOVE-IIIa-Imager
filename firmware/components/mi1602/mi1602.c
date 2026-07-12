/*
 * MI1602 thermal-camera driver — core file.
 *
 * Implements: init/deinit, probe, bootup, low-level register R/W, typed
 * configuration helpers (FPS, emissivity, filters, ...), single-shot frame
 * capture, and the streaming task.
 *
 * SPI frame transfer is implemented in mi1602_spi.c.
 * CRC-16/CCITT-FALSE is in mi1602_crc.c.
 * Pure-math post-processing is in mi1602_post.c.
 *
 * Behaviour is modelled on pysenxor 1.6.7. Cross-references in comments
 * use the format `pysenxor mi48.py:NNN-MMM`.
 */
#include <inttypes.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"

#include "mi1602.h"
#include "mi1602_internal.h"
#include "mi1602_regs.h"

static const char *TAG = "mi1602";

#define MI1602_REG_TIMEOUT_MS    50
/* pysenxor (senxor/mi48.py:406-410) polls BOOTING_UP with NO give-up timeout;
 * our old 500 ms bailed while the SenXor was still coming up. Give it real
 * time. DATA_READY likewise: the first single-frame after trigger can take
 * more than one frame period at low FRAME_RATE_DIVIDER settings. */
#define MI1602_BOOT_TIMEOUT_MS   3000
#define MI1602_BOOT_POLL_MS      10
#define MI1602_DATA_READY_TIMEOUT_MS 2000
#define MI1602_DATA_READY_POLL_MS    5

/* LEDC slot used by the optional sysclk output. TIMER_0/CH_0 are reserved
 * for the SC850SL EXTCLK in the parent firmware, so we default to slot 1. */
#define MI1602_SYSCLK_LEDC_TIMER     LEDC_TIMER_1
#define MI1602_SYSCLK_LEDC_CHANNEL   LEDC_CHANNEL_1
#define MI1602_SYSCLK_LEDC_MODE      LEDC_LOW_SPEED_MODE

/* ----------------------------------------------------------------------- */
/*  Helpers                                                                */
/* ----------------------------------------------------------------------- */

static inline int clampi(int v, int lo, int hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static esp_err_t mi1602_gpio_output(gpio_num_t pin, int initial_level)
{
    if (pin < 0) return ESP_OK;
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << (uint64_t)pin,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_RETURN_ON_ERROR(gpio_config(&cfg), TAG, "gpio_config(%d)", pin);
    gpio_set_level(pin, initial_level);
    return ESP_OK;
}

static esp_err_t mi1602_gpio_input(gpio_num_t pin)
{
    if (pin < 0) return ESP_OK;
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << (uint64_t)pin,
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg);
}

static esp_err_t mi1602_sysclk_start(struct mi1602_t *m)
{
    if (m->cfg.sysclk_gpio < 0) return ESP_OK;

    ledc_timer_config_t tcfg = {
        .speed_mode      = MI1602_SYSCLK_LEDC_MODE,
        .timer_num       = MI1602_SYSCLK_LEDC_TIMER,
        .duty_resolution = LEDC_TIMER_2_BIT,
        .freq_hz         = m->cfg.sysclk_hz ? m->cfg.sysclk_hz : 3000000,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_RETURN_ON_ERROR(ledc_timer_config(&tcfg), TAG, "sysclk ledc_timer_config");

    ledc_channel_config_t ccfg = {
        .gpio_num   = m->cfg.sysclk_gpio,
        .speed_mode = MI1602_SYSCLK_LEDC_MODE,
        .channel    = MI1602_SYSCLK_LEDC_CHANNEL,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = MI1602_SYSCLK_LEDC_TIMER,
        .duty       = 2,         /* 50 % at 2-bit resolution */
        .hpoint     = 0,
    };
    ESP_RETURN_ON_ERROR(ledc_channel_config(&ccfg), TAG, "sysclk ledc_channel_config");
    m->sysclk_ledc_ch = (int)MI1602_SYSCLK_LEDC_CHANNEL;
    m->sysclk_active  = true;
    ESP_LOGI(TAG, "sysclk %u Hz on GPIO%d (LEDC ch%d)",
             (unsigned)tcfg.freq_hz, m->cfg.sysclk_gpio, (int)MI1602_SYSCLK_LEDC_CHANNEL);
    return ESP_OK;
}

static void mi1602_sysclk_stop(struct mi1602_t *m)
{
    if (!m->sysclk_active) return;
    ledc_stop(MI1602_SYSCLK_LEDC_MODE, (ledc_channel_t)m->sysclk_ledc_ch, 0);
    m->sysclk_active = false;
}

/* ----------------------------------------------------------------------- */
/*  I²C register access                                                    */
/*  pysenxor interfaces.py:28-58                                           */
/* ----------------------------------------------------------------------- */

esp_err_t mi1602_reg_read(mi1602_handle_t h, uint8_t reg, uint8_t *val)
{
    if (h == NULL || val == NULL) return ESP_ERR_INVALID_ARG;
    return i2c_master_transmit_receive(h->i2c_dev, &reg, 1, val, 1,
                                       MI1602_REG_TIMEOUT_MS);
}

esp_err_t mi1602_reg_write(mi1602_handle_t h, uint8_t reg, uint8_t val)
{
    if (h == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(h->i2c_dev, buf, sizeof(buf),
                               MI1602_REG_TIMEOUT_MS);
}

/* ----------------------------------------------------------------------- */
/*  Probe & info                                                           */
/* ----------------------------------------------------------------------- */

esp_err_t mi1602_probe(mi1602_handle_t h)
{
    uint8_t fwmaj_min = 0;
    esp_err_t err = mi1602_reg_read(h, MI48_REG_FW_VERSION_1, &fwmaj_min);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "probe: I²C read FW_VERSION_1 failed: %s", esp_err_to_name(err));
        return err;
    }
    if (fwmaj_min == 0x00 || fwmaj_min == 0xFF) {
        ESP_LOGE(TAG, "probe: implausible FW_VERSION_1=0x%02x", fwmaj_min);
        return ESP_ERR_NOT_FOUND;
    }
    return ESP_OK;
}

esp_err_t mi1602_get_camera_info(mi1602_handle_t h,
                                 uint8_t  *senxor_type,
                                 uint8_t  *module_type,
                                 uint16_t *fw_version,
                                 uint8_t  *fw_build,
                                 uint8_t   serial[6])
{
    if (h == NULL) return ESP_ERR_INVALID_ARG;
    uint8_t fw1 = 0, fw2 = 0, sx = 0, mod = 0;
    ESP_RETURN_ON_ERROR(mi1602_reg_read(h, MI48_REG_FW_VERSION_1, &fw1), TAG, "FW1");
    ESP_RETURN_ON_ERROR(mi1602_reg_read(h, MI48_REG_FW_VERSION_2, &fw2), TAG, "FW2");
    ESP_RETURN_ON_ERROR(mi1602_reg_read(h, MI48_REG_SENXOR_TYPE,  &sx),  TAG, "SENXOR_TYPE");
    ESP_RETURN_ON_ERROR(mi1602_reg_read(h, MI48_REG_MODULE_TYPE,  &mod), TAG, "MODULE_TYPE");

    if (senxor_type) *senxor_type = sx;
    if (module_type) *module_type = mod;
    if (fw_version)  *fw_version  = ((uint16_t)(fw1 & 0xF0) << 4) | (uint16_t)(fw1 & 0x0F);
    if (fw_build)    *fw_build    = fw2;

    if (serial) {
        for (uint8_t i = 0; i < 6; ++i) {
            uint8_t b = 0;
            ESP_RETURN_ON_ERROR(mi1602_reg_read(h, MI48_REG_SENXOR_ID_0 + i, &b),
                                TAG, "SENXOR_ID_%u", i);
            serial[i] = b;
        }
    }
    return ESP_OK;
}

/* ----------------------------------------------------------------------- */
/*  Reset / bootup                                                         */
/*  pysenxor mi48.py:378-417                                               */
/* ----------------------------------------------------------------------- */

static void mi1602_pulse_reset(struct mi1602_t *m)
{
    if (m->cfg.reset_n_gpio < 0) return;
    gpio_set_level(m->cfg.reset_n_gpio, 0);
    esp_rom_delay_us(100);   /* assert ≥50 µs */
    gpio_set_level(m->cfg.reset_n_gpio, 1);
    vTaskDelay(pdMS_TO_TICKS(50));   /* deassert settle */
}

esp_err_t mi1602_get_status(mi1602_handle_t h, uint8_t *status)
{
    return mi1602_reg_read(h, MI48_REG_STATUS, status);
}

esp_err_t mi1602_get_mode(mi1602_handle_t h, uint8_t *mode)
{
    return mi1602_reg_read(h, MI48_REG_FRAME_MODE, mode);
}

esp_err_t mi1602_bootup(mi1602_handle_t h)
{
    if (h == NULL) return ESP_ERR_INVALID_ARG;

    mi1602_pulse_reset(h);

    /* clear any leftover streaming state */
    esp_err_t err = mi1602_reg_write(h, MI48_REG_FRAME_MODE, 0x00);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "bootup: initial FRAME_MODE clear failed: %s",
                 esp_err_to_name(err));
        /* don't return — chip may still be booting */
    }

    int64_t t0 = esp_timer_get_time();
    uint8_t status = 0;
    bool booted = false;
    while ((esp_timer_get_time() - t0) < (int64_t)MI1602_BOOT_TIMEOUT_MS * 1000) {
        err = mi1602_reg_read(h, MI48_REG_STATUS, &status);
        if (err == ESP_OK && !(status & MI48_STATUS_BOOTING_UP)) {
            booted = true;
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(MI1602_BOOT_POLL_MS));
    }
    int64_t boot_ms = (esp_timer_get_time() - t0) / 1000;
    if (!booted) {
        ESP_LOGE(TAG, "bootup: timed out after %lld ms (status=0x%02x)",
                 (long long)boot_ms, status);
        return ESP_ERR_TIMEOUT;
    }
    ESP_LOGI(TAG, "bootup OK in %lld ms (status=0x%02x)", (long long)boot_ms, status);

    /* If we see a residual DATA_READY, drain one frame so the SPI slave's
     * output buffer is empty when we next ask for one. We do this only when
     * SPI is up; the typical flow is bootup → capture → which already
     * tolerates a sticky DATA_READY by re-asserting FRAME_MODE. */
    if ((status & MI48_STATUS_CAPTURE_ERROR) || (status & MI48_STATUS_SXIF_ERROR)) {
        ESP_LOGW(TAG, "bootup: post-boot error flags 0x%02x", status);
    }
    return ESP_OK;
}

/* ----------------------------------------------------------------------- */
/*  Typed configuration                                                    */
/* ----------------------------------------------------------------------- */

esp_err_t mi1602_set_fps(mi1602_handle_t h, uint8_t fps_divisor)
{
    if (fps_divisor == 0) return ESP_ERR_INVALID_ARG;
    return mi1602_reg_write(h, MI48_REG_FRAME_RATE, fps_divisor);
}

esp_err_t mi1602_set_emissivity(mi1602_handle_t h, float emissivity)
{
    if (emissivity < 0.0f || emissivity > 1.0f) return ESP_ERR_INVALID_ARG;
    uint8_t v = (uint8_t)clampi((int)lroundf(emissivity * 100.0f), 0, 100);
    return mi1602_reg_write(h, MI48_REG_EMISSIVITY, v);
}

esp_err_t mi1602_set_offset_corr(mi1602_handle_t h, float kelvin)
{
    /* FW <4.2.3 uses 0.05 K/LSB, ≥4.2.3 uses 0.10 K/LSB. We can't probe
     * the FW version cheaply here without making this a stateful call, so
     * default to the newer 0.10 K/LSB and clamp. */
    int v = (int)lroundf(kelvin / 0.10f);
    v = clampi(v, -127, 127);
    return mi1602_reg_write(h, MI48_REG_OFFSET_CORR, (uint8_t)(int8_t)v);
}

esp_err_t mi1602_set_sens_factor(mi1602_handle_t h, float factor)
{
    int v = (int)lroundf(factor * 100.0f);
    v = clampi(v, 0, 255);
    return mi1602_reg_write(h, MI48_REG_SENS_FACTOR, (uint8_t)v);
}

esp_err_t mi1602_set_otf(mi1602_handle_t h, float factor)
{
    /* OBJ_TEMP_FACTOR is signed, unit 0.01. */
    int v = (int)lroundf(factor * 100.0f);
    v = clampi(v, -127, 127);
    return mi1602_reg_write(h, MI48_REG_OBJ_TEMP_FACTOR, (uint8_t)(int8_t)v);
}

esp_err_t mi1602_set_filter_temporal(mi1602_handle_t h, bool enable, uint16_t strength)
{
    esp_err_t err;
    err = mi1602_reg_write(h, MI48_REG_FILTER_1_LSB, (uint8_t)(strength & 0xFF));
    if (err != ESP_OK) return err;
    err = mi1602_reg_write(h, MI48_REG_FILTER_1_MSB, (uint8_t)((strength >> 8) & 0xFF));
    if (err != ESP_OK) return err;
    return mi1602_reg_write(h, MI48_REG_FILTER_1, enable ? 0x03 : 0x00);
}

esp_err_t mi1602_set_filter_stark(mi1602_handle_t h, bool enable)
{
    return mi1602_reg_write(h, MI48_REG_FILTER_2, enable ? 0x03 : 0x00);
}

esp_err_t mi1602_set_filter_median(mi1602_handle_t h, bool enable)
{
    return mi1602_reg_write(h, MI48_REG_FILTER_3, enable ? 0x01 : 0x00);
}

esp_err_t mi1602_set_mms(mi1602_handle_t h, bool enable)
{
    return mi1602_reg_write(h, MI48_REG_MMS_CTRL, enable ? 0x01 : 0x00);
}

/* ----------------------------------------------------------------------- */
/*  Frame header parsing (pysenxor mi48.py:1206-1251)                      */
/* ----------------------------------------------------------------------- */

static void mi1602_parse_header(const uint16_t *hdr, mi1602_frame_header_t *out)
{
    static const float KELVIN_0 = -273.15f;

    out->frame_counter      = hdr[MI48_HDR_FRCNT];
    out->senxor_vdd         = (float)hdr[MI48_HDR_SXVDD] / 10000.0f;
    out->senxor_temperature = (float)hdr[MI48_HDR_SXTA]  / 100.0f + KELVIN_0;
    out->timestamp          = ((uint32_t)hdr[MI48_HDR_TIME_HI] << 16) | (uint32_t)hdr[MI48_HDR_TIME_LO];
    out->pixel_max_c        = (float)hdr[MI48_HDR_MAXV] / 10.0f + KELVIN_0;
    out->pixel_min_c        = (float)hdr[MI48_HDR_MINV] / 10.0f + KELVIN_0;
    out->crc_from_header    = hdr[MI48_HDR_CRC];
    out->iplock1            = hdr[MI48_HDR_IPLOCK1];
    out->iplock2            = hdr[MI48_HDR_IPLOCK2];
    /* crc_ok is filled in by the capture path. */
}

/* ----------------------------------------------------------------------- */
/*  Wait for DATA_READY                                                    */
/* ----------------------------------------------------------------------- */

static esp_err_t mi1602_wait_data_ready(struct mi1602_t *m, uint32_t timeout_ms)
{
    int64_t t0 = esp_timer_get_time();
    while ((esp_timer_get_time() - t0) < (int64_t)timeout_ms * 1000) {
        if (m->cfg.data_ready_gpio >= 0) {
            if (gpio_get_level(m->cfg.data_ready_gpio)) return ESP_OK;
        } else {
            uint8_t st = 0;
            if (mi1602_reg_read((mi1602_handle_t)m, MI48_REG_STATUS, &st) == ESP_OK
                && (st & MI48_STATUS_DATA_READY)) {
                return ESP_OK;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(MI1602_DATA_READY_POLL_MS));
    }
    return ESP_ERR_TIMEOUT;
}

/* ----------------------------------------------------------------------- */
/*  Single capture                                                         */
/*  pysenxor example/single_capture_spi.py:28-59                           */
/* ----------------------------------------------------------------------- */

esp_err_t mi1602_capture_single(mi1602_handle_t h,
                                uint16_t *out_pixels,
                                mi1602_frame_header_t *out_hdr)
{
    if (h == NULL || out_pixels == NULL) return ESP_ERR_INVALID_ARG;

    const bool want_header = (out_hdr != NULL);
    uint8_t mode = MI1602_MODE_SINGLE_FRAME;
    if (!want_header) mode |= MI1602_MODE_NO_HEADER;

    ESP_RETURN_ON_ERROR(mi1602_reg_write(h, MI48_REG_FRAME_MODE, mode),
                        TAG, "capture: write FRAME_MODE");

    esp_err_t err = mi1602_wait_data_ready(h, MI1602_DATA_READY_TIMEOUT_MS);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "capture: DATA_READY timeout");
        /* try to leave the chip in a clean state */
        mi1602_reg_write(h, MI48_REG_FRAME_MODE, 0x00);
        return err;
    }

    uint16_t hdr_words[MI1602_FRAME_HDR_WORDS];
    err = mi1602_spi_read_frame(h, out_pixels, want_header ? hdr_words : NULL);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "capture: SPI read failed: %s", esp_err_to_name(err));
        return err;
    }

    if (want_header) {
        mi1602_parse_header(hdr_words, out_hdr);
        uint16_t crc_calc = mi1602_crc16_ccitt_false(out_pixels, MI1602_FPA_PIXELS);
        out_hdr->crc_ok = (crc_calc == out_hdr->crc_from_header);
        if (!out_hdr->crc_ok) {
            ESP_LOGW(TAG, "capture: CRC mismatch header=0x%04x calc=0x%04x",
                     out_hdr->crc_from_header, crc_calc);
        }
    }
    return ESP_OK;
}

esp_err_t mi1602_capture_no_trigger(mi1602_handle_t h, uint16_t *out_pixels,
                                    mi1602_frame_header_t *out_hdr)
{
    if (h == NULL || out_pixels == NULL) return ESP_ERR_INVALID_ARG;
    const bool want_header = (out_hdr != NULL);

    /* No FRAME_MODE write -- rely on the module already streaming / having a
     * frame pending. Wait for DATA_READY (GPIO if wired, else STATUS poll),
     * then read straight off SPI. This path never touches the I2C control
     * write that NACKs while the module is stuck in BOOTING_UP. */
    esp_err_t err = mi1602_wait_data_ready(h, MI1602_DATA_READY_TIMEOUT_MS);
    if (err != ESP_OK) return err;

    uint16_t hdr_words[MI1602_FRAME_HDR_WORDS];
    err = mi1602_spi_read_frame(h, out_pixels, want_header ? hdr_words : NULL);
    if (err != ESP_OK) return err;

    if (want_header) {
        mi1602_parse_header(hdr_words, out_hdr);
        uint16_t crc_calc = mi1602_crc16_ccitt_false(out_pixels, MI1602_FPA_PIXELS);
        out_hdr->crc_ok = (crc_calc == out_hdr->crc_from_header);
    }
    return ESP_OK;
}

esp_err_t mi1602_read_frame_now(mi1602_handle_t h, uint16_t *out_pixels,
                                mi1602_frame_header_t *out_hdr)
{
    if (h == NULL || out_pixels == NULL) return ESP_ERR_INVALID_ARG;
    const bool want_header = (out_hdr != NULL);

    /* Straight to the SPI read -- no FRAME_MODE write, no DATA_READY wait. */
    uint16_t hdr_words[MI1602_FRAME_HDR_WORDS];
    esp_err_t err = mi1602_spi_read_frame(h, out_pixels, want_header ? hdr_words : NULL);
    if (err != ESP_OK) return err;

    if (want_header) {
        mi1602_parse_header(hdr_words, out_hdr);
        uint16_t crc_calc = mi1602_crc16_ccitt_false(out_pixels, MI1602_FPA_PIXELS);
        out_hdr->crc_ok = (crc_calc == out_hdr->crc_from_header);
    }
    return ESP_OK;
}

/* ----------------------------------------------------------------------- */
/*  Streaming                                                              */
/* ----------------------------------------------------------------------- */

static void IRAM_ATTR mi1602_data_ready_isr(void *arg)
{
    struct mi1602_t *m = (struct mi1602_t *)arg;
    BaseType_t hpw = pdFALSE;
    xSemaphoreGiveFromISR(m->frame_ready_sem, &hpw);
    if (hpw == pdTRUE) portYIELD_FROM_ISR();
}

static void mi1602_stream_task(void *arg)
{
    struct mi1602_t *m = (struct mi1602_t *)arg;

    uint16_t *pixels = heap_caps_malloc(MI1602_FRAME_PIXEL_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (pixels == NULL) {
        ESP_LOGE(TAG, "stream: alloc pixels buffer failed");
        m->streaming = false;
        vTaskDelete(NULL);
        return;
    }
    uint16_t hdr_words[MI1602_FRAME_HDR_WORDS];
    mi1602_frame_header_t hdr;

    while (m->streaming) {
        bool got = false;
        if (m->cfg.data_ready_gpio >= 0) {
            got = (xSemaphoreTake(m->frame_ready_sem, pdMS_TO_TICKS(200)) == pdTRUE);
        } else {
            uint8_t st = 0;
            if (mi1602_reg_read((mi1602_handle_t)m, MI48_REG_STATUS, &st) == ESP_OK
                && (st & MI48_STATUS_DATA_READY)) {
                got = true;
            } else {
                vTaskDelay(pdMS_TO_TICKS(5));
            }
        }
        if (!got) continue;

        esp_err_t err = mi1602_spi_read_frame(m, pixels, hdr_words);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "stream: SPI read failed: %s", esp_err_to_name(err));
            continue;
        }
        mi1602_parse_header(hdr_words, &hdr);
        uint16_t crc_calc = mi1602_crc16_ccitt_false(pixels, MI1602_FPA_PIXELS);
        hdr.crc_ok = (crc_calc == hdr.crc_from_header);

        if (m->frame_cb) {
            m->frame_cb(pixels, &hdr, m->frame_cb_ctx);
        }
    }

    heap_caps_free(pixels);
    vTaskDelete(NULL);
}

esp_err_t mi1602_start_streaming(mi1602_handle_t h, mi1602_frame_cb_t cb, void *user_ctx)
{
    if (h == NULL || cb == NULL) return ESP_ERR_INVALID_ARG;
    if (h->streaming) return ESP_ERR_INVALID_STATE;

    h->frame_cb     = cb;
    h->frame_cb_ctx = user_ctx;
    h->frame_ready_sem = xSemaphoreCreateBinary();
    if (h->frame_ready_sem == NULL) return ESP_ERR_NO_MEM;

    if (h->cfg.data_ready_gpio >= 0) {
        gpio_set_intr_type(h->cfg.data_ready_gpio, GPIO_INTR_POSEDGE);
        esp_err_t err = gpio_install_isr_service(0);
        if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
            ESP_LOGE(TAG, "stream: gpio_install_isr_service failed: %s", esp_err_to_name(err));
            vSemaphoreDelete(h->frame_ready_sem);
            h->frame_ready_sem = NULL;
            return err;
        }
        gpio_isr_handler_add(h->cfg.data_ready_gpio, mi1602_data_ready_isr, h);
        gpio_intr_enable(h->cfg.data_ready_gpio);
    }

    h->streaming = true;
    BaseType_t ok = xTaskCreate(mi1602_stream_task, "mi1602_stream", 4096, h, 5,
                                &h->stream_task);
    if (ok != pdPASS) {
        h->streaming = false;
        if (h->cfg.data_ready_gpio >= 0) {
            gpio_intr_disable(h->cfg.data_ready_gpio);
            gpio_isr_handler_remove(h->cfg.data_ready_gpio);
        }
        vSemaphoreDelete(h->frame_ready_sem);
        h->frame_ready_sem = NULL;
        return ESP_ERR_NO_MEM;
    }

    return mi1602_reg_write(h, MI48_REG_FRAME_MODE, MI1602_MODE_CONTINUOUS_STREAM);
}

esp_err_t mi1602_stop_streaming(mi1602_handle_t h)
{
    if (h == NULL) return ESP_ERR_INVALID_ARG;
    if (!h->streaming) return ESP_OK;

    h->streaming = false;
    mi1602_reg_write(h, MI48_REG_FRAME_MODE, 0x00);

    if (h->cfg.data_ready_gpio >= 0) {
        gpio_intr_disable(h->cfg.data_ready_gpio);
        gpio_isr_handler_remove(h->cfg.data_ready_gpio);
    }
    /* Give the task a chance to observe streaming=false. The task deletes
     * itself; we just wait briefly. */
    for (int i = 0; i < 50 && h->stream_task != NULL; ++i) {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (eTaskGetState(h->stream_task) == eDeleted) break;
    }
    h->stream_task = NULL;

    if (h->frame_ready_sem) {
        vSemaphoreDelete(h->frame_ready_sem);
        h->frame_ready_sem = NULL;
    }
    return ESP_OK;
}

/* ----------------------------------------------------------------------- */
/*  Lifecycle                                                              */
/* ----------------------------------------------------------------------- */

esp_err_t mi1602_init(const mi1602_config_t *cfg, mi1602_handle_t *out)
{
    if (cfg == NULL || out == NULL) return ESP_ERR_INVALID_ARG;
    if (cfg->i2c_bus == NULL) return ESP_ERR_INVALID_ARG;
    if (cfg->sclk_gpio < 0 || cfg->miso_gpio < 0 || cfg->mosi_gpio < 0
        || cfg->cs_n_gpio < 0) {
        return ESP_ERR_INVALID_ARG;
    }

    struct mi1602_t *m = calloc(1, sizeof(*m));
    if (m == NULL) return ESP_ERR_NO_MEM;
    m->cfg            = *cfg;
    m->sysclk_ledc_ch = -1;

    /* GPIOs */
    esp_err_t err = mi1602_gpio_output(cfg->cs_n_gpio, 1);
    if (err != ESP_OK) goto fail;
    err = mi1602_gpio_output(cfg->reset_n_gpio, 1);
    if (err != ESP_OK) goto fail;
    err = mi1602_gpio_input (cfg->data_ready_gpio);
    if (err != ESP_OK) goto fail;

    /* Optional sysclk before talking to the chip. */
    err = mi1602_sysclk_start(m);
    if (err != ESP_OK) goto fail;

    /* I²C device */
    i2c_device_config_t devcfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = cfg->i2c_addr ? cfg->i2c_addr : MI1602_I2C_ADDR_DEFAULT,
        .scl_speed_hz    = cfg->i2c_freq_hz ? cfg->i2c_freq_hz : 100000,
        /* Tolerate MI48 clock-stretching. The MI48 holds SCL low while it is
         * busy -- notably during boot and between register accesses. With the
         * default (0) the master gives up too early: longer writes (reg+data)
         * NACK while shorter reads occasionally squeak through, which is exactly
         * the "reads intermittent, writes all NACK, stuck BOOTING_UP" symptom we
         * saw. 50 ms matches the sc850sl driver, which needs the same tolerance.
         * (A newer MI48 batch that boots slower / stretches more makes the
         * missing wait even more fatal.) */
        .scl_wait_us     = 50 * 1000,
    };
    err = i2c_master_bus_add_device(cfg->i2c_bus, &devcfg, &m->i2c_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_master_bus_add_device: %s", esp_err_to_name(err));
        goto fail;
    }

    /* SPI bus + device */
    err = mi1602_spi_init(m);
    if (err != ESP_OK) goto fail;

    ESP_LOGI(TAG, "ready: i2c_addr=0x%02x spi_host=%d cs=%d rst=%d dr=%d",
             devcfg.device_address, (int)cfg->spi_host, cfg->cs_n_gpio,
             cfg->reset_n_gpio, cfg->data_ready_gpio);

    *out = m;
    return ESP_OK;

fail:
    if (m->i2c_dev) i2c_master_bus_rm_device(m->i2c_dev);
    mi1602_sysclk_stop(m);
    free(m);
    return err;
}

void mi1602_deinit(mi1602_handle_t h)
{
    if (h == NULL) return;
    if (h->streaming) mi1602_stop_streaming(h);
    /* Best-effort: stop capture. */
    mi1602_reg_write(h, MI48_REG_FRAME_MODE, 0x00);

    mi1602_spi_deinit(h);
    if (h->i2c_dev) i2c_master_bus_rm_device(h->i2c_dev);
    mi1602_sysclk_stop(h);

    if (h->cfg.reset_n_gpio >= 0) gpio_set_level(h->cfg.reset_n_gpio, 0);
    free(h);
}
