/*
 * SPI frame transport for the MI1602.
 *
 * Behavioural model (pysenxor interfaces.py:61-122,
 * pysenxor example/single_capture_spi.py:86-126):
 *   - Full-duplex, Mode 0, MSB-first, 8-bit word
 *   - CS_N must remain asserted across the ENTIRE frame transfer (header
 *     row + pixel rows). Python sets `spidev.no_cs = True` and drives CS_N
 *     by hand for that reason; we do the same here — the SPI device is
 *     registered with `spics_io_num = -1` and the driver pulses CS_N via
 *     GPIO around the transfer, with ~100 µs settle on each edge.
 *   - The MI48 transmits 16-bit words big-endian. The ESP receive buffer is
 *     therefore byte-swapped to host order after the transfer.
 *   - The master must push dummy bytes (we use 0x00) to clock the slave's
 *     bytes out. Our TX buffer is zero-initialised once and reused.
 *
 * DMA: the buffer is ~38 KB (header + pixels for MI1602's 160×120). We
 * allocate it from DMA-capable internal SRAM via heap_caps_malloc().
 */
#include <string.h>

#include "esp_check.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_rom_sys.h"
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"

#include "mi1602.h"
#include "mi1602_internal.h"

static const char *TAG = "mi1602_spi";

#define MI1602_SPI_CS_SETTLE_US   100

esp_err_t mi1602_spi_init(struct mi1602_t *m)
{
    if (!m->cfg.spi_bus_already_initialised) {
        spi_bus_config_t buscfg = {
            .mosi_io_num     = m->cfg.mosi_gpio,
            .miso_io_num     = m->cfg.miso_gpio,
            .sclk_io_num     = m->cfg.sclk_gpio,
            .quadwp_io_num   = -1,
            .quadhd_io_num   = -1,
            .max_transfer_sz = MI1602_FRAME_TOTAL_BYTES,
        };
        esp_err_t err = spi_bus_initialize(m->cfg.spi_host, &buscfg, SPI_DMA_CH_AUTO);
        if (err == ESP_ERR_INVALID_STATE) {
            ESP_LOGW(TAG, "spi_bus_initialize: bus already initialised");
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
            return err;
        }
    }

    spi_device_interface_config_t devcfg = {
        .mode           = 0,                 /* CPOL=0, CPHA=0 */
        .clock_speed_hz = (int)(m->cfg.spi_freq_hz ? m->cfg.spi_freq_hz : 15600000),
        .spics_io_num   = -1,                /* manual CS via cs_n_gpio */
        .queue_size     = 1,
        .flags          = 0,                 /* MSB-first is the default */
        .command_bits   = 0,
        .address_bits   = 0,
        .dummy_bits     = 0,
    };
    esp_err_t err = spi_bus_add_device(m->cfg.spi_host, &devcfg, &m->spi_dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_add_device: %s", esp_err_to_name(err));
        return err;
    }

    /* Persistent DMA-capable buffers, sized to the full (header + pixels). */
    m->spi_xfer_bytes = MI1602_FRAME_TOTAL_BYTES;
    m->spi_rx_buf = heap_caps_malloc(m->spi_xfer_bytes, MALLOC_CAP_DMA);
    m->spi_tx_buf = heap_caps_malloc(m->spi_xfer_bytes, MALLOC_CAP_DMA);
    if (m->spi_rx_buf == NULL || m->spi_tx_buf == NULL) {
        ESP_LOGE(TAG, "alloc DMA buffers (%u bytes ×2) failed",
                 (unsigned)m->spi_xfer_bytes);
        return ESP_ERR_NO_MEM;
    }
    memset(m->spi_tx_buf, 0x00, m->spi_xfer_bytes);
    return ESP_OK;
}

esp_err_t mi1602_spi_deinit(struct mi1602_t *m)
{
    if (m == NULL) return ESP_OK;
    if (m->spi_dev) {
        spi_bus_remove_device(m->spi_dev);
        m->spi_dev = NULL;
    }
    if (m->spi_rx_buf) { heap_caps_free(m->spi_rx_buf); m->spi_rx_buf = NULL; }
    if (m->spi_tx_buf) { heap_caps_free(m->spi_tx_buf); m->spi_tx_buf = NULL; }
    return ESP_OK;
}

esp_err_t mi1602_spi_read_frame(struct mi1602_t *m,
                                uint16_t *out_pixels,
                                uint16_t *out_header_words)
{
    if (m == NULL || out_pixels == NULL) return ESP_ERR_INVALID_ARG;

    const bool with_header = (out_header_words != NULL);
    const size_t total_bytes = with_header ? MI1602_FRAME_TOTAL_BYTES
                                           : MI1602_FRAME_PIXEL_BYTES;

    /* The ESP32-P4 GP-SPI hardware caps a single transaction at
     * SPI_MS_DATA_BITLEN = 2^18-1 bits ≈ 32 KB. A full 160×120×2 frame
     * (38,400 B, +320 B with header) exceeds that, so we split the read
     * into ≤16 KB chunks. CS_N is driven manually and held LOW across the
     * entire sequence — the MI48 just pauses on the SCLK gaps between
     * chunks and resumes clocking out the same frame, so the data stays
     * contiguous. */
    #define MI1602_SPI_CHUNK_BYTES  16384u

    gpio_set_level(m->cfg.cs_n_gpio, 0);
    esp_rom_delay_us(MI1602_SPI_CS_SETTLE_US);

    esp_err_t err = ESP_OK;
    for (size_t off = 0; off < total_bytes; off += MI1602_SPI_CHUNK_BYTES) {
        size_t n = total_bytes - off;
        if (n > MI1602_SPI_CHUNK_BYTES) n = MI1602_SPI_CHUNK_BYTES;
        spi_transaction_t t = {
            .length    = n * 8,
            .rxlength  = n * 8,
            .tx_buffer = m->spi_tx_buf ? (m->spi_tx_buf + off) : NULL,
            .rx_buffer = m->spi_rx_buf + off,
        };
        err = spi_device_polling_transmit(m->spi_dev, &t);
        if (err != ESP_OK) break;
    }

    esp_rom_delay_us(MI1602_SPI_CS_SETTLE_US);
    gpio_set_level(m->cfg.cs_n_gpio, 1);

    if (err != ESP_OK) return err;

    /* Big-endian uint16 → host order. */
    const uint8_t *src = m->spi_rx_buf;
    if (with_header) {
        for (size_t i = 0; i < MI1602_FRAME_HDR_WORDS; ++i) {
            out_header_words[i] = ((uint16_t)src[2 * i] << 8) | (uint16_t)src[2 * i + 1];
        }
        src += MI1602_FRAME_HDR_BYTES;
    }
    for (size_t i = 0; i < MI1602_FPA_PIXELS; ++i) {
        out_pixels[i] = ((uint16_t)src[2 * i] << 8) | (uint16_t)src[2 * i + 1];
    }
    return ESP_OK;
}
