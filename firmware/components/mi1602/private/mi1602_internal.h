/*
 * MI1602 driver internal types — not part of the public API.
 */
#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/spi_master.h"
#include "mi1602.h"

#ifdef __cplusplus
extern "C" {
#endif

struct mi1602_t {
    mi1602_config_t           cfg;

    /* I²C */
    i2c_master_dev_handle_t   i2c_dev;

    /* SPI */
    spi_device_handle_t       spi_dev;
    uint8_t                  *spi_rx_buf;    /* DMA-capable, big-endian uint16 from MI48 */
    uint8_t                  *spi_tx_buf;    /* DMA-capable, all zeros (dummy bytes) */
    size_t                    spi_xfer_bytes;

    /* Streaming */
    TaskHandle_t              stream_task;
    SemaphoreHandle_t         frame_ready_sem;   /* given from ISR or polling task */
    volatile bool             streaming;
    mi1602_frame_cb_t         frame_cb;
    void                     *frame_cb_ctx;

    /* LEDC sysclk channel (only valid if sysclk_gpio >= 0) */
    int                       sysclk_ledc_ch;
    bool                      sysclk_active;
};

/* Internal helpers — defined in mi1602_spi.c */
esp_err_t mi1602_spi_init  (struct mi1602_t *m);
esp_err_t mi1602_spi_deinit(struct mi1602_t *m);
esp_err_t mi1602_spi_read_frame(struct mi1602_t *m,
                                uint16_t *out_pixels,
                                uint16_t *out_header_words /* NULL → NO_HEADER */);

/* Internal helpers — defined in mi1602_crc.c */
uint16_t mi1602_crc16_ccitt_false(const uint16_t *data_words, size_t nwords);

#ifdef __cplusplus
}
#endif
