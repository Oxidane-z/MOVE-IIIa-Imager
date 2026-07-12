/*
 * MI1602 thermal-camera CAPTURE test.
 *
 * Selected by CONFIG_CAMERA_TARGET_MI1602_TEST. Brings up ONLY the MI1602.
 *
 * History: the MI48 would never finish booting until RESET_N was wired.  The
 * carrier left RESET_N (active-low) floating, so the MI48 sat held in reset ->
 * silent I2C (probe timeouts) or, on a marginal power-on, a mis-latched ADDR
 * strap (answering at 0x40 instead of the strapped-high 0x41) stuck in
 * BOOTING_UP + SXIF_ERROR.  With RESET_N tied to 3V3 the chip boots cleanly:
 * ACK at 0x41, STATUS=0x00.  See datasheet MI48D5 Table 27 (ADDR high -> 0x41).
 *
 * This build: detect the I2C address, bring the driver up, then trigger
 * single-frame captures over SPI and print the thermal stats.
 */
#include "sdkconfig.h"
#include "app.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/i2c_master.h"
#include "mi1602.h"

static const char *TAG = "mi1602_cap";

void app_run(void)
{
    ESP_LOGI(TAG, "=== MI1602 capture test (RESET_N must be tied to 3V3) ===");
    ESP_LOGI(TAG, "I2C port=%d SDA=%d SCL=%d",
             CONFIG_MI1602_I2C_PORT, CONFIG_MI1602_I2C_SDA_GPIO, CONFIG_MI1602_I2C_SCL_GPIO);

    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = CONFIG_MI1602_I2C_PORT,
        .sda_io_num        = CONFIG_MI1602_I2C_SDA_GPIO,
        .scl_io_num        = CONFIG_MI1602_I2C_SCL_GPIO,
        .clk_source        = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    i2c_master_bus_handle_t bus = NULL;
    esp_err_t r = i2c_new_master_bus(&bus_cfg, &bus);
    if (r != ESP_OK) { ESP_LOGE(TAG, "i2c bus: %s -- halting", esp_err_to_name(r)); goto idle; }
    i2c_master_bus_reset(bus);

    /* Small settle; the MI48 is already up (RESET_N held high) but give the
     * rails a moment after the P4 reset. */
    vTaskDelay(pdMS_TO_TICKS(2000));

    /* Detect which address the MI48 answers on (0x41 when ADDR strapped high
     * and reset was clean; 0x40 otherwise). */
    uint8_t addr = 0;
    for (int i = 0; i < 2; ++i) {
        uint8_t a = (i == 0) ? MI1602_I2C_ADDR_ALT : MI1602_I2C_ADDR_DEFAULT;  /* try 0x41 first */
        esp_err_t p = i2c_master_probe(bus, a, 100);
        ESP_LOGI(TAG, "probe 0x%02x: %s", a, esp_err_to_name(p));
        if (p == ESP_OK) { addr = a; break; }
    }
    if (!addr) {
        ESP_LOGE(TAG, "no MI48 ACK on 0x40/0x41 -- is RESET_N tied to 3V3? halting");
        goto idle;
    }
    ESP_LOGI(TAG, "using I2C addr 0x%02x", addr);

    mi1602_config_t cfg = {
        .i2c_bus = bus, .i2c_addr = addr, .i2c_freq_hz = CONFIG_MI1602_I2C_FREQ_HZ,
        .spi_host = CONFIG_MI1602_SPI_HOST, .spi_bus_already_initialised = false,
        .sclk_gpio = CONFIG_MI1602_SPI_SCLK_GPIO, .miso_gpio = CONFIG_MI1602_SPI_MISO_GPIO,
        .mosi_gpio = CONFIG_MI1602_SPI_MOSI_GPIO, .cs_n_gpio = CONFIG_MI1602_SPI_CS_GPIO,
        .spi_freq_hz = CONFIG_MI1602_SPI_FREQ_HZ, .reset_n_gpio = CONFIG_MI1602_RESET_GPIO,
        /* Force the STATUS-poll DATA_READY path: I2C is proven good, and this
         * avoids depending on the DATA_READY GPIO wiring for a first capture. */
        .data_ready_gpio = -1, .sysclk_gpio = CONFIG_MI1602_SYSCLK_GPIO,
        .sysclk_hz = CONFIG_MI1602_SYSCLK_HZ,
    };
    mi1602_handle_t cam = NULL;
    r = mi1602_init(&cfg, &cam);
    if (r != ESP_OK) { ESP_LOGE(TAG, "mi1602_init: %s -- halting", esp_err_to_name(r)); goto idle; }

    esp_err_t pr = mi1602_probe(cam);
    ESP_LOGI(TAG, "probe: %s", esp_err_to_name(pr));

    uint8_t st0 = 0xff, md0 = 0xff;
    mi1602_get_status(cam, &st0);
    mi1602_get_mode(cam, &md0);
    ESP_LOGI(TAG, "pre-boot STATUS=0x%02x MODE=0x%02x", st0, md0);

    esp_err_t br = mi1602_bootup(cam);
    ESP_LOGI(TAG, "bootup: %s", esp_err_to_name(br));

    ESP_LOGW(TAG, ">>> single-frame capture loop");

    static uint16_t px[MI1602_FPA_PIXELS];
    int good = 0;
    bool dumped = false;
    for (int n = 0; ; ++n) {
        mi1602_frame_header_t hdr;
        esp_err_t cr = mi1602_capture_single(cam, px, &hdr);
        if (cr != ESP_OK) {
            uint8_t st = 0xff;
            mi1602_get_status(cam, &st);
            ESP_LOGW(TAG, "[%d] capture_single: %s (STATUS=0x%02x)", n, esp_err_to_name(cr), st);
        } else {
            /* One-time raw dump of a full frame so the PC side can (a) render a
             * real thermal image and (b) try CRC variants offline against the
             * header CRC -- no more firmware iterations to chase the CRC. px[]
             * holds host-order (byte-swapped) uint16 values; %04x prints the
             * numeric value MSB-first, so the wire bytes are hi=(v>>8) lo=(v&0xff). */
            if (!dumped) {
                dumped = true;
                printf("\nFRAMEDUMP_BEGIN cols=%d rows=%d fc=%u crc_hdr=0x%04x "
                       "hdr_min=%.2f hdr_max=%.2f die=%.2f\n",
                       MI1602_FPA_COLS, MI1602_FPA_ROWS, (unsigned)hdr.frame_counter,
                       hdr.crc_from_header, (double)hdr.pixel_min_c,
                       (double)hdr.pixel_max_c, (double)hdr.senxor_temperature);
                for (int i = 0; i < MI1602_FPA_PIXELS; ++i) {
                    printf("%04x", px[i]);
                    if ((i & 63) == 63) printf("\n");
                }
                printf("FRAMEDUMP_END\n");
            }
            uint16_t pmn = 0xffff, pmx = 0;
            uint64_t sum = 0;
            int sane = 0;
            for (int i = 0; i < MI1602_FPA_PIXELS; ++i) {
                uint16_t v = px[i];
                if (v != 0x0000 && v != 0xFFFF) ++sane;
                if (v < pmn) pmn = v;
                if (v > pmx) pmx = v;
                sum += v;
            }
            uint16_t pmean = (uint16_t)(sum / MI1602_FPA_PIXELS);
            if (hdr.crc_ok) ++good;
            ESP_LOGI(TAG, "[%d] #%u crc=%s die=%.1fC | px sane=%d/%d "
                     "C[min=%.1f mean=%.1f max=%.1f] good=%d",
                     n, (unsigned)hdr.frame_counter, hdr.crc_ok ? "OK" : "BAD",
                     (double)(hdr.senxor_temperature),
                     sane, MI1602_FPA_PIXELS,
                     (double)mi1602_dk_to_celsius(pmn),
                     (double)mi1602_dk_to_celsius(pmean),
                     (double)mi1602_dk_to_celsius(pmx), good);
        }
        vTaskDelay(pdMS_TO_TICKS(300));
    }

idle:
    ESP_LOGW(TAG, "MI1602 capture halted -- idling (reset the P4 to retry)");
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}
