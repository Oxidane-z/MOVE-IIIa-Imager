/*
 * RS485 full-duplex link test — bridges the RS485 UART <-> the USB console.
 *
 * Flash the SAME image to two boards, wire their THVD1424 transceivers together
 * (4-wire full-duplex + GND), and open each board's USB serial console on a
 * laptop. You then have a two-way serial chat over RS485:
 *
 *   - each board sends a "[<id>] hb #N" heartbeat once a second over RS485;
 *   - whatever arrives on RS485 is printed to the local console;
 *   - whatever you type on the console is sent out over RS485 (and echoed).
 *
 * So each laptop sees the OTHER board's heartbeat ticking (= the link works in
 * that direction) plus anything typed on the far end. Console SILENCE after the
 * banner = nothing is arriving over RS485 = check wiring / DE / termination.
 *
 * Wiring (THVD1424, full-duplex) — see README.md:
 *   P4 UART1 TX = GPIO38 -> transceiver DI (driver input)
 *   P4 UART1 RX = GPIO37 <- transceiver RO (receiver output)
 *   P4        DE = GPIO39 -> transceiver DE (driver enable, active-high)
 * (TX/RX are assigned to match the board wiring via the GPIO matrix — the
 *  transceiver's RO is on G37 and DI on G38, so G37=RX and G38=TX.)
 *   transceiver RE tied enabled in HW (or set RS485_RE_GPIO below)
 * Because it's full-duplex point-to-point, the driver is held enabled and the
 * receiver is always live — this is NORMAL UART mode, NOT the half-duplex RS485
 * mode (which would gate RX during TX and is meant for a 2-wire bus).
 */
#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "driver/usb_serial_jtag.h"
#include "esp_mac.h"

#define RS485_UART      UART_NUM_1
#define RS485_TX_GPIO   38        /* -> transceiver DI (driver input)   */
#define RS485_RX_GPIO   37        /* <- transceiver RO (receiver output) */
#define RS485_DE_GPIO   39        /* -> transceiver DE (active-high)   */
#define RS485_RE_GPIO   (-1)      /* -> transceiver RE (active-low); -1 if tied enabled in HW */
#define RS485_BAUD      115200
#define BUF_SZ          512

static char s_id[8];              /* short node id from the factory MAC */

static inline void con_write(const void *p, size_t n)
{
    usb_serial_jtag_write_bytes(p, n, pdMS_TO_TICKS(100));
}

/* RS485 -> console: print whatever the far board sent, verbatim. */
static void rs485_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[BUF_SZ];
    for (;;) {
        int n = uart_read_bytes(RS485_UART, buf, sizeof buf, pdMS_TO_TICKS(50));
        if (n > 0) con_write(buf, n);
    }
}

/* Console -> RS485: forward typed input, echo it locally so you see your typing. */
static void console_rx_task(void *arg)
{
    (void)arg;
    uint8_t buf[BUF_SZ];
    for (;;) {
        int n = usb_serial_jtag_read_bytes(buf, sizeof buf, pdMS_TO_TICKS(50));
        if (n > 0) {
            uart_write_bytes(RS485_UART, buf, n);
            con_write(buf, n);
        }
    }
}

/* Periodic heartbeat over RS485 so the link self-tests with no typing at all. */
static void heartbeat_task(void *arg)
{
    (void)arg;
    char line[48];
    unsigned n = 0;
    for (;;) {
        int len = snprintf(line, sizeof line, "[%s] hb #%u\r\n", s_id, n++);
        uart_write_bytes(RS485_UART, line, len);
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    /* Short node id from the factory MAC so the two boards are distinguishable. */
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_BASE);
    snprintf(s_id, sizeof s_id, "%02X%02X%02X", mac[3], mac[4], mac[5]);

    /* USB-Serial-JTAG console driver — buffered, used for both read and write. */
    usb_serial_jtag_driver_config_t usj = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
    usb_serial_jtag_driver_install(&usj);

    /* RS485 driver-enable: full-duplex point-to-point, so hold the driver ON. */
    gpio_config_t de_io = { .pin_bit_mask = 1ULL << RS485_DE_GPIO, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&de_io);
    gpio_set_level(RS485_DE_GPIO, 1);          /* DE high = driver enabled */
#if RS485_RE_GPIO >= 0
    gpio_config_t re_io = { .pin_bit_mask = 1ULL << RS485_RE_GPIO, .mode = GPIO_MODE_OUTPUT };
    gpio_config(&re_io);
    gpio_set_level(RS485_RE_GPIO, 0);          /* RE low = receiver enabled */
#endif

    /* RS485 UART — normal full-duplex mode (NOT UART_MODE_RS485_HALF_DUPLEX). */
    uart_config_t uc = {
        .baud_rate  = RS485_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(RS485_UART, BUF_SZ * 2, 0, 0, NULL, 0);
    uart_param_config(RS485_UART, &uc);
    uart_set_pin(RS485_UART, RS485_TX_GPIO, RS485_RX_GPIO,
                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    char banner[320];
    int bl = snprintf(banner, sizeof banner,
        "\r\n=== RS485 bridge | node %s | UART1 TX=%d RX=%d DE=%d | %d 8N1 full-duplex ===\r\n"
        "Type here to send over RS485; incoming RS485 prints below. Heartbeat every 1 s.\r\n"
        "(If nothing arrives, the other board's heartbeat is not reaching us -- check wiring/DE/term.)\r\n",
        s_id, RS485_TX_GPIO, RS485_RX_GPIO, RS485_DE_GPIO, RS485_BAUD);
    con_write(banner, bl);

    xTaskCreate(rs485_rx_task,   "rs485_rx", 4096, NULL, 11, NULL);
    xTaskCreate(console_rx_task, "con_rx",   4096, NULL, 10, NULL);
    xTaskCreate(heartbeat_task,  "hb",       3072, NULL, 5,  NULL);
}
