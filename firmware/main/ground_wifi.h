/*
 * ground_wifi.h -- GROUND-TEST ONLY WiFi station + (later) web control via the
 * M5 Stamp-AddOn C6 (ESP32-C6 over SDIO / esp_wifi_remote + esp_hosted).
 *
 * All three functions are always defined: when CONFIG_GROUND_WIFI_ENABLE is
 * off (the flight build) they compile to trivial stubs, so callers never need
 * to #ifdef around them. See FLIGHT_ARCHITECTURE.md s13.
 */
#pragma once

#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Bring up the C6 over SDIO and join the configured WiFi as a station.
 * Non-blocking: spawns an internal task that connects and auto-reconnects.
 * No-op returning ESP_OK when CONFIG_GROUND_WIFI_ENABLE is off. */
esp_err_t ground_wifi_start(void);

/* True once the station has an IP lease. */
bool ground_wifi_is_connected(void);

/* Copy the current IPv4 dotted-quad ("0.0.0.0" if not connected) into out. */
void ground_wifi_get_ip(char *out, size_t out_len);

#ifdef __cplusplus
}
#endif
