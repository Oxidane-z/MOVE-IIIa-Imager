/*
 * CubeSat imaging payload — application entry.
 *
 * The actual logic lives in one of the per-target modules selected by
 * the CAMERA_TARGET_* Kconfig switch:
 *
 *   - CAMERA_TARGET_SC850SL       → app_sc850sl.c (flight target)
 *   - CAMERA_TARGET_SC202CS_TAB5  → app_sc202cs_tab5.c (bench / Tab5)
 *
 * This file only does FreeRTOS-level wiring; everything else lives in
 * the per-target module. Keeps each path easy to read in isolation.
 */
#include "app.h"
#include "esp_log.h"

static const char *TAG = "main";

void app_main(void)
{
#if defined(CONFIG_CAMERA_TARGET_SC850SL)
    ESP_LOGI(TAG, "CubeSat imager — built for SC850SL (flight target)");
#elif defined(CONFIG_CAMERA_TARGET_SC202CS_TAB5)
    ESP_LOGI(TAG, "CubeSat imager — built for SC202CS on M5Stack Tab5");
#elif defined(CONFIG_CAMERA_TARGET_MI1602_TEST)
    ESP_LOGI(TAG, "CubeSat imager — MI1602 thermal-only bring-up (debug)");
#else
#error "No CAMERA_TARGET selected. Run idf.py menuconfig."
#endif

    /* Hand off to the target-specific application. It never returns.
     * (Ground-test WiFi is started from INSIDE app_run, after the camera is up
     * — bringing the C6/SDIO up before the sensor's stream-on wedged its I²C
     * bus. See the ground_wifi_start() call at the idle: label in
     * app_sc850sl.c.) */
    app_run();
}
