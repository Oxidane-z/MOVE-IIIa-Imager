/*
 * ground_wifi.c -- GROUND-TEST ONLY WiFi station bring-up via the M5 Stamp-AddOn
 * C6 (ESP32-C6 over SDIO). Compiles to trivial stubs in the flight build
 * (CONFIG_GROUND_WIFI_ENABLE off), so the flight image carries none of this.
 *
 * The ESP32-P4 has no radio: esp_wifi_remote proxies the standard esp_wifi STA
 * API to the C6 over the esp_hosted SDIO transport, which esp_wifi_remote
 * brings up transparently on esp_wifi_init(). The SDIO pinout (Slot-1 flexible
 * GPIOs 42-48) lives in sdkconfig.defaults.ground; credentials come from the
 * gitignored sdkconfig.ground.secret. See docs/FLIGHT_ARCHITECTURE.md s13.
 */
#include "ground_wifi.h"
#include "sdkconfig.h"

#if CONFIG_GROUND_WIFI_ENABLE

#include <string.h>
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "esp_event.h"
#include "nvs_flash.h"

static const char *TAG = "ground/wifi";

static volatile bool s_connected = false;
static char          s_ip[16]    = "0.0.0.0";

static void on_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();                       /* first attempt */
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_connected) ESP_LOGW(TAG, "WiFi disconnected; will retry");
        s_connected = false;
        strcpy(s_ip, "0.0.0.0");
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t *e = (ip_event_got_ip_t *)data;
        snprintf(s_ip, sizeof(s_ip), IPSTR, IP2STR(&e->ip_info.ip));
        s_connected = true;
        ESP_LOGI(TAG, "got ip: %s   (web control: http://%s/ )", s_ip, s_ip);
    }
}

/* Owns the reconnect cadence so a down/absent AP never tight-spins the system
 * event task. esp_wifi_connect() while already connecting/connected returns an
 * ignorable error. Logs are throttled so a missing AP doesn't flood the wire. */
static void wifi_task(void *arg)
{
    int down_ticks = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        if (s_connected) { down_ticks = 0; continue; }
        if ((down_ticks % 6) == 0) {              /* ~every 30 s while down */
            ESP_LOGI(TAG, "(re)connecting to SSID \"%s\"...", CONFIG_GROUND_WIFI_SSID);
        }
        ++down_ticks;
        esp_wifi_connect();
    }
}

esp_err_t ground_wifi_start(void)
{
    if (strlen(CONFIG_GROUND_WIFI_SSID) == 0) {
        ESP_LOGE(TAG, "SSID empty -- set CONFIG_GROUND_WIFI_SSID in sdkconfig.ground.secret");
        return ESP_ERR_INVALID_STATE;
    }

    /* WiFi needs NVS (PHY calibration + protocol state). Nothing else in this
     * firmware initializes NVS, so do it here. */
    esp_err_t r = nvs_flash_init();
    if (r == ESP_ERR_NVS_NO_FREE_PAGES || r == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        r = nvs_flash_init();
    }
    ESP_ERROR_CHECK(r);

    ESP_ERROR_CHECK(esp_netif_init());
    r = esp_event_loop_create_default();
    if (r != ESP_OK && r != ESP_ERR_INVALID_STATE) ESP_ERROR_CHECK(r);
    esp_netif_create_default_wifi_sta();

    /* esp_wifi_init() transparently brings up the C6 over SDIO through
     * esp_wifi_remote + esp_hosted (pins set by CONFIG_ESP_HOSTED_SDIO_* in
     * sdkconfig.defaults.ground). If the C6 slave firmware is incompatible this
     * is where it shows up -- as SDMMC errors or an init failure in the log. */
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                                        &on_event, NULL, NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                                        &on_event, NULL, NULL));

    wifi_config_t wc = { 0 };
    strlcpy((char *)wc.sta.ssid,     CONFIG_GROUND_WIFI_SSID,     sizeof(wc.sta.ssid));
    strlcpy((char *)wc.sta.password, CONFIG_GROUND_WIFI_PASSWORD, sizeof(wc.sta.password));
    wc.sta.threshold.authmode =
        (strlen(CONFIG_GROUND_WIFI_PASSWORD) == 0) ? WIFI_AUTH_OPEN : WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wc));
    ESP_ERROR_CHECK(esp_wifi_start());

    if (xTaskCreate(wifi_task, "ground_wifi", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "wifi_task create failed");
        return ESP_ERR_NO_MEM;
    }
    ESP_LOGI(TAG, "ground WiFi up (STA), joining \"%s\" via C6/SDIO", CONFIG_GROUND_WIFI_SSID);
    return ESP_OK;
}

bool ground_wifi_is_connected(void) { return s_connected; }

void ground_wifi_get_ip(char *out, size_t out_len)
{
    if (out && out_len) strlcpy(out, s_ip, out_len);
}

#else  /* !CONFIG_GROUND_WIFI_ENABLE -- flight build: stubs so callers link */

#include <string.h>

esp_err_t ground_wifi_start(void)            { return ESP_OK; }
bool      ground_wifi_is_connected(void)     { return false; }
void      ground_wifi_get_ip(char *out, size_t out_len)
{
    if (out && out_len) strlcpy(out, "0.0.0.0", out_len);
}

#endif /* CONFIG_GROUND_WIFI_ENABLE */
