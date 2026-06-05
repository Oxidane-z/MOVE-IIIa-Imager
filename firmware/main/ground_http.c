/*
 * ground_http.c -- GROUND-TEST ONLY web control server (esp_http_server).
 * Fully gated on CONFIG_GROUND_WIFI_ENABLE; the flight build compiles this to
 * nothing. Started from ground_wifi.c once the station has an IP.
 *
 * Snapshot/mailbox model (see ground_station.h): the camera task publishes
 * telemetry + the latest preview under a mutex; httpd handlers read consistent
 * snapshots and post commands the camera task drains once per frame. No httpd
 * handler ever touches a live camera buffer.
 *
 * Endpoints (filled in across P2-P4):
 *   GET  /            the control page (embedded ground_index.html)
 *   GET  /api/tlm     telemetry JSON
 *   POST /api/cmd     control (AE target/enable, manual exp/gain, sstv, capture)
 *   GET  /snapshot.jpg, GET /stream   live preview          [P3]
 *   GET  /thermal.jpg, GET /capture.raw, POST /api/ota, GET /api/log  [P4]
 */
#include "ground_station.h"
#include "sdkconfig.h"

#if CONFIG_GROUND_WIFI_ENABLE

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "ground_wifi.h"

static const char *TAG = "ground/http";

/* The control page, embedded via EMBED_FILES in main/CMakeLists.txt. */
extern const uint8_t index_html_start[] asm("_binary_ground_index_html_start");
extern const uint8_t index_html_end[]   asm("_binary_ground_index_html_end");

static SemaphoreHandle_t s_mtx;
static ground_tlm_t      s_tlm;
static bool              s_tlm_valid;
static uint16_t         *s_prev;        /* latest preview RGB565 (PSRAM)   */
static size_t            s_prev_cap, s_prev_bytes;
static uint16_t          s_prev_w, s_prev_h;
static ground_cmd_t      s_cmd;
static bool              s_cmd_pending;
static httpd_handle_t    s_server;

static inline void lock(void)   { if (s_mtx) xSemaphoreTake(s_mtx, portMAX_DELAY); }
static inline void unlock(void) { if (s_mtx) xSemaphoreGive(s_mtx); }

/* ---- app -> server ---- */
void ground_publish_tlm(const ground_tlm_t *t)
{
    lock(); s_tlm = *t; s_tlm_valid = true; unlock();
}

void ground_publish_preview(const void *rgb565, size_t bytes, uint16_t w, uint16_t h)
{
    lock();
    if (bytes > s_prev_cap) {
        free(s_prev);
        s_prev = heap_caps_malloc(bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        s_prev_cap = s_prev ? bytes : 0;
    }
    if (s_prev && bytes <= s_prev_cap) {
        memcpy(s_prev, rgb565, bytes);
        s_prev_bytes = bytes; s_prev_w = w; s_prev_h = h;
    }
    unlock();
}

/* ---- server -> app ---- */
void ground_cmd_post(const ground_cmd_t *c) { lock(); s_cmd = *c; s_cmd_pending = true; unlock(); }

bool ground_cmd_take(ground_cmd_t *o)
{
    lock(); bool p = s_cmd_pending; if (p) { *o = s_cmd; s_cmd_pending = false; } unlock();
    return p;
}

/* ---- server-side accessors ---- */
bool ground_tlm_snapshot(ground_tlm_t *o)
{
    lock(); bool v = s_tlm_valid; if (v) *o = s_tlm; unlock(); return v;
}

size_t ground_preview_copy(void *dst, size_t max, uint16_t *w, uint16_t *h)
{
    lock(); size_t n = 0;
    if (s_prev && s_prev_bytes && s_prev_bytes <= max) {
        memcpy(dst, s_prev, s_prev_bytes); n = s_prev_bytes; *w = s_prev_w; *h = s_prev_h;
    }
    unlock(); return n;
}

/* ---- handlers ---- */
static esp_err_t root_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, (const char *)index_html_start,
                           index_html_end - index_html_start);
}

static esp_err_t tlm_get(httpd_req_t *req)
{
    ground_tlm_t t; memset(&t, 0, sizeof t);
    bool v = ground_tlm_snapshot(&t);
    char ip[16] = "0.0.0.0"; ground_wifi_get_ip(ip, sizeof ip);
    int rssi = 0; wifi_ap_record_t ap;
    if (esp_wifi_sta_get_ap_info(&ap) == ESP_OK) rssi = ap.rssi;

    char buf[700];
    int n = snprintf(buf, sizeof buf,
        "{\"ok\":%d,\"ip\":\"%s\",\"rssi\":%d,\"up\":%lld,"
        "\"seq\":%u,\"w\":%u,\"h\":%u,\"min\":%d,\"max\":%d,\"mean\":%d,\"sat_ppm\":%d,"
        "\"exp\":%u,\"gain\":%u,\"bl\":%d,\"wbr\":%.2f,\"wbb\":%.2f,"
        "\"ae_target\":%d,\"ae_en\":%d,\"cap_us\":%lld,\"isp_us\":%lld,\"usb_us\":%lld,"
        "\"stream\":%d,\"thermal\":%d}",
        v ? 1 : 0, ip, rssi, (long long)(esp_timer_get_time() / 1000000),
        (unsigned)t.seq, (unsigned)t.w, (unsigned)t.h,
        t.raw_min, t.raw_max, t.raw_mean, t.raw_sat_ppm,
        (unsigned)t.exp_lines, (unsigned)t.gain_x1024, t.black_level, t.wb_r, t.wb_b,
        t.ae_target, t.ae_enabled ? 1 : 0,
        (long long)t.t_cap_us, (long long)t.t_isp_us, (long long)t.t_usb_us,
        t.cam_streaming ? 1 : 0, t.thermal_ok ? 1 : 0);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, n);
}

/* Parse one int query param; returns def if absent. */
static int qparam(httpd_req_t *req, const char *key, int def)
{
    char q[256], val[24];
    if (httpd_req_get_url_query_str(req, q, sizeof q) == ESP_OK &&
        httpd_query_key_value(q, key, val, sizeof val) == ESP_OK)
        return atoi(val);
    return def;
}

static esp_err_t cmd_post(httpd_req_t *req)
{
    ground_cmd_t c = {
        .ae_target    = qparam(req, "ae_target", -1),
        .ae_enabled   = qparam(req, "ae_en",     -1),
        .exp_lines    = qparam(req, "exp",       -1),
        .gain_x1024   = qparam(req, "gain",      -1),
        .sstv_trigger = qparam(req, "sstv",       0) != 0,
        .capture_hd   = qparam(req, "capture",    0) != 0,
    };
    ground_cmd_post(&c);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":1}");
}

esp_err_t ground_http_start(void)
{
    if (s_server) return ESP_OK;                 /* already running */
    if (!s_mtx) { s_mtx = xSemaphoreCreateMutex(); if (!s_mtx) return ESP_ERR_NO_MEM; }

    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port      = 80;
    cfg.lru_purge_enable = true;     /* free LRU connection when full (MJPEG holds one) */
    cfg.max_uri_handlers = 12;       /* room for P3/P4 endpoints */
    cfg.stack_size       = 8192;     /* JSON + JPEG work needs more than the 4 KB default */

    esp_err_t r = httpd_start(&s_server, &cfg);
    if (r != ESP_OK) { ESP_LOGE(TAG, "httpd_start: %s", esp_err_to_name(r)); return r; }

    httpd_uri_t u_root = { .uri = "/",        .method = HTTP_GET,  .handler = root_get };
    httpd_uri_t u_tlm  = { .uri = "/api/tlm", .method = HTTP_GET,  .handler = tlm_get  };
    httpd_uri_t u_cmd  = { .uri = "/api/cmd", .method = HTTP_POST, .handler = cmd_post };
    httpd_register_uri_handler(s_server, &u_root);
    httpd_register_uri_handler(s_server, &u_tlm);
    httpd_register_uri_handler(s_server, &u_cmd);

    char ip[16] = "0.0.0.0"; ground_wifi_get_ip(ip, sizeof ip);
    ESP_LOGI(TAG, "web control server up at http://%s/", ip);
    return ESP_OK;
}

#endif /* CONFIG_GROUND_WIFI_ENABLE */
