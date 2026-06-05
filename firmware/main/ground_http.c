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
#include <stdarg.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "esp_http_server.h"
#include "driver/jpeg_encode.h"   /* P4 hardware JPEG encoder */
#include "esp_ota_ops.h"          /* OTA firmware update over WiFi */
#include "esp_system.h"           /* esp_restart */
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
static volatile int      s_stream_clients;   /* active /stream (MJPEG) viewers */

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

int ground_preview_clients(void) { return s_stream_clients; }

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
        .usb_push     = qparam(req, "usb",       -1),
        .sstv_trigger = qparam(req, "sstv",       0) != 0,
        .capture_hd   = qparam(req, "capture",    0) != 0,
    };
    ground_cmd_post(&c);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, "{\"ok\":1}");
}

/* ---- live preview: hardware JPEG encode (P3) ---------------------------- */
static jpeg_encoder_handle_t s_jpeg;
static SemaphoreHandle_t     s_jpeg_mtx;
static uint8_t              *s_jin;     /* RGB565 input  (DMA-capable) */
static size_t                s_jin_cap;
static uint8_t              *s_jout;    /* JPEG output   (DMA-capable) */
static size_t                s_jout_cap;

/* One composite preview frame (640x240 RGB565) + generous JPEG headroom. */
#define JPEG_IN_MAX   (640 * 240 * 2)
#define JPEG_OUT_MAX  (128 * 1024)

static esp_err_t jpeg_init(void)
{
    if (s_jpeg) return ESP_OK;
    s_jpeg_mtx = xSemaphoreCreateMutex();
    if (!s_jpeg_mtx) return ESP_ERR_NO_MEM;
    jpeg_encode_engine_cfg_t eng = { .timeout_ms = 200 };
    esp_err_t r = jpeg_new_encoder_engine(&eng, &s_jpeg);
    if (r != ESP_OK) { ESP_LOGE(TAG, "jpeg engine: %s", esp_err_to_name(r)); return r; }
    jpeg_encode_memory_alloc_cfg_t im = { .buffer_direction = JPEG_ENC_ALLOC_INPUT_BUFFER };
    jpeg_encode_memory_alloc_cfg_t om = { .buffer_direction = JPEG_ENC_ALLOC_OUTPUT_BUFFER };
    s_jin  = jpeg_alloc_encoder_mem(JPEG_IN_MAX,  &im, &s_jin_cap);
    s_jout = jpeg_alloc_encoder_mem(JPEG_OUT_MAX, &om, &s_jout_cap);
    if (!s_jin || !s_jout) { ESP_LOGE(TAG, "jpeg buffers: out of memory"); return ESP_ERR_NO_MEM; }
    return ESP_OK;
}

/* Copy + encode the latest preview into s_jout. Call with s_jpeg_mtx held. */
static esp_err_t encode_locked(uint32_t *out_len)
{
    uint16_t w = 0, h = 0;
    size_t n = ground_preview_copy(s_jin, s_jin_cap, &w, &h);
    if (!n || !w || !h) return ESP_ERR_INVALID_STATE;   /* no frame yet */
    jpeg_encode_cfg_t cfg = {
        .width = w, .height = h,
        .src_type      = JPEG_ENCODE_IN_FORMAT_RGB565,
        .sub_sample    = JPEG_DOWN_SAMPLING_YUV420,
        .image_quality = 80,
        .pixel_reverse = false,   /* flip if colours come out wrong */
    };
    return jpeg_encoder_process(s_jpeg, &cfg, s_jin, n, s_jout, s_jout_cap, out_len);
}

static esp_err_t snapshot_get(httpd_req_t *req)
{
    if (!s_jpeg) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no encoder"); return ESP_FAIL; }
    xSemaphoreTake(s_jpeg_mtx, portMAX_DELAY);
    uint32_t len = 0;
    esp_err_t er = encode_locked(&len), sr = ESP_OK;
    if (er == ESP_OK) {
        httpd_resp_set_type(req, "image/jpeg");
        httpd_resp_set_hdr(req, "Cache-Control", "no-store");
        sr = httpd_resp_send(req, (const char *)s_jout, len);
    }
    xSemaphoreGive(s_jpeg_mtx);
    if (er != ESP_OK) {
        httpd_resp_set_status(req, "503 Service Unavailable");
        httpd_resp_set_type(req, "text/plain");
        httpd_resp_sendstr(req, "no frame yet");
        return ESP_OK;
    }
    return sr;
}

#define MJPEG_BOUNDARY "fr"
static esp_err_t stream_get(httpd_req_t *req)
{
    if (!s_jpeg) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no encoder"); return ESP_FAIL; }
    httpd_resp_set_type(req, "multipart/x-mixed-replace;boundary=" MJPEG_BOUNDARY);
    s_stream_clients++;        /* tell the camera task to render previews */
    char hdr[96];
    while (1) {
        xSemaphoreTake(s_jpeg_mtx, portMAX_DELAY);
        uint32_t len = 0;
        esp_err_t er = encode_locked(&len), sr = ESP_OK;
        if (er == ESP_OK) {
            int hn = snprintf(hdr, sizeof hdr,
                "\r\n--" MJPEG_BOUNDARY "\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n",
                (unsigned)len);
            sr = httpd_resp_send_chunk(req, hdr, hn);
            if (sr == ESP_OK) sr = httpd_resp_send_chunk(req, (const char *)s_jout, len);
        }
        xSemaphoreGive(s_jpeg_mtx);
        if (er != ESP_OK) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }  /* wait for first frame */
        if (sr != ESP_OK) break;                                        /* client gone */
        vTaskDelay(pdMS_TO_TICKS(300));   /* ~3 fps cap; the source is ~1 fps anyway */
    }
    if (s_stream_clients > 0) s_stream_clients--;
    httpd_resp_send_chunk(req, NULL, 0);  /* terminate */
    return ESP_OK;
}

/* ---- device log ring, served at /api/log (P4) -------------------------- */
/* A global esp_log vprintf hook tees every log line into a wrap-around ring so
 * the web UI can show recent device logs without a USB cable — the whole point
 * of working remotely. Installed only when the ground server starts, so the
 * flight build keeps the stock logger. Writer appends under a short spinlock;
 * the reader is lock-free (a torn line in the debug view is harmless). */
#define LOG_RING_SZ 4096
static char            s_logbuf[LOG_RING_SZ];
static volatile size_t s_loghead;
static volatile bool   s_logwrap;
static portMUX_TYPE    s_logmux = portMUX_INITIALIZER_UNLOCKED;
static vprintf_like_t  s_log_orig;

static int log_vprintf(const char *fmt, va_list ap)
{
    char tmp[200];
    va_list cp; va_copy(cp, ap);
    int n = vsnprintf(tmp, sizeof tmp, fmt, cp);
    va_end(cp);
    if (n > (int)sizeof tmp - 1) n = (int)sizeof tmp - 1;
    if (n > 0) {
        portENTER_CRITICAL(&s_logmux);
        for (int i = 0; i < n; ++i) {
            s_logbuf[s_loghead++] = tmp[i];
            if (s_loghead >= LOG_RING_SZ) { s_loghead = 0; s_logwrap = true; }
        }
        portEXIT_CRITICAL(&s_logmux);
    }
    return s_log_orig ? s_log_orig(fmt, ap) : 0;   /* keep console output too */
}

static esp_err_t log_get(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/plain");
    size_t head = s_loghead; bool wrap = s_logwrap;   /* snapshot; lock-free read */
    if (wrap)      httpd_resp_send_chunk(req, s_logbuf + head, LOG_RING_SZ - head);
    if (head > 0)  httpd_resp_send_chunk(req, s_logbuf, head);
    httpd_resp_send_chunk(req, NULL, 0);              /* terminate */
    return ESP_OK;
}

/* ---- OTA firmware update over WiFi (P4) -------------------------------- */
static void ota_reboot_task(void *a)
{
    (void)a;
    vTaskDelay(pdMS_TO_TICKS(800));   /* let the HTTP response flush first */
    esp_restart();
}

/* POST the raw .bin as the request body. Writes it to the inactive OTA slot,
 * sets it bootable, and reboots; the existing burn-in confirm/rollback in
 * app_run() guards against a bad image. */
static esp_err_t ota_post(httpd_req_t *req)
{
    const esp_partition_t *part = esp_ota_get_next_update_partition(NULL);
    if (!part) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "no OTA slot"); return ESP_FAIL; }
    ESP_LOGI(TAG, "OTA: receiving %d bytes -> partition '%s'", req->content_len, part->label);

    esp_ota_handle_t h = 0;
    esp_err_t r = esp_ota_begin(part, OTA_SIZE_UNKNOWN, &h);
    if (r != ESP_OK) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(r)); return ESP_FAIL; }

    char *buf = malloc(4096);
    if (!buf) { esp_ota_abort(h); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom"); return ESP_FAIL; }
    int remaining = req->content_len, total = 0;
    bool ok = (remaining > 0);
    while (remaining > 0) {
        int recv = httpd_req_recv(req, buf, remaining < 4096 ? remaining : 4096);
        if (recv == HTTPD_SOCK_ERR_TIMEOUT) continue;
        if (recv <= 0)                             { ok = false; break; }
        if (esp_ota_write(h, buf, recv) != ESP_OK) { ok = false; break; }
        remaining -= recv; total += recv;
    }
    free(buf);

    if (!ok) { esp_ota_abort(h); httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "recv/write failed"); return ESP_FAIL; }
    r = esp_ota_end(h);
    if (r != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
            r == ESP_ERR_OTA_VALIDATE_FAILED ? "image validation failed" : esp_err_to_name(r));
        return ESP_FAIL;
    }
    r = esp_ota_set_boot_partition(part);
    if (r != ESP_OK) { httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, esp_err_to_name(r)); return ESP_FAIL; }

    ESP_LOGI(TAG, "OTA: %d bytes OK; boot set to '%s'; rebooting", total, part->label);
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_sendstr(req, "OK - rebooting into the new image");
    xTaskCreate(ota_reboot_task, "ota_reboot", 3072, NULL, 6, NULL);
    return ESP_OK;
}

esp_err_t ground_http_start(void)
{
    if (s_server) return ESP_OK;                 /* already running */
    if (!s_mtx) { s_mtx = xSemaphoreCreateMutex(); if (!s_mtx) return ESP_ERR_NO_MEM; }

    /* Tee device logs into the ring for /api/log (install once). */
    if (!s_log_orig) s_log_orig = esp_log_set_vprintf(log_vprintf);

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
    httpd_uri_t u_snap = { .uri = "/snapshot.jpg", .method = HTTP_GET,  .handler = snapshot_get };
    httpd_uri_t u_strm = { .uri = "/stream",       .method = HTTP_GET,  .handler = stream_get };
    httpd_uri_t u_ota  = { .uri = "/api/ota",      .method = HTTP_POST, .handler = ota_post };
    httpd_uri_t u_log  = { .uri = "/api/log",      .method = HTTP_GET,  .handler = log_get  };
    httpd_register_uri_handler(s_server, &u_root);
    httpd_register_uri_handler(s_server, &u_tlm);
    httpd_register_uri_handler(s_server, &u_cmd);
    httpd_register_uri_handler(s_server, &u_snap);
    httpd_register_uri_handler(s_server, &u_strm);
    httpd_register_uri_handler(s_server, &u_ota);
    httpd_register_uri_handler(s_server, &u_log);

    /* Bring up the hardware JPEG encoder for the live preview. Non-fatal: if it
     * fails the server still serves telemetry + control, just no image. */
    if (jpeg_init() != ESP_OK)
        ESP_LOGW(TAG, "JPEG encoder init failed — live preview disabled");

    char ip[16] = "0.0.0.0"; ground_wifi_get_ip(ip, sizeof ip);
    ESP_LOGI(TAG, "web control server up at http://%s/", ip);
    return ESP_OK;
}

#endif /* CONFIG_GROUND_WIFI_ENABLE */
