/*
 * ground_station.h -- interface between the camera app (app_sc850sl.c) and the
 * ground-test web server (ground_http.c). GROUND-ONLY: every call site in the
 * app is wrapped in #if CONFIG_GROUND_WIFI_ENABLE, and the implementations live
 * in ground_http.c (also fully gated), so the flight build references none of
 * this. See FLIGHT_ARCHITECTURE.md s13.
 *
 * Data flow is snapshot-based to avoid the web (httpd) task racing the camera
 * task on live buffers: the app publishes a telemetry struct (and, for the
 * live preview, a copy of the latest RGB565 frame) under an internal mutex;
 * the server reads consistent snapshots. Commands go the other way through a
 * single-slot mailbox the app drains once per frame.
 */
#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* Telemetry the camera app publishes once per rendered frame. */
typedef struct {
    uint32_t seq;             /* frame counter                              */
    uint16_t w, h;            /* preview composite dimensions               */
    int      raw_min, raw_max, raw_mean, raw_sat_ppm;
    uint32_t exp_lines;       /* current sensor exposure (lines)            */
    uint32_t gain_x1024;      /* current total gain, 1024 = 1.0x            */
    int      black_level;
    float    wb_r, wb_b;      /* gray-world AWB gains                       */
    int      ae_target;       /* AE target raw mean                         */
    bool     ae_enabled;      /* AE loop active vs manual exposure          */
    int64_t  t_cap_us, t_isp_us, t_usb_us;  /* per-stage frame timings      */
    bool     cam_streaming;
    bool     thermal_ok;
} ground_tlm_t;

/* Command mailbox: server sets fields, app applies + clears once per frame.
 * Integer fields use <0 to mean "no change"; bools are one-shot triggers. */
typedef struct {
    int  ae_target;     /* >=0: set AE target raw mean        */
    int  ae_enabled;    /* 0/1: disable/enable AE; <0 no change */
    int  exp_lines;     /* >=0: set exposure (lines)          */
    int  gain_x1024;    /* >=0: set total gain (1024=1x)      */
    int  usb_push;      /* 0/1: disable/enable legacy USB push; <0 no change */
    bool sstv_trigger;  /* one-shot: kick an SSTV TX          */
    bool capture_hd;    /* one-shot: latch a full-res HD frame */
} ground_cmd_t;

/* ---- app -> server (called from the camera/usb_stream task) ---- */
void   ground_publish_tlm(const ground_tlm_t *tlm);
/* Copy the latest RGB565 preview for the server (w*h*2 bytes). Cheap; only the
 * most recent frame is retained. */
void   ground_publish_preview(const void *rgb565, size_t bytes, uint16_t w, uint16_t h);

/* ---- server -> app ---- */
/* Drain a pending command (returns true and fills *out if one was queued). */
bool   ground_cmd_take(ground_cmd_t *out);

/* ---- server-side accessors (called from the httpd task) ---- */
bool   ground_tlm_snapshot(ground_tlm_t *out);
size_t ground_preview_copy(void *dst, size_t maxbytes, uint16_t *w, uint16_t *h);
void   ground_cmd_post(const ground_cmd_t *cmd);
/* Active /stream (MJPEG) client count; the camera task uses it to skip the
 * software-ISP render when nobody is watching. */
int    ground_preview_clients(void);

/* Start the web server. Call once the station has an IP; idempotent. */
esp_err_t ground_http_start(void);

#ifdef __cplusplus
}
#endif
