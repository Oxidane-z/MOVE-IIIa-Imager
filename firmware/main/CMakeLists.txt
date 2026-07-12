# Build the per-target source file based on the CAMERA_TARGET_* Kconfig
# choice. Each app_*.c also self-guards with #ifdef, so even if both
# were listed unconditionally the wrong one's body would be empty —
# but actually NOT listing it means the compiler doesn't have to even
# open the file, which shaves a few seconds off a clean build.
set(srcs "main.c" "mi1602_probe.c" "ground_wifi.c" "ground_http.c")
if(CONFIG_CAMERA_TARGET_SC850SL)
    list(APPEND srcs "app_sc850sl.c")
elseif(CONFIG_CAMERA_TARGET_SC202CS_TAB5)
    list(APPEND srcs "app_sc202cs_tab5.c")
elseif(CONFIG_CAMERA_TARGET_MI1602_TEST)
    list(APPEND srcs "app_mi1602_test.c")
endif()

# Components we always need.
#
# NOTE: mi1602 is listed UNCONDITIONALLY. We can't gate it on
# CONFIG_MI1602_ENABLED here — component REQUIRES are resolved in an
# early-expansion CMake pass that runs BEFORE Kconfig is loaded, so any
# `if(CONFIG_*)` around a REQUIRES entry is always false there and the
# dependency silently drops (build then fails: "mi1602.h ... mi1602
# component is not in the requirements list"). mi1602_probe.c still
# self-guards with #if CONFIG_MI1602_ENABLED, so when disabled the
# component links no active code — only a small compile cost remains.
set(requires
    sc850sl
    mi1602            # MI1602 thermal camera (sibling-folder component)
    sstv_robot36      # Robot36 SSTV encoder
    esp_driver_gpio
    esp_driver_i2c
    esp_driver_ledc
    esp_driver_i2s
    esp_timer
    app_update        # esp_ota_ops: OTA write + anti-brick rollback
    esp_wifi          # esp_wifi.h for ground_wifi.c. esp_wifi_remote PRIV_REQUIREs
                      # esp_wifi (so it isn't transitive) and REQUIRES can't be
                      # Kconfig-gated, so list it unconditionally. On P4 without
                      # esp_wifi_remote it builds as weak/empty stubs, and
                      # ground_wifi.c is #if'd out, so flight links no WiFi code.
    esp_http_server   # web control server (ground_http.c). IDF built-in, so it
                      # can't be gated via idf_component.yml; REQUIRES can't be
                      # Kconfig-gated either, so list it unconditionally.
                      # ground_http.c is #if'd out in flight, so its code (and
                      # lwIP's) dead-strips away there.
    esp_driver_jpeg   # hardware JPEG encoder for the live-preview MJPEG stream
                      # (ground_http.c). Same unconditional-require rationale.
    nvs_flash         # NVS init for the (ground-only) WiFi stack; inert in flight.
)

# Tab5 target needs the BSP + LVGL + audio codec stack.
if(CONFIG_CAMERA_TARGET_SC202CS_TAB5)
    list(APPEND requires
        espressif__m5stack_tab5
        espressif__esp_lvgl_port
        espressif__esp_codec_dev
        lvgl__lvgl
    )
endif()

# NOTE on the ground-WiFi deps: esp_wifi + nvs_flash are in the unconditional
# `requires` above (REQUIRES cannot be Kconfig-gated — they expand before
# Kconfig loads). esp_netif/esp_event arrive transitively from esp_wifi_remote's
# public REQUIRES when GROUND_WIFI_ENABLE is on; esp_wifi_remote + esp_hosted
# themselves are pulled (gated on that symbol) via idf_component.yml. The flight
# build fetches none of that and ground_wifi.c self-#if-guards to empty.

# Always-needed private requires. SC850SL needs CSI + ISP + LDO for the
# MIPI host; both targets touch esp_cache_msync etc. through esp_hw_support.
set(priv_requires
    esp_driver_cam               # MIPI CSI receiver (SC850SL path)
    esp_driver_isp               # On-chip ISP: debayer, AWB, AE, CCM, gamma
    esp_hw_support               # esp_ldo_acquire_channel (MIPI CSI PHY rail)
    esp_driver_usb_serial_jtag   # usb_serial_jtag_vfs_set_tx_line_endings (binary-clean USB stream)
)

# Tab5-only private requires:
#   - esp_driver_ppa: hardware rotation of the live preview (~1 ms/frame
#     vs ~85 ms CPU). The SC850SL flight target never rotates.
#   - espressif__esp-dsp: FFT for the SSTV-RX waterfall (Phase 2A).
# Gating these means a clean SC850SL build skips ~1-2 minutes of
# compiling code we'll never link.
if(CONFIG_CAMERA_TARGET_SC202CS_TAB5)
    list(APPEND priv_requires
        esp_driver_ppa
        espressif__esp-dsp
    )
endif()

idf_component_register(
    SRCS ${srcs}
    INCLUDE_DIRS "."
    REQUIRES ${requires}
    PRIV_REQUIRES ${priv_requires}
    # The ground-station control page is embedded unconditionally (~6 KB); only
    # ground_http.c (gated) references it, so it's dead weight in flight but
    # avoids conditionalizing the register() call. Symbols:
    #   _binary_ground_index_html_start / _binary_ground_index_html_end
    EMBED_FILES "ground_index.html"
)
