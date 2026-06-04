# Placeholder — fill in during Phase 2 (CSI host + ISP wiring).
# An empty component still has to register so the build accepts it.
idf_component_register(
    SRCS
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_isp esp_driver_jpeg esp_driver_cam
)
