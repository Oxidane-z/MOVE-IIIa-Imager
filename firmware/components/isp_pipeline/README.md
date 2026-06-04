# `isp_pipeline` — placeholder

Phase 2 work. Will host:

- `esp_cam_ctlr_csi` setup for 2-lane RAW10 @ 1080 Mbps/lane
- `esp_isp` setup for the 4 quadrant-tile pipeline (4K → 4 × 1080p)
- Hardware JPEG encoder wrap (full-res + Robot36 paths)
- PPA helpers for resize / CSC
- AE/AWB statistics consumer feeding the SC850SL exposure/gain regs

For now: empty `idf_component_register` so the build accepts it as a
dependency. See `firmware/main/main.c` Phase 2 stub for the integration
point.
