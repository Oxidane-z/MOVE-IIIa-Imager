# `sc850sl` — SmartSens SC850SL camera driver

ESP-IDF v6.0+ component for the 8 MP SmartSens SC850SL CMOS image sensor.

Used in the M5Stack LLM630 Compute Kit (Axera AX630C). This is a clean
re-implementation for ESP32-P4, sharing the same register init tables that
M5Stack ships in their `libsns_sc850sl.so`.

## Capabilities

- Power-on sequencing (rails, XSHUTDN, EXTCLK via LEDC)
- I²C control bus (reg16/data8, ESP-IDF v5.4 `i2c_master` API)
- Chip ID probe (expect `0x9d1e`)
- Init-table loader with 4 modes:
  - **Table 2** ⭐ `4K15 RAW10 2-lane` — CubeSat target
  - Table 0: 4K30 RAW10 4-lane (LLM630 default)
  - Table 1: 4K30 HDR-DOL 4-lane
  - Table 3: 4K15 RAW10 2-lane (alt HTS timing)
- Output window / mirror / flip / test pattern
- Stream on/off, deep sleep, wake

## What this component does NOT do

- CSI host configuration → see `components/isp_pipeline/`
- AE/AGC closed-loop control → planned for `components/isp_pipeline/`
  using the ISP's stats output
- Frame buffer management → ISP component
- HDR de-multiplexing → ISP component

Keep this component *sensor-only*. The boundary makes it easy to bring the
sensor up on a different SoC later if needed.

## Configuration (Kconfig)

Open `idf.py menuconfig` → `Component config` → `SC850SL Camera`. Default
GPIO pins:

| Pin | Default | Notes |
|---|---|---|
| `SDA`     | GPIO 8  | Pull up to **1.8 V**, not 3.3 V |
| `SCL`     | GPIO 9  | Pull up to **1.8 V** |
| `XSHUTDN` | GPIO 10 | Active low, level-shift to 1.8 V if your DOVDD is 1.8 V |
| `EXTCLK`  | GPIO 11 | LEDC 24 MHz, 50 % duty |
| `LED`     | GPIO 14 | Heartbeat |

Change these to match your carrier PCB. **Pull-ups must reference DOVDD
(1.8 V)** for I²C, not the P4's 3.3 V, because the sensor I/O is 1.8 V.

## Example use (Phase 1: probe only)

```c
i2c_master_bus_handle_t i2c;
i2c_master_bus_config_t bus = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_NUM_0,
    .sda_io_num = CONFIG_SC850SL_I2C_SDA_GPIO,
    .scl_io_num = CONFIG_SC850SL_I2C_SCL_GPIO,
};
i2c_new_master_bus(&bus, &i2c);

sc850sl_config_t cfg = {
    .i2c_bus      = i2c,
    .i2c_addr     = SC850SL_I2C_ADDR_DEFAULT,
    .i2c_freq_hz  = CONFIG_SC850SL_I2C_FREQ_HZ,
    .xshutdn_gpio = CONFIG_SC850SL_XSHUTDN_GPIO,
    .extclk_gpio  = CONFIG_SC850SL_EXTCLK_GPIO,
    .extclk_hz    = CONFIG_SC850SL_EXTCLK_HZ,
    .rail_en_dovdd = -1,  // always-on rails
    .rail_en_dvdd  = -1,
    .rail_en_avdd  = -1,
    .mode = SC850SL_MODE_4K15_2LANE_RAW10,
};

sc850sl_handle_t cam;
ESP_ERROR_CHECK(sc850sl_init(&cfg, &cam));

uint16_t id;
ESP_ERROR_CHECK(sc850sl_probe(cam, &id));
// id == 0x9d1e
```

## File layout

```
sc850sl/
├── CMakeLists.txt                  # component registration
├── Kconfig.projbuild               # menuconfig knobs
├── README.md                       # this file
├── include/
│   └── sc850sl.h                   # public API
├── private/
│   └── sc850sl_regs.h              # register defines (driver-internal)
├── sc850sl.c                       # implementation
└── init_tables/
    ├── sc850sl_table_0.c           # 4K30 4-lane (M5Stack default)
    ├── sc850sl_table_1.c           # 4K30 HDR-DOL 4-lane
    ├── sc850sl_table_2.c           # ★ 4K15 2-lane (CubeSat target)
    └── sc850sl_table_3.c           # 4K15 2-lane alt HTS
```

The init tables are generated from M5Stack's BSP — see the project root's
`PROJECT_MEMORY.md` section 3 for the analysis trail and the
`dump_init_tables.py` script that produces them.
