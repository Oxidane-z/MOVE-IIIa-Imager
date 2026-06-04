# Integrating the MI1602 driver

The MI1602 thermal-camera driver is developed as a **separate project** at
`C:\Users\zeyu.zhu\Pictures\MI1602 Dev\`. The top-level CMakeLists here
automatically picks it up via `EXTRA_COMPONENT_DIRS` the moment that path
exists. Nothing else to do — no symlinks, no copies, no submodules.

## Expected layout in the sibling folder

```
C:\Users\zeyu.zhu\Pictures\MI1602 Dev\
├── README.md
├── components/
│   └── mi1602/
│       ├── CMakeLists.txt
│       ├── Kconfig.projbuild           # SPI pin/freq, RSTN pin, SYSCLK pin
│       ├── README.md
│       ├── include/
│       │   └── mi1602.h                # public API
│       ├── private/
│       │   └── mi1602_regs.h
│       ├── mi1602.c                    # core driver
│       └── palette/
│           └── iron.c                  # optional pseudo-color LUTs
└── test/                               # optional standalone test app
    ├── CMakeLists.txt
    ├── sdkconfig.defaults
    └── main/
        └── main.c                      # captures one frame, dumps over UART
```

The `components/mi1602/` folder structure should mirror
`firmware/components/sc850sl/` for consistency. The `test/` folder is
an entire mini ESP-IDF project that drives only the MI1602 — useful to
develop in isolation before wiring it into the main firmware.

## How the SC850SL firmware finds it

`firmware/CMakeLists.txt` contains:

```cmake
set(MI1602_COMPONENT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../MI1602 Dev/components")
if(EXISTS "${MI1602_COMPONENT_DIR}/mi1602")
    list(APPEND EXTRA_COMPONENT_DIRS "${MI1602_COMPONENT_DIR}")
endif()
```

When you run `idf.py build` from `firmware/`, CMake will:

1. Detect `../../MI1602 Dev/components/mi1602/`
2. Add it as a known component directory
3. Any component (e.g. `main`) that `REQUIRES mi1602` will link against it

To wire the MI1602 into Phase 4 (SSTV), edit `firmware/main/CMakeLists.txt`:

```cmake
idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        sc850sl
        mi1602        # ← add this
        esp_driver_gpio
        esp_driver_i2c
)
```

…and call `mi1602_init(...)` from `app_main`. CMake won't complain even if
the MI1602 folder is missing today: the `if(EXISTS ...)` gate makes it
optional during early phases.

## Recommended MI1602 driver API shape (suggestion only)

Mirror the SC850SL component for symmetry:

```c
typedef struct {
    spi_host_device_t  spi_host;
    gpio_num_t         cs_n_gpio;
    gpio_num_t         rst_n_gpio;
    gpio_num_t         sysclk_gpio;     // LEDC 3 MHz
    gpio_num_t         data_avail_gpio; // optional irq input
    uint32_t           sysclk_hz;       // 3000000
    uint32_t           spi_freq_hz;     // up to 24 MHz
} mi1602_config_t;

typedef struct mi1602_t *mi1602_handle_t;

esp_err_t mi1602_init(const mi1602_config_t *cfg, mi1602_handle_t *out);
esp_err_t mi1602_capture_frame(mi1602_handle_t h, uint16_t *out_160x120);
esp_err_t mi1602_get_temperature_range(mi1602_handle_t h, int16_t *min_mK, int16_t *max_mK);
void      mi1602_deinit(mi1602_handle_t h);
```

The thermal-frame format is 160×120 radiometric (16-bit per pixel = 38.4 KB).
SSTV / overlay code in `components/sstv_robot36/` consumes it.

## Verifying integration

After the MI1602 driver lands:

```powershell
cd "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware"
idf.py reconfigure
idf.py build
```

`reconfigure` makes CMake re-scan `EXTRA_COMPONENT_DIRS`. Look for this
line in the configure output:

```
-- MI1602 driver found at C:/Users/zeyu.zhu/Pictures/MI1602 Dev/components/mi1602
```

If you see "*not present*" instead, the path is wrong — either the folder
isn't where the parent CMakeLists expects, or the inner `components/mi1602/`
directory doesn't exist yet.
