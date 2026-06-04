# How to build / flash / monitor

A walk-through from "no ESP-IDF installed" to "watching serial logs from
the Stamp-P4". Targeted at Windows 10/11.

---

## 1. Install ESP-IDF v6.0.1 (one-time)

### Step A — download the offline installer

1. Go to <https://dl.espressif.com/dl/esp-idf/> and download
   **"ESP-IDF Tools Installer - Online"** (Windows). When the installer
   asks which IDF version to fetch, pick **v6.0.1**.
2. Run it. When asked where to install ESP-IDF, use the default
   `C:\Espressif\frameworks\esp-idf-v6.0.1` (the rest of this doc assumes
   that path).
3. When it asks which targets to install toolchains for, leave them all
   selected (or at minimum check **ESP32-P4**, which is `riscv32-esp-elf`).
4. Let it finish. It will:
   - Download ESP-IDF source to `C:\Espressif\frameworks\esp-idf-v6.0.1\`
   - Install the RISC-V toolchain
   - Set up a Python virtual env at `C:\Espressif\python_env\idf6.0_py3.11_env\`
     (this is separate from your system Python 3.13 — leave that alone)
   - Add a **Start Menu shortcut: "ESP-IDF 6.0 PowerShell"**
   - Install Espressif USB-Serial-JTAG drivers

### Step B — verify the CLI works

Open the **"ESP-IDF 6.0 PowerShell"** shortcut from the Start menu. You
should see something like:

```
Setting IDF_PATH: C:\Espressif\frameworks\esp-idf-v6.0.1
Setting ESP-IDF tools: ...
Done! You can now compile ESP-IDF projects.
Go to the project directory and run: idf.py build
```

Then verify the version and that P4 is a known target:

```powershell
idf.py --version          # should print "v6.0.1" (or v6.0 if you didn't update)
idf.py --list-targets
```

The second command must include `esp32p4` somewhere in the output.

### Step C — install the VS Code extension (optional but recommended)

1. In VS Code, open Extensions (`Ctrl+Shift+X`), search for
   **"ESP-IDF"** by Espressif Systems, install it.
2. Open the Command Palette (`Ctrl+Shift+P`) → run
   **"ESP-IDF: Configure ESP-IDF Extension"**.
3. Pick **"Use Existing Setup"**.
4. Point it at `C:\Espressif\frameworks\esp-idf-v6.0.1`. It will
   auto-discover the toolchain, Python env, and tools.
5. Done. The extension now shares the exact same backend as the CLI;
   anything you build in one will be visible to the other.

> **Note on v6.0.1 vs v5.4** — v6.0.1 is the latest stable
> (April 2026 bugfix release on top of v6.0). It introduces a few
> breaking changes vs v5.x (legacy ADC/I2S/RMT/etc. drivers removed,
> I²C NACK now returns `ESP_ERR_INVALID_RESPONSE`, ESP32-P4 default
> silicon rev bumped to v3.0). None of these affect this project —
> our driver code uses the new `i2c_master.h` API and tolerates any
> error code. Just install v6.0.1.
>
> If the installer only offers v6.0.0, the auto-updater inside the
> "ESP-IDF 6.0 PowerShell" can pull the latest tag with
> `git -C $env:IDF_PATH fetch && git -C $env:IDF_PATH checkout v6.0.1
> && git -C $env:IDF_PATH submodule update --recursive`.

---

## 2. Build this project

### From the ESP-IDF 5.4 PowerShell:

```powershell
cd "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware"
idf.py set-target esp32p4         # one-time per checkout
idf.py build
```

The first build is slow (~3–5 minutes — it compiles libc, lwIP, all of
IDF). Subsequent builds are seconds because of ccache.

Expected output ends with:
```
Project build complete. To flash, run this command:
ninja flash
or run idf.py:
idf.py flash
```

If something fails, scroll up for the first error line. Most common
errors at this stage:

- **`fatal error: 'sc850sl.h' file not found`** — wrong working directory.
  Make sure you `cd` into `firmware/`, not the project root.
- **`region 'iram0_0_seg' overflowed`** — you've added something heavy
  in `main.c`. Try `CONFIG_COMPILER_OPTIMIZATION_SIZE` if it bites.
- **Build picks up the wrong target** — delete `build/` and `sdkconfig`,
  then re-run `idf.py set-target esp32p4`.

### From VS Code

1. Open the **`firmware/`** folder in VS Code (`File → Open Folder...`).
   Open the inner `firmware/`, not the project root — the IDF extension
   needs to find `CMakeLists.txt` at the workspace root.
2. Bottom-left status bar: click the chip icon and pick **`esp32p4`**.
3. Bottom-left status bar: click the **wrench icon** (Build).
4. Wait for "Build Successful".

---

## 3. Flash & monitor

### Find the COM port

Plug the Stamp-P4 into USB. Open Device Manager → "Ports (COM & LPT)".
It enumerates as a "USB JTAG/serial debug unit" or similar — note the
COM number (e.g. `COM7`).

### From CLI

```powershell
idf.py -p COM7 flash monitor
```

This flashes and then opens a serial console. Press `Ctrl+]` to exit.

### From VS Code

1. Click the **plug icon** in the status bar to set the COM port.
2. Click the **flame icon** (Flash) — pick "UART" if asked.
3. Click the **TV icon** (Monitor) to open serial.

---

## 4. Expected output (Phase 1 milestone)

### Bare Stamp-P4 (no SC850SL carrier attached)

```
I (29) boot: ESP-IDF v6.0.1
I (30) boot: chip revision: vN.N
...
I (456) main: CubeSat imager booting (Phase 1)
I (459) main: I²C master open: SDA=8 SCL=9
I (465) sc850sl: EXTCLK 24000000 Hz on GPIO11 (LEDC ch0)
I (476) sc850sl: powered, addr=0x30, freq=100000 Hz, mode=2
E (526) sc850sl: id hi: ESP_ERR_INVALID_RESPONSE   ← expected — sensor NACKs (not on bus)
W (529) main: probe failed: ESP_ERR_INVALID_RESPONSE — running without sensor
```

(In v5.4 this was `ESP_ERR_TIMEOUT`. v6.0.1 distinguishes "bus busy" from
"address NACK" — the latter is what you see when no sensor is connected.)

The heartbeat LED on GPIO 14 (if connected) blinks at 1 Hz from here on.
**Seeing this output proves the firmware boots and the I²C peripheral is
working correctly.**

### With the SC850SL FPC connected

```
I (476) sc850sl: powered, addr=0x30, freq=100000 Hz, mode=2
I (489) sc850sl: chip ID = 0x9d1e (expected ✓)
I (490) main: probe OK — sensor is alive (id=0x9d1e)
```

That's the Phase 1 success criterion.

---

## 5. Common gotchas

| Symptom | Likely cause |
|---|---|
| `Error opening COM port: Permission denied` | Another monitor is open, or VS Code holds the port. Close them. |
| Build fails with `python: command not found` | You opened a regular PowerShell, not the ESP-IDF one. Use the Start-Menu shortcut. |
| Reset loops on flash | Wrong flash size in `sdkconfig` — verify `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y`. |
| esptool says **chip revision too old** | v6.0.1 defaults to P4 rev v3.0. Set `CONFIG_ESP32P4_SELECTS_REV_LESS_V3=y` in `sdkconfig.defaults` and re-flash. |
| `i2c_master_transmit ... ESP_ERR_INVALID_RESPONSE` | NACK — sensor not on bus, wrong address, or pull-ups not on 1.8 V DOVDD. |
| `i2c_master_transmit ... ESP_ERR_TIMEOUT` | Bus stuck — SDA or SCL pulled to GND, or hardware fault. |
| Heartbeat blinking but `probe failed` | Sensor unpowered, wrong I²C pins, or no carrier. |

---

## 6. Switching build target (SC850SL ↔ Tab5)

The firmware has two camera-target build variants, selected at compile
time via Kconfig:

| Kconfig | Means | Used on |
|---|---|---|
| `CAMERA_TARGET_SC850SL` | Hand-rolled SC850SL driver | Stamp-P4 flight carrier, Olimex/Waveshare + 30-pin adapter |
| `CAMERA_TARGET_SC202CS_TAB5` (**default**) | Tab5 SC202CS via `esp_cam_sensor` | M5Stack Tab5 |

Switch via `idf.py menuconfig`:

```
CubeSat Imager — Camera target  → Visible camera target  → [pick one]
```

Or directly in `sdkconfig.defaults` for a permanent project default
(currently pinned to the Tab5 target).

### Tab5 path is live

The Tab5 SC202CS path is fully working as of Phase 2 — see
`PROJECT_MEMORY.md` §8.5 for the patch list (IPA pipeline controller,
PPA hardware rotation, `-Werror=discarded-qualifiers` workaround, etc.)
that took `esp_video` + `esp_cam_sensor` from "fails to compile on
v6.0.1" to "full LVGL preview + SSTV TX/RX demo".

## 7. Next steps after Phase 1 passes

1. Bench-confirm EXTCLK 24 MHz on a scope at the GPIO pin.
2. Once the SC850SL probes OK, uncomment the `sc850sl_load_init_table` and
   `sc850sl_stream_on` lines in `main.c` and verify on the scope that the
   MIPI CSI-2 differential pair starts toggling.
3. Move to Phase 2: fill in `components/isp_pipeline/` with the
   `esp_cam_ctlr_csi` + `esp_isp` + `jpeg_encoder` integration.

For MI1602 integration once that driver exists, see
[MI1602_INTEGRATION.md](MI1602_INTEGRATION.md).
