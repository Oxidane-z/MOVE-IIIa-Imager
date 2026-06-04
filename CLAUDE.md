# CLAUDE.md

This file is read at the start of every Claude session in this repo.
Keep it short and concrete. Long-form context goes in `PROJECT_MEMORY.md`.

## What this repo is

A CubeSat imaging payload firmware for ESP32-P4, plus all the R&D scratch
that went into it. **Two camera targets** share the same firmware:

- **SC850SL flight target** (`CONFIG_CAMERA_TARGET_SC850SL=y`).
  Hand-rolled driver in `firmware/components/sc850sl/` with M5Stack's
  production 2-lane 4K15 register init table. Runs on a Stamp-P4 +
  custom carrier (or Olimex/Waveshare P4 dev board). Phase 1 milestone
  done — chip-ID probe passes; full bring-up is future work.

- **SC202CS bench target on M5Stack Tab5**
  (`CONFIG_CAMERA_TARGET_SC202CS_TAB5=y`, **the default**, pinned in
  `sdkconfig.defaults`). Uses upstream `esp_video` + `esp_cam_sensor` +
  Tab5 BSP. (Early M5Stack docs and the M5Tab5-UserDemo sample
  mislabel this sensor as SC2336; the silicon PID readback is `0xeb52`
  which is SC202CS, and that's the esp_cam_sensor driver we enable.)
  Live demo: SC202CS → 640×480 landscape preview (rotated CW90° via
  PPA hardware) → Capture+Send buttons → Robot36 SSTV TX out the
  onboard speaker, **plus** mic-based Robot36 RX decoder with a color
  image building up from the bottom of the screen.

Robot36 SSTV encoder and decoder are both written in-tree:
- TX: `firmware/components/sstv_robot36/`
- RX: inlined in `firmware/main/app_sc202cs_tab5.c`

The MI1602 thermal camera driver lives in a sibling folder
(`../MI1602 Dev/components/mi1602/`) and is pulled in via
`EXTRA_COMPONENT_DIRS` from the firmware's top-level `CMakeLists.txt`.

## Where to look first

| Question                                  | File                                                |
|-------------------------------------------|-----------------------------------------------------|
| **Full project context / history**        | `PROJECT_MEMORY.md` (esp. §5.8 + §8.5)              |
| **Where the Tab5 demo lives**             | `firmware/main/app_sc202cs_tab5.c`                  |
| **Where the SSTV encoder lives**          | `firmware/components/sstv_robot36/sstv_robot36.c`   |
| **Build defaults**                        | `firmware/sdkconfig.defaults`                       |
| **Per-target Kconfig**                    | `firmware/main/Kconfig.projbuild`                   |
| **Component deps + versions**             | `firmware/main/idf_component.yml`                   |
| **How to build/flash (human-readable)**   | `firmware/HOW_TO_BUILD.md`                          |
| **MI1602 integration plan**               | `firmware/MI1602_INTEGRATION.md`                    |
| **Why various things are patched**        | `PROJECT_MEMORY.md` §5.8 (6 patches) + §8.5 (more) |

## Build / flash (Windows, this machine)

The IDF env on this machine is at `C:\esp\esp-idf-v6.0.1`. The Tab5 is on
**COM15** as of last run. ESP-IDF v6.0.1 + RISC-V toolchain are installed.

**MSYS bash gotcha**: `export.ps1` is blocked by PS execution policy,
`export.bat` refuses to run when `MSYSTEM` is set (which leaks into
every `cmd /c` child spawned from MSYS bash). So all build/flash use
wrapper `.bat` files that `set "MSYSTEM="` before sourcing.

```sh
# From the firmware/ directory in MSYS bash:
cmd //c "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware\_build.bat"
cmd //c "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware\_flash.bat"
cmd //c "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\firmware\_reset_capture.bat"
```

Note the **double slash** in `cmd //c` — MSYS bash mangles paths in
single-slash `cmd /c` args. Wrappers always exit with `=== BUILD_OK ===`
or `BUILD_FAILED`; check that line, don't trust the shell's `$?` because
pipes (`| tail …`) mask compile failures.

When the bin needs reflashing after the COM port re-enumerates post-flash,
list ports with `Get-CimInstance -ClassName Win32_PnPEntity | Where-Object
{ $_.Name -match 'COM\d+' }` and update `COM15` inside `_flash.bat` if
needed.

## Build/flash output capture

`_build.bat`, `_flash.bat`, `_reset_capture.bat` all redirect to the
project root. The harness captures their stdout. Key markers to grep for:

- `=== BUILD_OK ===` — successful build
- `=== FLASH_OK ===` — successful flash
- `error:` / `FAILED:` — anything bad in build
- `task_wdt` — watchdog fired (was a recurring debug target — see §8.5)
- `mic raw=` / `fft=` — periodic decoder diagnostics
- `app/tab5:` — Tab5 demo log tag

## Common pitfalls (learned the hard way — see PROJECT_MEMORY §8.5)

1. **Delete `sdkconfig` carefully**. Kconfig choices ARE persisted there.
   If you `rm sdkconfig` to re-derive from defaults, double-check that
   `CONFIG_CAMERA_TARGET_SC202CS_TAB5=y` is still pinned (it is, in
   `sdkconfig.defaults`, as of phase 2).

2. **Don't call `lv_obj_*` from a non-LVGL thread without the lock**.
   Either wrap calls in `bsp_display_lock(timeout)` / `bsp_display_unlock`,
   or queue invalidates into the existing `preview_tick` `lv_timer`.
   `mic_task` learned this the hard way (deadlocked IDLE1, tripped
   the task watchdog).

3. **`vTaskDelay(1)` is not 1 ms**. With FREERTOS_HZ=1000, `vTaskDelay(1)`
   can be anywhere from 0 to 1 ms because the tick alignment can collapse
   it. Use `vTaskDelay(pdMS_TO_TICKS(20))` or more when you need IDLE
   to actually run for long enough to feed the watchdog.

4. **PPA's `rotation_angle` is counter-clockwise**. To rotate the camera
   CW 90° (which is what makes the scene right-side-up on Tab5 in
   portrait), use `PPA_SRM_ROTATION_ANGLE_270`.

5. **`esp_codec_dev_read()` doesn't always block** until the full request
   is filled — it can return early with what DMA currently has. Don't
   rely on the read call to pace the mic loop; add an explicit
   `vTaskDelay`.

6. **`heap_caps_aligned_calloc(64, ...)`** for any PPA output buffer in
   PSRAM (L2 cache line on ESP32-P4 is 64 bytes).

## Style notes

- Comments explain *why*, not *what*. Especially around any patched
  code path, leave a paragraph saying what the original behavior was
  and why we changed it.
- No emojis in code, no emojis in user-facing logs (esp. log lines that
  the Windows COM-port viewer will mangle — `capture_serial.py` already
  forces UTF-8 stdout but be sparing anyway).
- Stick to ASCII identifiers + Latin-1 for comments. UTF-8 is fine in
  doc strings (`PROJECT_MEMORY.md` etc.) but compiler diagnostics get
  ugly with multi-byte chars in source.
- When you edit `sdkconfig.defaults`, also edit the comment above the
  changed config explaining the rationale — that comment IS the
  permanent documentation.

## What NOT to commit (`.gitignore` handles these)

- `firmware/build/` (~400 MB)
- `firmware/managed_components/` (~225 MB, re-fetched on first build)
- `firmware/sdkconfig` (re-derived from `sdkconfig.defaults`)
- `firmware/dependencies.lock`
- Datasheets (`*.pdf`, `datasheet_utf8.txt`, `mi1602.txt`)
- Extraction leftovers (`*.deb`, `rootfs_files/`, `m5_research/`)
- `.claude/` session data

## How to communicate status back to the user

The user is technically fluent (Chinese-native, hardware EE background).
- Lead with the conclusion + the next concrete test step
- Show actual log evidence when claiming something works
- For visual demo features, hand back to the user with a clear list of
  what to look at on the Tab5 screen
- When something doesn't work, propose 1–2 specific fixes, not 5
- Build + flash cycles are slow (~3–5 min build, ~5 s flash, ~15 s
  serial capture). Plan changes accordingly — batch as much as makes
  sense without making debugging harder if it fails.
