# MOVE-IIIa Imager — Flight Software Architecture

> **Status:** design draft (2026-06). Target: ESP32-P4 (M5Stamp-P4 + custom
> carrier). This is the blueprint for the flight firmware; sections marked
> **TBD** still need a decision or a hardware fact.

---

## 1. Scope & hardware

| Subsystem | Part | Interface | Notes |
|-----------|------|-----------|-------|
| MCU | ESP32-P4 (rev v1.3) | — | 2× HP RISC-V @360 MHz (FPU) + 1× LP RISC-V @~40 MHz; 768 KB on-chip SRAM, 32 MB PSRAM, 16 MB flash |
| Visible cam | SmartSens **SC850SL** (4K) | MIPI-CSI 2-lane RAW10, **I2C0**, XSHUTDN GPIO54 | host-driven exposure/gain (vendor recipe; no internal AEC) |
| Thermal cam | Meridian **MI1602 / MI48** (160×120) | **I2C1** control + **SPI2** data, DATA_READY GPIO8 | I²C addr 0x40/0x41 **auto-detected** (ADDR strap floats) |
| Audio/SSTV | PCM5102A DAC | **I2S** (MCLK20/BCK21/LRCK23/DIN22) | Robot36 SSTV TX out the audio chain |
| OBC link | RS422 transceiver | **UART0: TX=GPIO37, RX=GPIO38, DE=GPIO39** | the lifeline; full-duplex (RX always-on, DE gates TX); console is on USB so U0 is free |
| Board temp | Microchip **AT30TS74** | **LP-I²C (I2C2): SDA=LP_GPIO7, SCL=LP_GPIO8**, addr **0x48** | read by the LP core for health telemetry (works while HP cores sleep) |
| Ground WiFi | ESP32-C6 addon | **SDIO** (esp_wifi_remote / ESP-Hosted) | **ground test only**, not populated/active in flight |
| Storage | internal flash | dual-OTA + FAT `storage` | see §12 OTA |

---

## 2. Design principles

1. **OBC is master; the payload is a command-driven responder.** The OBC
   orchestrates; the payload executes and reports.
2. **RS422 is the lifeline** (control + telemetry + recovery + firmware). It
   gets a dedicated core and the highest task priority and is **never blocked**
   by imaging/encoding.
3. **Autonomous-beacon default (the key fault-tolerance rule).** The OBC owns a
   **hardware power switch to the P4**, so the payload only runs when the OBC
   has powered it — a *dead* OBC just means the board is never powered, which
   needs no software handling. The fault we DO handle is **"powered, but the
   RS422 link is down":** with no usable OBC link the payload **autonomously
   captures and beacons SSTV, alternating visible / thermal**, so a broken comms
   link never stops it from doing useful, ground-receivable work. (SSTV is
   decodable by any ham ground station.) The OBC's power switch is the ultimate
   override.
4. **Fault isolation + graceful degradation.** Every subsystem is its own task
   with bounded timeouts and fail-fast (no retry storms); one failing subsystem
   never takes down another. One camera down → beacon with the other.
5. **Independent safety supervisor on the LP core** — separate clock & power
   domain, survives HP-side hangs.

---

## 3. Cores & memory

- **2× HP cores** share one address space (SMP): 768 KB internal SRAM + 32 MB
  PSRAM. Cross-core hand-off via FreeRTOS queues (carry barriers); DMA buffers
  get `esp_cache_msync` at the DMA boundary. **This is where frame buffers
  live.**
- **LP core** has its own small always-on SRAM (LP_MEM); HP can read/write it
  (the `ulp_*` symbols); LP can reach HP RAM but slowly. **The HP↔LP heartbeat
  / telemetry mailbox lives in LP_MEM**; image data never flows through LP.

---

## 4. Task → core map

| Task | Core | Prio | Role |
|------|------|------|------|
| `rs422_link`   | HP1 | **high** | OBC frame protocol: RX/TX, CRC, ACK/NACK, ARQ for bulk transfer |
| `cmd_dispatch` | HP0 | med-high | mode state machine; routes commands to subsystems; OBC-link health timer |
| `camera`       | HP0 | med | bring-up + capture SC850SL & MI1602; software ISP-lite |
| `jpeg_store`   | HP0 | med | HD frame → HW JPEG → `storage` and/or hand to `rs422_link` |
| `sstv_i2s`     | HP1 | med-high (RT) | image or arbitrary payload → encode → feed I2S DMA continuously |
| `ota`          | HP0 | low | `esp_ota` write + verify + reboot (transport-agnostic) |
| `ae_ctrl`      | HP0 | low | exposure/gain over I2C0 (isolated, throttled, fail-fast) |
| `wifi_ground`  | HP1 | low | **ground only**: C6 preview + OTA |
| `lp_supervisor`| **LP** | — | heartbeat watchdog; reads AT30TS74 temp on LP-I²C; telemetry; low-power manager; RAM scrub |

**Split logic:** HP0 = heavy compute (cameras / ISP / JPEG / OTA); HP1 =
real-time I/O (RS422 lifeline + I2S); LP = independent safety net.

---

## 5. Operating modes (state machine)

> The OBC owns a **hardware power switch** to the P4 — the master power control.
> The software modes below only exist *while powered*. "OBC dead" is not a
> software state: it means no power, so the P4 is simply off.

```
            ┌─────── BOOT ───────┐
            ▼                    │ (rollback on bad image)
        SELFTEST                 │
            │ pass               │
            ▼                    │
   ┌──> AUTONOMOUS  ──OBC cmd──>  OBC-SERVING ──done──┐
   │   (default after boot;       (capture / img-down  │
   │    capture + SSTV,           / sstv / i2s-tx /     │
   │    RGB⇄LWIR, low duty)        ota / ...)           │
   │        ▲  │ OBC: SET_MODE(standby)                 │
   │        │  ▼                                        │
   │     STANDBY (low power, RS422 stays responsive) ───┘
   │        │
   └────────┘  RS422 silent > T_link   (from ANY mode → AUTONOMOUS)

         any unrecoverable fault / reset-loop
                       ▼
                    SAFE  (RS422 + telemetry only, no imaging, await OBC)
```

- **AUTONOMOUS (default + fallback):** the state after SELFTEST, and re-entered
  from any mode when RS422 is silent for `T_link`. Every `T_beacon` (tunable)
  capture a frame and SSTV-transmit it, **alternating visible / thermal**; low
  duty cycle. (Because the OBC powered us on, it normally tasks us over RS422
  before the first beacon fires; autonomous is what runs if it doesn't / can't.)
- **One simple fallback rule:** *RS422 silent for `T_link` → AUTONOMOUS, from
  any mode.* We are powered (so the OBC wants us on) but can no longer be told
  what to do → do the useful default. **No "dead-man" logic** — if the OBC
  wanted us off/quiet it would cut power.
- **OBC-SERVING:** while the link is healthy, the OBC preempts autonomous to run
  a specific operation, then we return to AUTONOMOUS (or STANDBY if commanded).
- **STANDBY:** an OBC-commanded quiet/low-power state for short windows where
  fast resume matters (avoids a cold boot + camera bring-up). Persists only
  while RS422 is healthy; on link loss it too falls back to AUTONOMOUS. For long
  dormancy the OBC cuts power instead.
- **SAFE:** repeated faults / reset-loops (tracked in RTC RAM) → RS422 +
  telemetry only, imaging off; awaits OBC to diagnose / re-init / re-flash.

---

## 6. Interaction model

```
 OBC ──RS422──>┌─────────────┐  cmd_q   ┌──────────────┐
               │ rs422_link  │ ───────► │ cmd_dispatch │  state machine
               │ HP1 · high  │ ◄─────── │    HP0       │  + OBC-link timer
               └─────────────┘ result/  └──────┬───────┘
                 frame/CRC/ARQ data_q           │ trigger
                                  ┌─────────────┼─────────────┬────────┐
                                  ▼             ▼             ▼        ▼
                               camera        sstv_i2s     jpeg_store  ota
                               (HP0)         (HP1)         (HP0)      (HP0)
                                  │             ▲
                          frame-buffer pool (PSRAM; pass pointers via queue,
                          cache-sync at CSI-in / JPEG-/USB-out DMA edges)
               ┌─────────────┐
   telemetry ◄─┤lp_supervisor├─ polls HP heartbeats (LP_MEM mailbox);
               └─────────────┘  stall → controlled reset; reset-loop → SAFE
```

Three data flows: **(1) commands** (small, `cmd_q`); **(2) bulk data** (frames
in the pool, pointers handed around, streamed over RS422 in CRC'd windows);
**(3) telemetry/health** (LP collects → shared struct → served on OBC request).

**Backpressure = drop**, never block: if a consumer (RS422/I2S) is slow, the
producer (camera) reuses the oldest buffer and skips a frame. Comms congestion
must never freeze capture.

---

## 7. RS422 ↔ OBC protocol

- **Frame:** `SYNC(2) | LEN(2) | TYPE(1) | SEQ(1) | PAYLOAD(LEN) | CRC16(2)`.
- **Command/response:** each command → `ACK(status)` or `NACK(errcode)`.
- **Bulk transfer** (HD image down, firmware up): chunked, **per-chunk CRC +
  sequence + timeout/retransmit (ARQ)**, plus a whole-image CRC/signature.
- **Command set (small, fixed):**
  `PING`, `GET_TLM`, `CAPTURE`, `IMG_GET(begin·chunk·end)`,
  `TX_I2S(mode, data)`, `OTA(begin·chunk·end)`,
  `SET_MODE(autonomous·active·standby)`, `SET_BEACON(period, src)`,
  `REINIT(subsys)`, `RESET`.

---

## 8. Requirements → design map

| # | Requirement | How it's met |
|---|-------------|--------------|
| 1 | Reliable dual-camera bring-up + capture | `camera` task: retry'd bring-up (SC850SL I²C retries, MI1602 addr auto-detect, WDT-safe writes); independent — one down ≠ both down |
| 2 | Encode SSTV, TX over I2S | `sstv_i2s`: Robot36 encode → continuous I2S DMA |
| 3 | Reliable RS422 ↔ OBC | `rs422_link` framed protocol, own core/high prio, ARQ |
| 4 | OBC stores HD image (over RS422) | `camera`→`jpeg_store`→ `rs422_link` `IMG_GET` windowed CRC download |
| 5 | OBC transmits "something else" via I2S | OBC sends payload → `TX_I2S(mode)` → `sstv_i2s` encodes per mode → I2S out |
| 6 | OBC firmware update (over RS422) | `OTA` chunks → `esp_ota` (transport-agnostic) → verify + rollback + reboot |
| 7 | Error handling / self-recovery / low-power / rad | §9, §10, §11 |
| 8 | Ground test with WiFi module | `wifi_ground` (C6/SDIO), ground-only; flight path identical without it |
| — | **Survive RS422 failure** | **AUTONOMOUS beacon default (§5)** — capture + SSTV (RGB⇄LWIR) with no OBC |

---

## 9. Error handling & self-recovery (layered)

1. **Task level:** every blocking I/O (I²C/SPI/UART/I2S) has a timeout +
   bounded retry + **fail-fast** (no retry storms — the cause of the earlier
   watchdog reset loop). On failure → error code to OBC + degrade.
2. **Subsystem re-init:** a failed camera/thermal can be re-initialised via
   `REINIT` (or auto-retry with back-off) without a full reboot.
3. **Per-core task watchdog (TWDT):** every task feeds it.
4. **LP supervisor (independent):** HP heartbeat stalls → controlled reset;
   **reset count in RTC RAM**; reset-loop → SAFE mode so the OBC can intervene.

---

## 10. Low-power modes

> The OBC's power switch is the master / deepest control (full off). The
> software modes below are the *fast-resume* low-power options while powered.

- **AUTONOMOUS idle gap:** tickless + light-sleep between beacons; HP wakes on
  RS422 RX or the beacon timer; LP stays alive.
- **STANDBY:** commanded quiet state — sleep the heavy work, keep RS422
  responsive for instant re-tasking; power down the camera rails (rail-enable
  GPIOs) + lower CPU freq. Much faster to resume than a cold boot.
- **Full off:** the OBC flips the power switch (eclipse / long dormancy); no
  software involved, cold-boot on next power-up.

---

## 11. Radiation mitigations (software basics)

- **Watchdogs first** (SEU → hang/wrong-state → reset clears it): HP TWDT + LP
  supervisor + (ideally) an external HW watchdog.
- **TMR critical state:** store config/state variables ×3, majority vote.
- **RAM scrubbing:** LP periodically CRC-checks critical RAM; mismatch → reload
  from a known-good copy in flash. Enable PSRAM/L2 ECC if available.
- **Periodically re-assert** key sensor registers (SEU can corrupt sensor cfg).
- **Golden image:** keep one OTA slot as a never-updated recovery image +
  rollback as last resort.
- CRC on all command / firmware paths (in the protocol). Image pixel bit-flips
  are tolerated (no EDAC on pixels).

---

## 12. OTA (firmware update)

- **Partitions** (16 MB flash): `nvs` + `otadata` + `phy` + **`ota_0` (2 MB) +
  `ota_1` (2 MB)** + `storage` (~12 MB FAT). Rollback enabled
  (`CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE`).
- **Flow:** `esp_ota_begin(inactive)` → `esp_ota_write` chunks →
  `esp_ota_end` (SHA256) → `esp_ota_set_boot_partition` → reboot → first-boot
  self-test → `mark_app_valid` else **rollback**.
- **Transport-agnostic:** the same `ota_feed_chunk()` core is fed by RS422
  (flight) or WiFi/`esp_https_ota` (ground).
- **Power-fail safe** (inactive-slot write + atomic otadata flip).
  **LP supervisor** watches the new image's first boot → forces rollback if it
  doesn't come up healthy.
- Imaging is **quiesced during OTA** (flash writes stall code fetch).

---

## 13. Ground test (WiFi via C6)

- C6 over **SDIO** (`esp_wifi_remote` / ESP-Hosted; Tab5 is the reference).
- Brought up only in **ground mode** (build flag / NVS flag / C6-present
  detect); in flight `wifi_ground` is never started.
- Provides: fast UDP preview + `esp_https_ota`. The flight RS422 paths are
  byte-identical with or without WiFi (shared transport-agnostic cores).

---

## 14. Implementation roadmap

1. **OTA foundation** — dual-slot partition table + rollback + first-boot
   self-test hook. (No hardware dependency; non-breaking.)
2. **RS422 link + `cmd_dispatch` skeleton** — `PING / GET_TLM / CAPTURE` first;
   stabilise the lifeline.
3. **AUTONOMOUS beacon mode** — capture + SSTV, RGB⇄LWIR, low duty; OBC-silence
   fallback + dead-man. (Delivers the "works without RS422" guarantee early.)
4. **Multi-core refactor** — cameras/ISP on HP0, rs422/i2s on HP1, frame-buffer
   pool + queues, LP heartbeat supervisor.
5. **Bulk paths** — `IMG_GET` download, `TX_I2S`, OTA-over-RS422.
6. **Hardening** — WiFi ground channel, low-power modes, scrub/TMR.

---

## 15. Open questions / TBD

- RS422: **resolved** → UART0 (TX=GPIO37, RX=GPIO38), DE=GPIO39, full-duplex
  (RX always-on). Still TBD: transceiver part #, baud rate, DE assert/turnaround
  timing.
- C6 addon: exact P4 GPIOs behind `SDIO2_*` (from the Stamp-P4 pinmap) —
  confirm no clash with the camera/thermal/audio header.
- `T_beacon` (autonomous beacon duty cycle) and `T_link` (RS422-silence timeout
  before falling back to AUTONOMOUS).
- (The beacon need NOT be power-aware: the OBC gates gross power via its switch,
  so the payload doesn't read bus voltage to decide whether to beacon.)
- OTA slot size (2 MB proposed) vs `storage` size for capture buffering.
- MI1602: RESET_N wiring (for recovering a stuck MI48) — currently rst = −1.
