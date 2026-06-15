# RS485 link test (2-board console bridge)

A tiny standalone ESP-IDF app to prove an **RS485 full-duplex** link between two
ESP32-P4 boards (TI **THVD1424** transceivers). Flash the **same** image to both
boards; each board's USB console becomes one end of a two-way serial chat over
RS485.

What each board does:
- sends a `[<id>] hb #N` heartbeat over RS485 once a second (`<id>` = last 3 MAC
  bytes, so the two boards are distinguishable);
- prints anything it receives over RS485 to its USB console;
- sends anything you type on its console out over RS485.

**Pass criterion:** open both consoles — each shows the *other* board's
`[<id>] hb #N` ticking every second (proves the link works in both directions),
and text typed on one console appears on the other.

## Wiring (full-duplex, 4-wire + ground)

Per board, P4 ↔ THVD1424:

| P4 (UART1) | THVD1424 | note |
|---|---|---|
| GPIO38 (TX) | `DI` (driver input) | board wiring: DI on G38 |
| GPIO37 (RX) | `RO` (receiver output) | board wiring: RO on G37 |
| GPIO39 (DE) | `DE` (driver enable) | active-high; held high by the app |
| — | `RE` (receiver enable) | tie enabled (active-low → GND), or wire to a GPIO and set `RS485_RE_GPIO` |

Board-to-board bus (full-duplex = two differential pairs, cross-connected):

```
   Board A  Y/Z (driver out)  ───────►  Board B  A/B (receiver in)
   Board B  Y/Z (driver out)  ───────►  Board A  A/B (receiver in)
   Board A  GND  ───────────────────────  Board B  GND
```

Terminate each **receiver** pair with 120 Ω (across A/B at each end), or enable
the THVD1424 on-chip termination via its `TERM_RX` pin. Keep the pair wires
twisted; share a ground.

> The app uses **UART1** (not UART0) so the ROM's boot-time UART0 chatter never
> appears on the bus, and the console is on USB-Serial-JTAG. It uses **normal**
> UART mode (full-duplex), not the half-duplex RS485 mode.

## Build & flash

ESP-IDF v6.0.1. From this directory:

```sh
idf.py set-target esp32p4      # one-time
idf.py build
idf.py -p <COMx> flash monitor # do this for each board (its own COM port)
```

On Windows you can use the wrapper (mirrors the firmware ones; handles the
MSYS/`export.bat` quirk):

```sh
cmd //c "C:\Users\zeyu.zhu\Pictures\SC850SL Dev\rs485_test\_build.bat"
```

Then flash each board from its laptop with `idf.py -p <COMx> flash monitor`
(the build artifact is `build/rs485_bridge.bin`; same image on both boards).

## Tuning

Edit the `#define`s at the top of `main/rs485_bridge.c`: pins, `RS485_BAUD`
(**3000000** = 3 Mbps default; the link tested clean to 15 Mbps, so you can go
much higher), `RS485_RE_GPIO` if `RE` is on a GPIO rather than tied enabled, and
**`RS485_INVERT_RX`** (invert RXD to compensate for a reversed A/B differential
pair — see below). The heartbeat is 1 s; raise its `vTaskDelay` if it clutters
your chat.

Helper scripts (run with the IDF python): `hexcap.py <COMx> <secs>` dumps the raw
received bytes as hex (deterministic garbage = inverted/wrong-framed, not a dead
wire); `typetest.py <COMa> <COMb>` injects a message on each port and checks it
arrives on the other — the automated version of "type in one window, watch the
other".

## Verified

Two Stamp-P4 (**rev v1.3**) boards, THVD1424 full-duplex:

- **Link (2026-06-07):** clean heartbeats **both directions** (`[<id>] hb #N`,
  counter advancing, no drops).
- **Baud sweep (2026-06-16):** clean every step from 460 800 up to **15 Mbps**
  (460 k / 921 k / 1.84 M / 3.69 M / 5 M / 7.5 M / 10 M / 15 M). The ceiling is the
  THVD1424's own 20 Mbps rating, not the wiring or the P4. Default left at 3 Mbps
  for margin.
- **Typing (2026-06-16):** injected a message on each console and saw it on the
  other — bidirectional console <-> RS485 chat confirmed.

Two gotchas hit + fixed during bring-up, both recorded in the source:

- **Chip revision:** these are P4 **rev v1.3** engineering silicon; a fresh
  IDF v6.0.1 project defaults to rev v3.x and refuses to flash. Fixed in
  `sdkconfig.defaults` (`ESP32P4_SELECTS_REV_LESS_V3=y` + `REV_MIN_0`).
- **Swapped A/B pair:** the differential pair was wired reverse-polarity, so the
  receiver saw inverted logic → deterministic garbage. Fixed in software with
  `RS485_INVERT_RX=1` (`uart_set_line_inverse(..., UART_SIGNAL_RXD_INV)`), no
  rewiring. If you wire A/B straight, set `RS485_INVERT_RX 0`.
