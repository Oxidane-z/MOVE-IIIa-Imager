# Deriving a 2-lane mode for the SC850SL on ESP32-P4

Source: `sensor_sc850sl_mipi.c` from OpenIPC/sensors (SigmaStar BSP, public,
re-distributable as part of OpenIPC firmware).

## What the public driver gives us

Three complete modes — all 4-lane. The header comments above each init table
are the Rosetta Stone:

| Mode | Comment header | 0x3018 | 0x3031 | PLL key regs |
|------|----------------|--------|--------|--------------|
| Linear 4K30 RAW12 | `27Minput_1c4d_1080Mbps_12bit_3840x2160_30fps` | 0x7a (4L) | 0x0c (RAW12) | 36ea=08, 36eb=0c, 36ec=4a, 36ed=24, 36fa=16, 36fb=33 |
| HDR 4K15 RAW12 (DOL) | same PLL as Linear 4K30 | 0x7a | 0x0c | identical PLL |
| HDR 4K30 RAW10 (DOL, 1458 Mbps) | `27Minput_1C4D_1458Mbps_10bit_3840x2160_30fps_SHDR` | 0x7a | 0x0a (RAW10) | 36ea=09, 36ed=34, 36fa=09, 36fb=31 |

Key facts:

- **EXTCLK = 27 MHz** in all three (so plan for 27 MHz, not 24 MHz)
- **The PLL is the same between 30 fps and 15 fps** — fps is changed only by
  doubling VTS (`0x320e/0x320f`). 15 fps linear is therefore "free":
  start from Mode A, write VTS = 0x1194 (4500), get 4K15.
- **The HDR-DOL modes transmit 2 frames worth of pixels per real frame**, so
  4K15-DOL has the *same* MIPI bandwidth as 4K30 linear. The 4-lane bus is
  already at ~1080 Mbps/lane = 4.32 Gbps total.
- `0x3037 = 0x00` even when emitting RAW10/RAW12. The datasheet is misleading
  about this field — trust the driver. **Do not write 0x20 to 0x3037 like I
  suggested earlier.**

## What we want: 2-lane on ESP32-P4

The Stamp-P4 CSI is hardwired to 2 data lanes max. We need to halve the
sensor's MIPI bus width without losing pixels.

### Bandwidth budget (≤ 1.5 Gbps / lane on the sensor side; ≤ 1.5 Gbps / lane on the P4)

| Target mode | Pixel payload | Per-lane @ 2L (+25 % MIPI overhead) | Verdict |
|---|---|---|---|
| 4K30 RAW12 | 2.99 Gbps | 1.87 Gbps | exceeds 1.5 Gbps — NO |
| 4K30 RAW10 | 2.49 Gbps | 1.56 Gbps | marginal — risky |
| 4K30 RAW8  | 1.99 Gbps | 1.24 Gbps | OK |
| 4K15 RAW12 | 1.49 Gbps | 0.93 Gbps | OK |
| 4K15 RAW10 | 1.24 Gbps | 0.78 Gbps | comfortable |
| 1080p30 RAW10 | 0.62 Gbps | 0.39 Gbps | trivial |

**Recommended capture mode for the CubeSat: 4K15 RAW10, 2-lane.** Still
captures one full-res frame in 67 ms. Good headroom for vacuum/thermal
variation.

## Three derivation strategies, easiest → hardest

### Strategy 1 — flip lane bit, keep PLL (lowest risk, fastest)

Re-use `Sensor_init_table_8M30fps[]` verbatim with two edits:

```c
{ 0x3018, 0x3a },   // was 0x7a: bit[7:5] 011 (4L) -> 001 (2L); low bits same
{ 0x3031, 0x0a },   // was 0x0c: RAW12 -> RAW10
{ 0x320c, 0x03 },   // HTS hi: copy from HDR 1458 Mbps RAW10 table (line 559)
{ 0x320d, 0x84 },   // HTS lo:   "    "
{ 0x320e, 0x11 },   // VTS hi: 4500 lines -> halves the readout fps
{ 0x320f, 0x94 },   // VTS lo
```

What this should produce, *theoretically*: SC850SL streams 4K at half-rate
on 2 lanes, RAW10 DT=0x2B. The internal `mipi_lane_sel` field in the
clk_tree (struct in the driver, line 152–173) means the sensor's PLL
already factors lane count into the MIPI clock — halving lanes should
halve the per-lane rate automatically, giving ~540 Mbps/lane and ~15 fps
effective frame rate. Bring-up risk: it may simply refuse to lock if
SmartSens hardcoded any clock-tree register to assume 4 lanes. Test with a
scope before declaring victory.

### Strategy 2 — adjust PLL registers explicitly

If Strategy 1 doesn't lock, the per-lane rate has to be set explicitly via
the PLL block (0x36e9, 0x36ea-0x36ed, 0x36f9, 0x36fa-0x36fd). We have two
known operating points (1080 Mbps/12-bit and 1458 Mbps/10-bit) — that's
not enough to extrapolate a 540 Mbps/10-bit setting without the SmartSens
PLL spec. Either (a) email SmartSens FAE for the 2-lane PLL values, or
(b) bisect experimentally by varying 0x36ea and 0x36fa one bit at a time
while monitoring lock.

### Strategy 3 — official 2-lane init table from SmartSens

Ask M5Stack first (they ship the SC850SL on the LLM630 4-lane; they may
have a 2-lane variant for their roadmap). Failing that, NDA with
SmartSens for the 2-lane/2x2-binning mode tables. Long-lead but
definitive.

## What `0x3037 = 0x00` really means

The datasheet calls bit[6:5] of 0x3037 the "PHY bit mode" with 00=8b /
01=10b / 10=12b / 11=16b. The working driver leaves it at 0x00 even when
emitting 10/12-bit data. So either:

- the PHY is rate-adaptive and the field is unused in current silicon, or
- the bit decode in the datasheet is wrong / reversed.

Either way: **leave 0x3037 at the value the driver writes (0x00 for now)**.

## Other things this driver tells us that the datasheet doesn't

- EXTCLK input frequency for tuned modes: **27 MHz** (not 24 as I'd assumed).
  The Stamp-P4 needs to source 27 MHz (or use an external XO on the carrier).
- HTS register pair: **0x320c / 0x320d** (datasheet didn't expose this).
- VTS for linear 30 fps: **0x08ca = 2250**. For 15 fps: **0x1194 = 4500**.
- Sensor ID-table format and the 0xffff sentinel-as-delay pattern used in
  the init array (see `pCus_init_linear_8M30fps`, line 912).
- Default analog/PLL preamble that must be issued before any other PLL
  write: `0x36e9 = 0x80, 0x36f9 = 0x80` (then PLL fields, then 0x36e9 = 0x00,
  0x36f9 = 0x54 to commit).
- That gain split is "analog ⇄ digital" at register 0x3e08/0x3e09 (analog)
  and 0x3e06/0x3e07 (digital), with the LE/SE pair for HDR mirrored at
  0x3e58/0x3e59 and 0x3e66/0x3e67.
