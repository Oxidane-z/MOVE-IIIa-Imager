#!/usr/bin/env python3
"""Three-way SC850SL register diff: Sophgo CV183x vs M5Stack tables.

Compares the *effective* (last-write-wins) register file of three INDEPENDENT
4K init sequences from three independent vendors:

  S  = Sophgo/CVITEK CV183x  sc850sl_linear_2160P30_init  (4-lane RAW12 4K30)
  T0 = M5Stack/Axera         sc850sl_table_0              (4-lane RAW10 4K30)
  T2 = M5Stack/Axera         sc850sl_table_2              (2-lane RAW10 4K15)  <- our flight mode

Purpose: when we hand-derive a sub-4K / binning / 1080p mode (no vendor ships
one), we need to know which registers are SENSOR-CORE INVARIANTS (identical
across all independent sources -> safe to keep untouched) vs PER-MODE KNOBS
(differ by lane/bitdepth/timing/PLL -> must be recomputed for a new mode).

Run from anywhere; paths are resolved relative to this file.
"""
import re, os

HERE = os.path.dirname(os.path.abspath(__file__))
REF  = os.path.dirname(HERE)  # reference/

SOPHGO_CTL = os.path.join(HERE, "sc850sl_sensor_ctl.c")
M5_T0      = os.path.join(REF, "sc850sl_table_0.c")
M5_T2      = os.path.join(REF, "sc850sl_table_2.c")

def parse_sophgo_func(path, func_name):
    """Extract (reg,val) writes inside one sc850sl_*_init function body."""
    src = open(path, encoding="utf-8", errors="replace").read()
    # find function def, then capture to the matching close brace (flat body)
    m = re.search(r'\bvoid\s+%s\s*\(VI_PIPE ViPipe\)\s*\{' % re.escape(func_name), src)
    if not m:
        raise RuntimeError("func %s not found" % func_name)
    start = m.end()
    depth = 1
    i = start
    while i < len(src) and depth:
        if src[i] == '{': depth += 1
        elif src[i] == '}': depth -= 1
        i += 1
    body = src[start:i]
    pat = re.compile(r'sc850sl_write_register\(ViPipe,\s*0x([0-9a-fA-F]{1,4}),\s*0x([0-9a-fA-F]{1,2})\)')
    return [(int(r,16), int(v,16)) for r,v in pat.findall(body)]

def parse_m5(path):
    src = open(path, encoding="utf-8", errors="replace").read()
    pat = re.compile(r'\{\s*0x([0-9a-fA-F]{1,4}),\s*0x([0-9a-fA-F]{1,2})\s*\}')
    return [(int(r,16), int(v,16)) for r,v in pat.findall(src)]

def effective(seq):
    """last-write-wins -> {reg: val}; also return write count."""
    d = {}
    for r,v in seq:
        d[r] = v
    return d

S_seq  = parse_sophgo_func(SOPHGO_CTL, "sc850sl_linear_2160P30_init")
T0_seq = parse_m5(M5_T0)
T2_seq = parse_m5(M5_T2)
S, T0, T2 = effective(S_seq), effective(T0_seq), effective(T2_seq)

print("=== write counts (raw / unique effective) ===")
print(f"  Sophgo linear 4K30 4L RAW12 : {len(S_seq):3d} writes / {len(S):3d} unique regs")
print(f"  M5Stack table_0 4K30 4L RAW10: {len(T0_seq):3d} writes / {len(T0):3d} unique regs")
print(f"  M5Stack table_2 4K15 2L RAW10: {len(T2_seq):3d} writes / {len(T2):3d} unique regs")

allk = sorted(set(S)|set(T0)|set(T2))
common = sorted(set(S)&set(T0)&set(T2))

ident_all3      = [r for r in common if S[r]==T0[r]==T2[r]]
s_eq_t0_ne_t2   = [r for r in common if S[r]==T0[r]!=T2[r]]
t0_eq_t2_ne_s   = [r for r in common if T0[r]==T2[r]!=S[r]]
all_differ      = [r for r in common if len({S[r],T0[r],T2[r]})==3]
s_eq_t2_ne_t0   = [r for r in common if S[r]==T2[r]!=T0[r]]
only_S  = sorted(set(S)-set(T0)-set(T2))
only_M5 = sorted((set(T0)|set(T2))-set(S))

print(f"\n=== classification over {len(common)} regs common to ALL THREE ===")
print(f"  identical in all 3 (SENSOR-CORE INVARIANTS): {len(ident_all3)}")
print(f"  S==T0 != T2  (4-lane/30fps trait)          : {len(s_eq_t0_ne_t2)}")
print(f"  T0==T2 != S  (RAW10 vs Sophgo-RAW12 trait)  : {len(t0_eq_t2_ne_s)}")
print(f"  S==T2 != T0  (2-lane-ish coincidence)       : {len(s_eq_t2_ne_t0)}")
print(f"  all three differ (PLL/timing volatile)      : {len(all_differ)}")
print(f"  only in Sophgo (not in either M5 table)     : {len(only_S)}")
print(f"  only in M5 tables (absent from Sophgo)      : {len(only_M5)}")

def show(label, regs, n=999):
    if not regs: return
    print(f"\n--- {label} ({len(regs)}) ---")
    for r in regs[:n]:
        sv  = f"{S[r]:02x}"  if r in S  else "--"
        t0v = f"{T0[r]:02x}" if r in T0 else "--"
        t2v = f"{T2[r]:02x}" if r in T2 else "--"
        print(f"  0x{r:04x}:  S={sv}  T0={t0v}  T2={t2v}")

# The decision table: key mode-defining registers side by side
KEY = [0x3018,0x3019,0x301a,0x301e,0x301f,0x3031,0x3037,
       0x3208,0x3209,0x320a,0x320b,0x320c,0x320d,0x320e,0x320f,
       0x3210,0x3211,0x3212,0x3213,0x3221,
       0x36e9,0x36ea,0x36eb,0x36ec,0x36ed,0x36f9,0x36fa,0x36fb,0x36fc,0x36fd,
       0x3e00,0x3e01,0x3e02,0x3e03,0x3e06,0x3e08,0x3e09]
print("\n=== KEY mode-defining registers (lane/bit/window/timing/PLL/exposure) ===")
print("     reg     Sophgo  M5_T0  M5_T2   note")
notes = {0x3018:"lane bits[7:5]",0x3031:"bit depth 0a=R10 0c=R12",0x3037:"PHY",
         0x320c:"HTS hi",0x320d:"HTS lo",0x320e:"VTS hi",0x320f:"VTS lo",
         0x3208:"outW hi",0x320a:"outH hi",0x3221:"mirror/flip",
         0x36e9:"PLL commit",0x36fa:"MIPI PLL"}
for r in KEY:
    sv  = f"0x{S[r]:02x}"  if r in S  else " -- "
    t0v = f"0x{T0[r]:02x}" if r in T0 else " -- "
    t2v = f"0x{T2[r]:02x}" if r in T2 else " -- "
    mark = "" if (r in S and r in T0 and r in T2 and S[r]==T0[r]==T2[r]) else "  <-differs"
    print(f"  0x{r:04x}   {sv:>5}   {t0v:>5}  {t2v:>5}   {notes.get(r,'')}{mark}")

show("SENSOR-CORE INVARIANTS (identical all 3 -> keep when deriving new mode)", ident_all3)
show("ONLY in Sophgo (candidate new data M5Stack lacks)", only_S)
show("ONLY in M5Stack tables (Sophgo omits)", only_M5)
