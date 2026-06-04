#!/usr/bin/env python3
"""Dump SC850SL config symbols (MipiAttr, SnsClkAttr, etc.) from llm_camera ELF."""
import sys, struct
from pathlib import Path
from elftools.elf.elffile import ELFFile
from elftools.dwarf.descriptions import describe_form_class

BIN = Path(__file__).parent / "extracted/opt/m5stack/bin/llm_camera-1.9"

TARGETS = [
    "gSc850slMipiAttr",     # MIPI lane/clock attribute
    "gSc850slChn0Attr",     # channel 0
    "gSc850slPipeAttr",
    "gSc850slDevAttr",
    "gSc850slSnsAttr",
    "gSc850slSnsClkAttr",
    "gSnssc850slObj",
]

def read_vaddr(elf, vaddr, size):
    """Read `size` bytes at virtual address `vaddr` from any LOAD segment."""
    for seg in elf.iter_segments():
        if seg['p_type'] != 'PT_LOAD':
            continue
        v = seg['p_vaddr']; f = seg['p_offset']; m = seg['p_memsz']; fs = seg['p_filesz']
        if v <= vaddr < v + m:
            off = f + (vaddr - v)
            # If beyond filesz it's bss / zero
            end_file = f + fs
            avail = max(0, end_file - off)
            real = elf.stream
            real.seek(off)
            chunk = real.read(min(size, avail))
            return chunk + b"\x00" * (size - len(chunk))
    return None

def main():
    with BIN.open('rb') as f:
        elf = ELFFile(f)
        # Build symbol table index
        sym_by_name = {}
        for sec in elf.iter_sections():
            if sec.name not in ('.symtab', '.dynsym'):
                continue
            for sym in sec.iter_symbols():
                if sym.name and sym.entry['st_size'] > 0:
                    sym_by_name[sym.name] = sym

        print(f"loaded {len(sym_by_name):,} symbols\n")

        for name in TARGETS:
            if name not in sym_by_name:
                # try partial match
                cand = [s for s in sym_by_name if name in s]
                if cand:
                    print(f"# {name}: not direct, candidates {cand[:5]}")
                    name = cand[0]
                else:
                    print(f"# {name}: NOT FOUND")
                    continue
            sym = sym_by_name[name]
            addr = sym.entry['st_value']
            size = sym.entry['st_size']
            print(f"=== {name} @ 0x{addr:x}  size={size} ===")
            buf = read_vaddr(elf, addr, size)
            if buf is None:
                print("  (no LOAD segment covers it)")
                continue
            # hex + interpret as u32 array for skim
            for i in range(0, len(buf), 16):
                chunk = buf[i:i+16]
                hex_s = ' '.join(f"{b:02x}" for b in chunk)
                u32s = struct.unpack(f"<{len(chunk)//4}I", chunk[:len(chunk)//4*4]) if len(chunk) >= 4 else ()
                u32_s = ' '.join(f"{x:>10}" for x in u32s)
                ascii_s = ''.join(chr(b) if 0x20<=b<0x7f else '.' for b in chunk)
                print(f"  +{i:04x}  {hex_s:<48}  |{ascii_s}|")
            print()

if __name__ == "__main__":
    main()
