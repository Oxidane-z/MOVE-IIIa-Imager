#!/usr/bin/env python3
"""Decode all SC850SL register init tables from libsns_sc850sl.so.

Table format: { u32 reg_LE, u32 val_LE } records.
Tables start with reg=0x0103, val=0x01 (soft reset) and end with
reg=0x0100, val=0x01 (stream on) or just trailing zeros.
"""
import struct, re
from elftools.elf.elffile import ELFFile

SO = "rootfs_files/opt/lib/libsns_sc850sl.so.0.0.0"

def main():
    with open(SO, "rb") as f:
        elf = ELFFile(f)
        for sec in elf.iter_sections():
            if sec.name == ".data":
                data = sec.data()
                addr = sec.header['sh_addr']
                break

    # Find all 8-byte aligned records where reg == 0x0103, val == 0x01 (soft reset)
    starts = []
    for off in range(0, len(data) - 8, 8):
        reg, val = struct.unpack("<II", data[off:off+8])
        if reg == 0x0103 and val == 0x01:
            starts.append(off)
    print(f"Found {len(starts)} candidate table starts: {[hex(s) for s in starts]}")
    print()

    tables = []
    for ti, start in enumerate(starts):
        # Bound this table by the start of the next one
        end_limit = starts[ti+1] if ti+1 < len(starts) else len(data)
        entries = []
        off = start
        saw_stream_on = False
        while off + 8 <= end_limit:
            reg, val = struct.unpack("<II", data[off:off+8])
            # If we've already seen stream_on, stop at next zero pair
            if saw_stream_on and reg == 0 and val == 0:
                break
            # Sanity: reg in 0x0000-0x5FFF or 0xFFFF (delay sentinel)
            if reg > 0x5FFF and reg != 0xFFFF:
                break
            entries.append((reg, val))
            off += 8
            # Detect end-of-table marker (stream-on then optionally a few more)
            if reg == 0x0100 and val == 0x01:
                saw_stream_on = True
        tables.append((start, entries))
        print(f"Table #{ti}: starts at offset 0x{start:04x} (vaddr 0x{addr+start:08x}), {len(entries)} entries")

    # Identify each table by its 0x3018 (lane) and 0x3031 (bit depth) writes
    print()
    for ti, (start, entries) in enumerate(tables):
        regs = dict(entries)
        lane_reg = regs.get(0x3018, None)
        bit_reg  = regs.get(0x3031, None)
        # Decode lane from bit[7:5]
        if lane_reg is not None:
            lane_bits = (lane_reg >> 5) & 0x7
            lane_n = {0: 1, 1: 2, 3: 4, 7: 8}.get(lane_bits, '?')
        else:
            lane_n = '?'
        # Bit depth
        bit_n = {0x08: 'RAW8', 0x0a: 'RAW10', 0x0c: 'RAW12'}.get(bit_reg, f'0x{bit_reg:02x}' if bit_reg is not None else '?')
        # HTS / VTS
        hts = (regs.get(0x320c, 0) << 8) | regs.get(0x320d, 0)
        vts = (regs.get(0x320e, 0) << 8) | regs.get(0x320f, 0)
        # Output W/H
        out_w = (regs.get(0x3208, 0) << 8) | regs.get(0x3209, 0)
        out_h = (regs.get(0x320a, 0) << 8) | regs.get(0x320b, 0)
        # Pixel clock estimate: HTS*VTS*fps -- unknown fps, but report HTS/VTS
        print(f"\nTable #{ti}: lane={lane_n}, bit={bit_n}, HTS=0x{hts:04x}={hts}, "
              f"VTS=0x{vts:04x}={vts}, out=({out_w}, {out_h})")
        # PLL key regs
        for r in [0x36e9, 0x36ea, 0x36eb, 0x36ec, 0x36ed, 0x36f9, 0x36fa, 0x36fb, 0x36fc, 0x36fd]:
            if r in regs:
                print(f"  0x{r:04x} = 0x{regs[r]:02x}", end='  ')
        print()

    # Write each table as a C-style array to a file
    for ti, (start, entries) in enumerate(tables):
        out_name = f"sc850sl_table_{ti}.c"
        with open(out_name, "w") as out:
            regs = dict(entries)
            lane_reg = regs.get(0x3018, 0)
            lane_bits = (lane_reg >> 5) & 0x7
            lane_n = {0: 1, 1: 2, 3: 4, 7: 8}.get(lane_bits, '?')
            bit_reg = regs.get(0x3031, 0)
            bit_n = {0x0a: 'RAW10', 0x0c: 'RAW12', 0x08: 'RAW8'}.get(bit_reg, '???')
            out.write(f"// SC850SL init table #{ti} from libsns_sc850sl.so .data @ 0x{addr+start:08x}\n")
            out.write(f"// lanes={lane_n}, bit_depth={bit_n}, {len(entries)} entries\n")
            out.write(f"// Format: {{ u16 reg, u8 val }}\n\n")
            out.write(f"static const struct {{ uint16_t reg; uint8_t val; }} sc850sl_table_{ti}[] = {{\n")
            for reg, val in entries:
                out.write(f"    {{ 0x{reg:04x}, 0x{val:02x} }},\n")
            out.write("};\n")
        print(f"wrote {out_name}")

if __name__ == "__main__":
    main()
