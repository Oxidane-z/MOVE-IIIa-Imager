#!/usr/bin/env python3
"""Parse Android sparse image format and convert to raw ext4 image.

Reference: system/core/libsparse/sparse_format.h in AOSP.

Sparse magic: 0xed26ff3a
Chunk types:
  0xCAC1 = RAW data (chunk_sz blocks of real data follow)
  0xCAC2 = FILL (4-byte pattern, output chunk_sz blocks of pattern)
  0xCAC3 = DON'T_CARE (skip chunk_sz blocks of output)
  0xCAC4 = CRC32 (4-byte CRC32)
"""
import sys, struct, zipfile
from pathlib import Path

CHUNK_RAW   = 0xCAC1
CHUNK_FILL  = 0xCAC2
CHUNK_SKIP  = 0xCAC3
CHUNK_CRC32 = 0xCAC4

def unsparse_stream(src, out_path):
    """Unsparse from a streaming source to out_path (which is created sparse-aware)."""
    # Read header
    magic, maj, mn, file_hdr_sz, chunk_hdr_sz, blk_sz, total_blks, total_chunks, checksum = \
        struct.unpack("<IHHHHIIII", src.read(28))
    assert magic == 0xed26ff3a, f"bad magic {magic:#x}"
    print(f"sparse v{maj}.{mn}  blk_sz={blk_sz}  total_blks={total_blks:,}  total_chunks={total_chunks:,}")
    print(f"unsparse size = {total_blks*blk_sz:,} bytes ({total_blks*blk_sz/1e9:.2f} GB)")

    # Skip extra header bytes if any
    if file_hdr_sz > 28: src.read(file_hdr_sz - 28)

    counts = {"RAW":0, "FILL":0, "SKIP":0, "CRC32":0}
    bytes_written = 0

    with open(out_path, "wb") as out:
        # Pre-allocate sparsely on NTFS: seek to end and write 0
        out.seek(total_blks * blk_sz - 1)
        out.write(b"\x00")
        out.seek(0)

        for ci in range(total_chunks):
            ctype, _r, chunk_blks, total_sz = struct.unpack("<HHII", src.read(12))
            payload_sz = total_sz - 12

            if ctype == CHUNK_RAW:
                counts["RAW"] += 1
                # Stream copy
                remaining = payload_sz
                while remaining:
                    buf = src.read(min(remaining, 1<<20))
                    if not buf: raise EOFError
                    out.write(buf)
                    remaining -= len(buf)
                bytes_written += chunk_blks * blk_sz
            elif ctype == CHUNK_FILL:
                counts["FILL"] += 1
                pat = src.read(4)
                # Write pattern * chunk_blks blocks
                fill_block = pat * (blk_sz // 4)
                for _ in range(chunk_blks):
                    out.write(fill_block)
                bytes_written += chunk_blks * blk_sz
            elif ctype == CHUNK_SKIP:
                counts["SKIP"] += 1
                out.seek(chunk_blks * blk_sz, 1)  # seek forward, leave as sparse 0
            elif ctype == CHUNK_CRC32:
                counts["CRC32"] += 1
                src.read(4)
            else:
                raise ValueError(f"unknown chunk type {ctype:#x} at chunk {ci}")

            if ci and ci % 5000 == 0:
                print(f"  chunk {ci:,}/{total_chunks:,}  written~{bytes_written/1e9:.2f} GB  {counts}")

    print(f"\ndone. chunks: {counts}")
    print(f"bytes written: {bytes_written:,} ({bytes_written/1e9:.2f} GB)")

if __name__ == "__main__":
    axp = "M5_LLM_ubuntu22.04_20250328_AX630C_LITE.axp"
    out = "rootfs.ext4"
    z = zipfile.ZipFile(axp)
    with z.open("rootfs_sparse.ext4") as src:
        unsparse_stream(src, out)
