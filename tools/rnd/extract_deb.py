#!/usr/bin/env python3
"""Extract a .deb (ar archive) without needing the `ar` binary."""
import sys, os, struct, tarfile, lzma, gzip, io

def parse_ar(path):
    """Yield (name, data) for each member of an ar archive."""
    with open(path, 'rb') as f:
        magic = f.read(8)
        if magic != b'!<arch>\n':
            raise ValueError(f"Not an ar archive: magic={magic!r}")
        while True:
            hdr = f.read(60)
            if not hdr:
                return
            if len(hdr) < 60:
                return
            name = hdr[0:16].decode('ascii', errors='replace').rstrip().rstrip('/')
            size = int(hdr[48:58].decode('ascii').strip())
            data = f.read(size)
            if size % 2:
                f.read(1)  # padding
            yield name, data

if __name__ == "__main__":
    deb = sys.argv[1] if len(sys.argv) > 1 else "llm-camera_1.9-m5stack1_arm64.deb"
    out = sys.argv[2] if len(sys.argv) > 2 else "extracted"
    os.makedirs(out, exist_ok=True)

    for name, data in parse_ar(deb):
        print(f"[ar] member {name!r}  ({len(data)} bytes)")
        if name.startswith("data.tar"):
            # decompress
            if name.endswith(".xz"):
                raw = lzma.decompress(data)
            elif name.endswith(".gz"):
                raw = gzip.decompress(data)
            elif name.endswith(".zst"):
                import zstandard
                raw = zstandard.ZstdDecompressor().stream_reader(io.BytesIO(data)).read()
            else:
                raw = data
            print(f"  decompressed {len(raw)} bytes")
            tf = tarfile.open(fileobj=io.BytesIO(raw))
            tf.extractall(out)
            print(f"  extracted to {out}/")
            for m in tf.getmembers():
                if m.isfile():
                    print(f"    {m.name}  ({m.size} bytes)")
