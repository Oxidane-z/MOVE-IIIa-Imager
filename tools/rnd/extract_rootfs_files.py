#!/usr/bin/env python3
"""Extract sc850sl files from rootfs.ext4."""
import io, os, sys
import ext4

ROOTFS = "rootfs.ext4"
OUTDIR = "rootfs_files"

sys.stdout.reconfigure(encoding="utf-8")

def list_dir(dir_inode):
    for de, ftype in dir_inode.opendir():
        name = de.name
        if isinstance(name, bytes):
            # The library sometimes includes a trailing record-length byte.
            # Drop everything from the first non-printable.
            try:
                clean = name.split(b'\x00',1)[0]
                # Drop trailing bytes that aren't valid filename chars
                for cut in range(len(clean), 0, -1):
                    b = clean[cut-1]
                    if b < 0x20:
                        clean = clean[:cut-1]
                    else:
                        break
                name = clean.decode('utf-8', 'replace')
            except Exception:
                name = name.decode('utf-8', 'replace')
        if name in ('.', '..'): continue
        try:
            child = dir_inode.volume.inodes[de.inode]
        except Exception:
            continue
        yield name, ftype, child

def find_inode_by_path(root, path, follow=True, depth=0):
    if depth > 8: return None
    parts = [p for p in path.split("/") if p]
    cur = root
    for part in parts:
        found = None
        for name, ftype, inode in list_dir(cur):
            if name == part:
                found = inode
                break
        if found is None: return None
        cur = found
        if isinstance(cur, ext4.SymbolicLink) and follow:
            tgt = cur.readlink()
            if isinstance(tgt, bytes): tgt = tgt.decode('utf-8','replace')
            if not tgt.startswith("/"):
                # join with current path component dir
                tgt = "/".join(path.split("/")[:-1] + [tgt])
            cur = find_inode_by_path(root, tgt, follow=True, depth=depth+1)
            if cur is None: return None
    return cur

def safe_save(out_path, data):
    # sanitize filename
    clean = []
    for ch in out_path:
        if ch == '\x00' or (0 < ord(ch) < 0x20):
            continue
        clean.append(ch)
    out_path = ''.join(clean)
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "wb") as out:
        out.write(data)
    return out_path

def read_file_inode(inode):
    if isinstance(inode, ext4.File):
        return inode.open().read()
    return None

def main():
    f = io.open(ROOTFS, "rb", buffering=1<<20)
    vol = ext4.Volume(f)
    root = vol.root

    targets = [
        # Full mode INIs
        "/opt/etc/sc850sl_sdr_2lane_15fps.ini",
        "/opt/etc/sc850sl_sdr_4lane.ini",
        "/opt/etc/sc850sl_sdr_4lane_620q.ini",
        "/opt/etc/sc850sl_hdr_4lane.ini",
        "/opt/etc/sc850sl_hdr_4lane_620q.ini",
        "/opt/etc/sc850sl_os04a10_double_sdr_online_entry.ini",
        # Entry stubs
        "/opt/etc/sc850sl_single_sdr_2lane_15fps_entry.ini",
        "/opt/etc/sc850sl_single_sdr_4lane_entry.ini",
        "/opt/etc/sc850sl_single_hdr_4lane_entry.ini",
        "/opt/etc/sc850sl_single_sdr_4lane_entry_620q.ini",
        "/opt/etc/sc850sl_single_hdr_4lane_entry_620q.ini",
        # Binary tuning / AE tables
        "/opt/etc/sc850sl_sdr_mode3_switch_mode7.bin",
        "/opt/etc/sc850sl_sdr_ptnw768_600G_25fps.bin",
        "/opt/etc/sc850sl_hdr_2x_ratio_default.bin",
        "/opt/etc/sc850sl_hdr_2x_ratio_1to1.bin",
        # Sensor driver
        "/opt/lib/libsns_sc850sl.so.0.0.0",
        # Application configs
        "/opt/bin/FRTDemo/config/ipc/sensor/sc850sl.json",
        "/opt/bin/FRTTest/config/ipc/sensor/sc850sl.json",
    ]

    os.makedirs(OUTDIR, exist_ok=True)
    print("=== extracting ===")
    for path in targets:
        # Brute search since dir traversal might have name-encoding glitches
        parts = path.split("/")
        parent_path = "/".join(parts[:-1])
        target_name = parts[-1]
        parent = find_inode_by_path(root, parent_path)
        if parent is None:
            print(f"  - {path}: parent dir not found")
            continue
        inode = None
        for name, ftype, child in list_dir(parent):
            if name == target_name:
                inode = child
                break
        if inode is None:
            # try fuzzy match (sometimes name has trailing garbage)
            for name, ftype, child in list_dir(parent):
                if name.startswith(target_name[:30]):
                    print(f"  ~ fuzzy match: {name!r}")
                    inode = child
                    break
        if inode is None:
            print(f"  - {path}: NOT FOUND")
            continue
        try:
            data = read_file_inode(inode)
            if data is None:
                print(f"  ! {path}: not a regular file")
                continue
            out_path = os.path.join(OUTDIR, *[p for p in path.split("/") if p])
            saved = safe_save(out_path, data)
            print(f"  + {len(data):>10,} -> {saved}")
        except Exception as e:
            print(f"  ! {path}: {e}")

    # Also list everything in /opt/etc that mentions sc850sl
    print("\n=== full /opt/etc/sc850sl_* listing ===")
    opt_etc = find_inode_by_path(root, "/opt/etc")
    if opt_etc:
        for name, ftype, inode in list_dir(opt_etc):
            if 'sc850sl' in name.lower():
                sz = inode.i_size_lo if hasattr(inode, 'i_size_lo') else 0
                print(f"  {sz:>10,}  /opt/etc/{name}")

if __name__ == "__main__":
    main()
