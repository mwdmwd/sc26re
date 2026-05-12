#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
import argparse
import datetime
from pathlib import Path
import struct
import zlib

MAGICS = {
    0xD2D86467: "IBEX (controller)",
    0x2E795631: "PROTEUS (puck)",
}


def parse_one(path: Path, extract: bool):
    b = path.read_bytes()
    if len(b) < 32:
        raise SystemExit(f"{path}: too small")

    words = struct.unpack_from("<8I", b, 0)
    magic, payload_len, crc = words[:3]
    payload = b[32:]

    print(f"== {path.name} ==")
    print(f"type:        {MAGICS.get(magic, 'unknown!')}")
    print(f"magic:       {magic:#10x}")
    print(f"file size:   {len(b):#x}")
    print(f"payload len: {payload_len:#x}")
    print(f"crc header:  {crc:#10x}")
    print(f"crc actual:  {zlib.crc32(payload) & 0xffffffff:#10x}")
    print(f"len ok:      {payload_len == len(payload)}")
    print(f"crc ok:      {crc == (zlib.crc32(payload) & 0xffffffff)}")
    print(f"reserved:    {' '.join(f'{x:#010x}' for x in words[3:])}")

    # decode suffix like in IBEX_FW_69FE17FF.fw
    stem = path.stem
    suffix = stem.rsplit("_", 1)[-1]
    try:
        ts = int(suffix, 16)
        dt = datetime.datetime.fromtimestamp(ts, datetime.UTC)
        print(f"suffix time: {dt.isoformat()}")
    except Exception:
        pass

    sp = struct.unpack_from("<I", payload, 0)[0]
    reset = struct.unpack_from("<I", payload, 4)[0]
    print(f"initial SP:  0x{sp:08x}")
    print(f"reset vec:   0x{reset:08x}")

    if extract:
        out = path.with_suffix(path.suffix + ".payload.bin")
        out.write_bytes(payload)
        print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+", type=Path)
    ap.add_argument("-x", "--extract", action="store_true")
    args = ap.parse_args()
    for f in args.files:
        parse_one(f, args.extract)


if __name__ == "__main__":
    main()
