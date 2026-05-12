#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
"""Package a raw Zephyr image for the Valve Ibex bootloader."""

import argparse
import struct
import zlib
from pathlib import Path

IBEX_FW_MAGIC = 0xD2D86467
HEADER_SIZE = 32
FLASH_SIZE = 512 * 1024
APP_LOAD_ADDRESS = 0x8000
APP_HEADER_ADDRESS = 0x7CFE0
APP_MAX_SIZE = 0x70000
SRAM_START = 0x20000000
SRAM_END = SRAM_START + 128 * 1024


def validate_payload(payload: bytes) -> tuple[int, int]:
    if len(payload) < 8:
        raise ValueError("payload is too short to contain an ARM vector table")
    if len(payload) > APP_MAX_SIZE:
        raise ValueError(
            f"payload overlaps custom-firmware settings ({len(payload)} > {APP_MAX_SIZE})"
        )
    if len(payload) % 4:
        raise ValueError("payload length is not word-aligned")

    initial_sp, reset_vector = struct.unpack_from("<II", payload)
    if not SRAM_START <= initial_sp <= SRAM_END:
        raise ValueError(f"initial SP 0x{initial_sp:08x} is outside Ibex SRAM")
    if not reset_vector & 1:
        raise ValueError(f"reset vector 0x{reset_vector:08x} is not Thumb code")
    reset_address = reset_vector & ~1
    if not APP_LOAD_ADDRESS <= reset_address < APP_LOAD_ADDRESS + len(payload):
        raise ValueError(
            f"reset vector 0x{reset_vector:08x} is outside the payload loaded at "
            f"0x{APP_LOAD_ADDRESS:08x}"
        )
    return initial_sp, reset_vector


def package(payload: bytes) -> bytes:
    header = struct.pack(
        "<8I",
        IBEX_FW_MAGIC,
        len(payload),
        zlib.crc32(payload) & 0xFFFFFFFF,
        0,
        0,
        0,
        0,
        0,
    )
    return header + payload


def verify_container(container: bytes) -> bytes:
    if len(container) < HEADER_SIZE:
        raise ValueError("firmware container is shorter than its header")

    magic, payload_len, expected_crc, *reserved = struct.unpack_from("<8I", container)
    payload = container[HEADER_SIZE:]
    if magic != IBEX_FW_MAGIC:
        raise ValueError(f"unexpected firmware magic 0x{magic:08x}")
    if payload_len != len(payload):
        raise ValueError(
            f"header length {payload_len} does not match payload length {len(payload)}"
        )
    if any(reserved):
        raise ValueError("reserved firmware header words are nonzero")

    actual_crc = zlib.crc32(payload) & 0xFFFFFFFF
    if actual_crc != expected_crc:
        raise ValueError(f"header CRC32 0x{expected_crc:08x} does not match 0x{actual_crc:08x}")
    validate_payload(payload)
    return payload


def write_if_changed(path: Path, data: bytes) -> None:
    if path.exists() and path.read_bytes() == data:
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)


def validate_elf_load_address(path: Path) -> None:
    data = path.read_bytes()
    if len(data) < 52 or data[:4] != b"\x7fELF" or data[4:6] != b"\x01\x01":
        raise ValueError(f"{path} is not a 32-bit little-endian ELF")

    (program_header_offset,) = struct.unpack_from("<I", data, 28)
    program_header_size, program_header_count = struct.unpack_from("<HH", data, 42)
    load_addresses = []
    for index in range(program_header_count):
        offset = program_header_offset + index * program_header_size
        if offset + program_header_size > len(data):
            raise ValueError(f"{path} has a truncated program header table")
        segment_type, _, virtual_address, physical_address, file_size = struct.unpack_from(
            "<IIIII", data, offset
        )
        if segment_type == 1 and file_size:
            load_addresses.append(physical_address or virtual_address)

    if not load_addresses or min(load_addresses) != APP_LOAD_ADDRESS:
        actual = min(load_addresses) if load_addresses else None
        raise ValueError(f"ELF first load address is {actual!r}, expected 0x{APP_LOAD_ADDRESS:08x}")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=Path, required=True, help="raw Zephyr binary")
    parser.add_argument(
        "--elf", type=Path, required=True, help="Zephyr ELF used to verify the link address"
    )
    parser.add_argument("--output", type=Path, required=True, help="output Ibex .fw container")
    args = parser.parse_args()

    validate_elf_load_address(args.elf)
    payload = args.input.read_bytes()
    validate_payload(payload)
    container = package(payload)
    verify_container(container)
    write_if_changed(args.output, container)

    print(
        f"IBEX FW: {args.output} ({len(payload)} byte payload, "
        f"CRC32 0x{zlib.crc32(payload) & 0xFFFFFFFF:08X})"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
