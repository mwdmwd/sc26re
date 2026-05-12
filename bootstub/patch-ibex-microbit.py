#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
import argparse
import struct
from dataclasses import dataclass
from pathlib import Path

from keystone import KS_ARCH_ARM, KS_MODE_LITTLE_ENDIAN, KS_MODE_THUMB, Ks, KsError


@dataclass(frozen=True)
class ThumbPatch:
    addr: int
    expected: tuple[str, ...]
    replacement: tuple[str, ...]
    label: str


WORD_PATCHES = (
    # UARTE0 active state: TXD P0.21, RXD P0.24 -> micro:bit DAPLink UART pins.
    # micro:bit V2 target P0.06 is DAPLink CDC RX from the interface side, so it is
    # target TX. Target P1.08 is DAPLink CDC TX from the interface side, so it is RX.
    (0x4E7DC, 0x00008015, 0x00008006, "UARTE0 active TXD P0.21 -> P0.06"),
    (0x4E7E0, 0x01008018, 0x01008028, "UARTE0 active RXD P0.24 -> P1.08"),
    # UARTE0 sleep state uses the same pins without the active-state flag.
    (0x4E7E4, 0x00000015, 0x00000006, "UARTE0 sleep TXD P0.21 -> P0.06"),
    (0x4E7E8, 0x01000018, 0x01000028, "UARTE0 sleep RXD P0.24 -> P1.08"),
    # Button table entries are logical input id, GPIO device pointer, packed
    # flags/pin, and label pointer. Stock A/B already use the nRF GPIO0 device
    # (gpio@50000000); keep their active-low/pull-up flags while retargeting
    # them to the micro:bit front buttons: Button A is P0.14, Button B is P0.23.
    (0x4ED48, 0x00110017, 0x0011000E, "button A pin P1.23 -> micro:bit P0.14"),
    (0x4ED58, 0x0011000A, 0x00110017, "button B pin P1.10 -> micro:bit P0.23"),
    # This pinctrl state otherwise reconfigures P0.14 after boot, overriding
    # Button A's pull-up input setup. Keep it as a pulled-up GPIO input.
    (0x4E2DC, 0x0C00800E, 0x0C00060E, "pinctrl P0.14 active -> pulled-up input"),
    (0x4E2E4, 0x0C00000E, 0x0C00060E, "pinctrl P0.14 sleep -> pulled-up input"),
)

THUMB_PATCHES = (
    # The stock controller waits for board/battery state that the micro:bit cannot
    # provide. Let the initial wireless startup path see "board ready" and a
    # non-low battery so it can reach transport init.
    ThumbPatch(
        0x1D620,
        ("ldrb.w r3, [r4, #0x4d]",),
        ("movs r3, #1", "nop"),
        "initial state: force board ready",
    ),
    ThumbPatch(
        0x1D626,
        ("ldrb.w r3, [r4, #0x4e]",),
        ("movs r3, #3", "nop"),
        "initial state: force battery status valid",
    ),
    ThumbPatch(
        0x1D656,
        ("ldrb.w r3, [r4, #0x4e]",),
        ("movs r3, #3", "nop"),
        "initial state: force non-low battery",
    ),
    ThumbPatch(
        0x1CDA2,
        ("ldrb.w r3, [r4, #0x4e]",),
        ("movs r3, #3", "nop"),
        "battery state: force non-low battery",
    ),
    # A nonzero user/wireless_transport selects the descriptor containing the
    # Bluetooth no-bond/advertising path.
    ThumbPatch(
        0x1D696,
        ("ldrb r0, [r6, #0]",),
        ("movs r0, #1",),
        "initial state: force BT transport for settings notify",
    ),
    ThumbPatch(
        0x1D69C,
        ("ldrb r0, [r6, #0]",),
        ("movs r0, #1",),
        "initial state: force BT transport descriptor",
    ),
    ThumbPatch(
        0x2016E,
        ("ldr r0, [pc, #0x20]", "pop.w {r3, lr}", "b.w 0x3d5ac"),
        ("movs r0, #0", "pop.w {r3, lr}", "b.w 0x1fe2c"),
        "BT no-bond path: enter advertising setup path",
    ),
    ThumbPatch(
        0x1E574,
        ("push {r3, r4, r7, lr}", "add r7, sp, #0"),
        ("movs r0, #0", "bx lr"),
        "HID ID_TURN_OFF: ignore host shutdown request",
    ),
)

STATE_TABLE_PATCHES = (
    # If any remaining board-specific path asks for low-battery shutdown, keep the
    # firmware in the normal battery-mode handlers instead.
    (0x4D544, 0x0001D14D, 0x0001CAC1, "state table: low-battery entry -> battery entry"),
    (0x4D548, 0x0001CE8D, 0x0001CD39, "state table: low-battery run -> battery run"),
    (0x4D54C, 0x0004192D, 0x00041925, "state table: low-battery exit -> battery exit"),
)


def asm_text(instructions: tuple[str, ...]) -> str:
    return "; ".join(instructions)


def assemble_thumb(
    ks: Ks, addr: int, instructions: tuple[str, ...], label: str, role: str
) -> bytes:
    text = asm_text(instructions)
    try:
        encoded, _ = ks.asm(text, addr=addr)
        assert encoded is not None
    except (KsError, AssertionError) as exc:
        raise SystemExit(f"{label}: failed to assemble {role} at 0x{addr:08x}: {text}: {exc}")
    return bytes(encoded)


def patch_word(
    data: bytearray, payload_base: int, addr: int, expected: int, new: int, label: str
) -> None:
    offset = addr - payload_base
    if offset < 0 or offset + 4 > len(data):
        raise SystemExit(f"{label}: address 0x{addr:08x} outside payload")
    actual = struct.unpack_from("<I", data, offset)[0]
    if actual != expected:
        raise SystemExit(
            f"{label}: expected 0x{expected:08x} at 0x{addr:08x}, found 0x{actual:08x}"
        )
    struct.pack_into("<I", data, offset, new)
    print(f"{label}: 0x{addr:08x}: 0x{expected:08x} -> 0x{new:08x}")


def patch_bytes(
    data: bytearray, payload_base: int, addr: int, expected: bytes, new: bytes, label: str
) -> None:
    offset = addr - payload_base
    if len(expected) != len(new):
        raise SystemExit(f"{label}: expected and replacement lengths differ")
    if offset < 0 or offset + len(expected) > len(data):
        raise SystemExit(f"{label}: address 0x{addr:08x} outside payload")
    actual = bytes(data[offset : offset + len(expected)])
    if actual != expected:
        raise SystemExit(
            f"{label}: expected {expected.hex()} at 0x{addr:08x}, found {actual.hex()}"
        )
    data[offset : offset + len(new)] = new
    print(f"{label}: 0x{addr:08x}: {expected.hex()} -> {new.hex()}")


def patch_thumb(data: bytearray, payload_base: int, ks: Ks, patch: ThumbPatch) -> None:
    expected = assemble_thumb(ks, patch.addr, patch.expected, patch.label, "expected")
    replacement = assemble_thumb(ks, patch.addr, patch.replacement, patch.label, "replacement")
    patch_bytes(data, payload_base, patch.addr, expected, replacement, patch.label)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("input", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--payload-base", type=lambda value: int(value, 0), default=0x8000)
    args = parser.parse_args()

    ks = Ks(KS_ARCH_ARM, KS_MODE_THUMB | KS_MODE_LITTLE_ENDIAN)
    data = bytearray(args.input.read_bytes())
    for patch in WORD_PATCHES:
        patch_word(data, args.payload_base, *patch)
    for patch in STATE_TABLE_PATCHES:
        patch_word(data, args.payload_base, *patch)
    for patch in THUMB_PATCHES:
        patch_thumb(data, args.payload_base, ks, patch)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_bytes(data)
    print(f"wrote {args.output}")


if __name__ == "__main__":
    main()
