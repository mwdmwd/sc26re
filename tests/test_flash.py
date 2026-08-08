#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later

import argparse
import struct
import tempfile
import unittest
import zlib
from pathlib import Path
from unittest import mock

import flash


def firmware_container(fw_class: str, payload: bytes) -> bytes:
    header = struct.pack(
        "<8I",
        flash.CLASS_MAGIC[fw_class],
        len(payload),
        zlib.crc32(payload) & 0xFFFFFFFF,
        0,
        0,
        0,
        0,
        0,
    )
    return header + payload


class FirmwareImageTests(unittest.TestCase):
    def test_empty_payload_is_rejected(self) -> None:
        with self.assertRaises(flash.ToolError):
            flash.FirmwareImage.parse(Path("empty.fw"), firmware_container("ibex", b""))

    def test_valid_container_is_accepted(self) -> None:
        payload = bytes(64)

        image = flash.FirmwareImage.parse(Path("valid.fw"), firmware_container("ibex", payload))

        self.assertEqual(image.payload, payload)

    def test_unknown_bootloader_class_is_rejected_before_loading_or_erasing(self) -> None:
        args = argparse.Namespace(
            serial=None,
            port="/dev/ttyFAKE",
            target="ibex",
            firmware="firmware.fw",
            verbose=False,
        )

        with (
            mock.patch.object(
                flash,
                "resolve_bootloader_selection",
                return_value=(args.port, None, None),
            ),
            mock.patch.object(flash.FirmwareImage, "load") as load,
            mock.patch.object(flash, "flash_bootloader_port") as program,
        ):
            with self.assertRaises(flash.ToolError):
                flash.cmd_bl_flash(args)

        load.assert_not_called()
        program.assert_not_called()

    def test_wrong_firmware_class_is_rejected_before_erasing(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "proteus.fw"
            path.write_bytes(firmware_container("proteus", bytes(64)))
            args = argparse.Namespace(
                serial=None,
                port="/dev/ttyFAKE",
                target=None,
                firmware=str(path),
                verbose=False,
            )

            with (
                mock.patch.object(
                    flash,
                    "resolve_bootloader_selection",
                    return_value=(args.port, "ibex", None),
                ),
                mock.patch.object(flash, "flash_bootloader_port") as program,
            ):
                with self.assertRaises(flash.ToolError):
                    flash.cmd_bl_flash(args)

        program.assert_not_called()


if __name__ == "__main__":
    unittest.main()
