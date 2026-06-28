#!/usr/bin/env python3
# SPDX-License-Identifier: AGPL-3.0-or-later
import argparse
import ctypes
import json
import os
import struct
import sys
import time
import zlib
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from collections.abc import Iterable
from typing import Any, Self

if sys.platform == "darwin":
    try:
        import hid

        hid.hidapi.hid_darwin_set_open_exclusive(0)
    except Exception:
        try:
            ctypes.CDLL("libhidapi.dylib").hid_darwin_set_open_exclusive(0)
        except Exception:
            pass

VALVE_VID = 0x28DE

PID_TRITON_BL = 0x1005
PID_PROTEUS_BL = 0x1007
PID_TRITON_USB = 0x1302
PID_TRITON_BLE = 0x1303
PID_PROTEUS_USB = 0x1304
PID_NEREID_USB = 0x1305

HID_FEATURE_REPORT_SIZE = 64

FW_MAGIC = {
    0xD2D86467: "ibex",
    0x2E795631: "proteus",
}
CLASS_MAGIC = {v: k for k, v in FW_MAGIC.items()}

DEFAULT_CONFIG = "hardwareupdater.cfg"
DEFAULT_MIN_HW_ID = 68

ESCAPE_BYTE = 0xAC
SOF_BYTE = 0xAD
EOF_BYTE = 0xAE

MESSAGE_INFO = 0x1233
MESSAGE_FW_BEGIN = 0x1234
MESSAGE_FW_DATA = 0x1235
MESSAGE_FW_END = 0x1236
MESSAGE_RESET = 0x1237
FW_DATA_CHUNK_SIZE = 0x7FFC
MAX_IGNORED_BOOTLOADER_FRAMES = 16
FW_BEGIN_ATTEMPTS = 3
FW_BEGIN_RETRY_DELAY_SECONDS = 2
MESSAGE_UICR_PROVISION = 0x1238
UICR_PROVISION_KEY = 0xE86DA4C7
UICR_CUSTOMER_SIZE = 128
COMMON_PROVISIONING_MAGIC = 0xAC32A429
PROVISIONING_MAGICS_FOR_CLASS = {
    "ibex": (COMMON_PROVISIONING_MAGIC,),
    "proteus": (COMMON_PROVISIONING_MAGIC, 0xAC388E29),
}

TYPE_TRITON_BL = 0
TYPE_PROTEUS_BL = 1
TYPE_TRITON_USB = 2
TYPE_TRITON_BLE = 3
TYPE_TRITON_ESB = 4
TYPE_PROTEUS_USB = 5
TYPE_NEREID_USB = 6

DEVICE_TYPES = {
    TYPE_TRITON_BL: ("Triton BL", "ibex", "serial"),
    TYPE_PROTEUS_BL: ("Proteus BL", "proteus", "serial"),
    TYPE_TRITON_USB: ("Triton USB", "ibex", "hid"),
    TYPE_TRITON_BLE: ("Triton BLE", "ibex", "hid"),
    TYPE_TRITON_ESB: ("Triton ESB", "ibex", "esb"),
    TYPE_PROTEUS_USB: ("Proteus USB", "proteus", "hid"),
    TYPE_NEREID_USB: ("Nereid USB", "proteus", "hid"),
}

HID_PIDS = {
    PID_TRITON_USB: TYPE_TRITON_USB,
    PID_TRITON_BLE: TYPE_TRITON_BLE,
    PID_PROTEUS_USB: TYPE_PROTEUS_USB,
    PID_NEREID_USB: TYPE_NEREID_USB,
}

NUMERIC_TAGS = {
    0: "unique_id",
    1: "product_id",
    2: "capabilities",
    4: "build_timestamp",
    5: "radio_build_timestamp",
    9: "hw_id",
    10: "boot_build_timestamp",
    11: "frame_rate",
    12: "secondary_build_timestamp",
    13: "secondary_boot_build_timestamp",
    14: "secondary_hw_id",
    15: "data_streaming",
    16: "trackpad_id",
    17: "secondary_trackpad_id",
}

NOT_PROVISIONED_TEXT = "Not Provisioned"
CFW_NVS_INFO_PATH = "nvs/info"
CFW_NVS_ERASE_PATH = "nvs/erase"
CFW_NVS_PATH_PREFIX = "nvs/"
CFW_NVS_REBOOT_DELAY_SECONDS = 2
CFW_NVS_RESTORE_HID_TIMEOUT_SECONDS = 12
CFW_NVS_READ_TIMEOUT_SECONDS = 1.5
CFW_NVS_READ_ATTEMPTS = 4
CFW_NVS_READ_RETRY_DELAY_SECONDS = 0.15


class ToolError(Exception):
    pass


class BootloaderReadError(ToolError):
    pass


class BootloaderTimeoutError(BootloaderReadError):
    def __init__(self, partial: bytes | bytearray = b"", message: str | None = None):
        self.partial = bytes(partial)
        if message is None:
            if self.partial:
                message = f"ERROR: Timed out reading bootloader frame {self.partial.hex()}"
            else:
                message = "ERROR: Timed out reading bootloader response"
        super().__init__(message)


class BootloaderSerialReadError(BootloaderReadError):
    pass


class BootloaderFrameError(ToolError):
    def __init__(self, raw: bytes | bytearray):
        self.raw = bytes(raw)
        super().__init__(f"ERROR: Invalid message {self.raw.hex()}")


class HidResponseError(ToolError):
    pass


def out(*args: Any) -> None:
    print(*args, flush=True)


def hex_opt(value: int | None) -> str | None:
    if value is None:
        return None
    return f"0x{value:08X}"


def human_ts(value: int | None) -> str:
    if value is None:
        return "unknown"
    return f"0x{value:08X}"


def timestamp_datetime(value: int | None) -> str | None:
    if value is None:
        return None
    try:
        return datetime.fromtimestamp(value, timezone.utc).isoformat()
    except (OverflowError, OSError, ValueError):
        return None


def provisioned_text(value: str | None) -> str:
    return value if value else NOT_PROVISIONED_TEXT


@dataclass
class FirmwareImage:
    path: Path
    magic: int
    fw_class: str
    payload_len: int
    crc32: int
    header: bytes
    payload: bytes
    reserved: bytes

    @classmethod
    def load(
        cls,
        path: Path,
        *,
        target: str | None = None,
        device_class: str | None = None,
        verbose: bool = False,
    ) -> Self:
        try:
            data = path.read_bytes()
        except OSError as exc:
            raise ToolError("ERROR: INVALID FIRMWARE FILE") from exc
        try:
            fw = cls.parse(path, data)
        except ToolError:
            raise
        except Exception as exc:
            raise ToolError("ERROR: INVALID FIRMWARE FILE") from exc

        if target and fw.magic != CLASS_MAGIC[target]:
            raise ToolError(
                f"ERROR: Firmware magic 0x{fw.magic:08X} does not match target {target}"
            )
        if device_class and fw.fw_class != device_class:
            raise ToolError(
                f"ERROR: Firmware class {fw.fw_class} does not match device class {device_class}"
            )
        if verbose and any(fw.reserved):
            out(f"WARNING: Firmware reserved bytes are nonzero: {fw.reserved.hex()}")
        return fw

    @classmethod
    def parse(cls, path: Path, data: bytes) -> Self:
        if len(data) < 32:
            raise ToolError("ERROR: INVALID FIRMWARE FILE")
        header = data[:32]
        magic, payload_len, expected_crc = struct.unpack_from("<III", header)
        payload = data[32:]
        fw_class = FW_MAGIC.get(magic)
        if fw_class is None:
            raise ToolError("ERROR: INVALID FIRMWARE FILE")
        if payload_len != len(payload):
            raise ToolError("ERROR: INVALID FIRMWARE FILE")
        actual_crc = zlib.crc32(payload) & 0xFFFFFFFF
        if actual_crc != expected_crc:
            raise ToolError("ERROR: INVALID FIRMWARE FILE")
        return cls(
            path=path,
            magic=magic,
            fw_class=fw_class,
            payload_len=payload_len,
            crc32=expected_crc,
            header=header,
            payload=payload,
            reserved=header[12:32],
        )


@dataclass
class Config:
    path: Path
    values: dict[str, int]

    @classmethod
    def load(cls, path: Path) -> Self:
        try:
            text = path.read_text(encoding="utf-8")
        except OSError as exc:
            raise ToolError(f"ERROR: Could not read config file: {path}") from exc
        values: dict[str, int] = {}
        for raw in text.splitlines():
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            try:
                values[key.strip()] = int(value, 16)
            except ValueError as exc:
                raise ToolError("ERROR: Did not find complete data in config file") from exc
        required = {
            "TRITON_FW_TS",
            "PROTEUS_FW_TS",
            "MUST_UPDATE_TRITON_FW_TS",
            "MUST_UPDATE_PROTEUS_FW_TS",
        }
        if not required.issubset(values):
            raise ToolError("ERROR: Did not find complete data in config file")
        return cls(path=path, values=values)

    def target_ts(self, fw_class: str) -> int:
        key = "TRITON_FW_TS" if fw_class == "ibex" else "PROTEUS_FW_TS"
        return self.values[key]

    def mandatory_ts(self, fw_class: str) -> int:
        key = "MUST_UPDATE_TRITON_FW_TS" if fw_class == "ibex" else "MUST_UPDATE_PROTEUS_FW_TS"
        return self.values[key]

    def firmware_path(self, fw_class: str) -> Path:
        if fw_class == "ibex":
            return self.path.parent / f"IBEX_FW_{self.values['TRITON_FW_TS']:08X}.fw"
        return self.path.parent / f"PROTEUS_FW_{self.values['PROTEUS_FW_TS']:08X}.fw"


@dataclass
class Device:
    type_id: int
    name: str
    fw_class: str
    transport: str
    serial_number: str | None
    hardware_id: int = 0
    current_ts: int | None = None
    bootloader_port: str | None = None
    hid_path: bytes | str | None = None
    bcd_version: int = 0
    updateable_connection: bool = True
    attributes: dict[str, int] = field(default_factory=dict)
    raw_info: bytes | None = None
    pcba_serial: str = "None"

    @classmethod
    def from_type(cls, type_id: int, serial_number: str | None, **kwargs: Any) -> Self:
        name, fw_class, transport = DEVICE_TYPES[type_id]
        updateable = type_id not in (TYPE_TRITON_BLE, TYPE_TRITON_ESB)
        return cls(
            type_id=type_id,
            name=name,
            fw_class=fw_class,
            transport=transport,
            serial_number=serial_number,
            updateable_connection=updateable,
            **kwargs,
        )

    def is_updateable(self, min_hw_id: int, apply_hw_gate: bool = True) -> bool:
        if not self.updateable_connection:
            return False
        if apply_hw_gate and self.fw_class == "ibex" and self.hardware_id < min_hw_id:
            return False
        return True


def parse_hid_string_payload(payload: bytes) -> str | None:
    if len(payload) < 2 or payload[1] == 0xFF:
        return None
    string_payload = payload[1:]
    if b"\x00" not in string_payload:
        return None
    raw = string_payload.split(b"\x00", 1)[0]
    if not raw:
        return None
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        return None
    return text or None


def parse_numeric_payload(payload: bytes, *, strict: bool = False) -> dict[str, int]:
    if strict and len(payload) % 5:
        raise ToolError("ERROR: Malformed HID numeric response")
    attrs: dict[str, int] = {}
    usable = len(payload) - (len(payload) % 5)
    for offset in range(0, usable, 5):
        tag = payload[offset]
        value = struct.unpack_from("<I", payload, offset + 1)[0]
        name = NUMERIC_TAGS.get(tag, f"tag_{tag}")
        attrs[name] = value
    return attrs


def hid_report_selection(pid: int, bcd_version: int) -> tuple[int, int, int, int]:
    if pid in (PID_TRITON_USB, PID_TRITON_BLE):
        return 0xAE, 0x01, 0x83, 0x01
    if pid in (PID_PROTEUS_USB, PID_NEREID_USB) and bcd_version == 2:
        return 0xAE, 0x02, 0x83, 0x02
    return 0xA4, 0x01, 0xA6, 0x01


def parse_hid_response(response: bytes | list[int], expected_opcode: int) -> bytes:
    data = bytes(response)
    if len(data) < 3:
        raise HidResponseError("ERROR: Malformed HID response")
    if data[1] != expected_opcode:
        raise HidResponseError("ERROR: Malformed HID response")
    payload_len = data[2]
    if 3 + payload_len > len(data):
        raise HidResponseError("ERROR: Malformed HID response")
    return data[3 : 3 + payload_len]


def open_hid_device(path: bytes | str | None) -> Any:
    # TODO after Python 3.15: search for `except ImportError` and use lazy import
    try:
        import hid
    except ImportError as exc:
        raise ToolError("ERROR: Missing hidapi Python module, install requirements.txt") from exc
    handle = hid.device()
    handle.open_path(path)
    return handle


def close_device(handle: Any) -> None:
    try:
        handle.close()
    except Exception:
        pass


def hid_exchange(
    handle: Any,
    report_id: int,
    opcode: int,
    payload: bytes = b"",
    *,
    expect_response: bool = True,
    timeout: float = 0.5,
) -> bytes | None:
    report = bytearray(HID_FEATURE_REPORT_SIZE)
    report[0] = report_id
    report[1] = opcode
    report[2 : 2 + len(payload)] = payload
    handle.send_feature_report(bytes(report))
    if not expect_response:
        return None
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            response = handle.get_feature_report(report_id, HID_FEATURE_REPORT_SIZE)
            if response:
                return parse_hid_response(response, opcode)
        except Exception as exc:
            last_error = exc
        time.sleep(0.025)
    if last_error:
        raise ToolError("ERROR: HID device did not respond") from last_error
    raise ToolError("ERROR: HID device did not respond")


def read_hid_string(handle: Any, opcode: int, report_id: int, attr: int) -> str | None:
    payload = hid_exchange(handle, report_id, opcode, bytes([0x01, attr]))
    if payload is None:
        raise HidResponseError("ERROR: Malformed HID response")
    return parse_hid_string_payload(payload)


def read_hid_numeric(
    handle: Any, opcode: int, report_id: int, *, strict: bool = False
) -> dict[str, int]:
    payload = hid_exchange(handle, report_id, opcode)
    if payload is None:
        raise HidResponseError("ERROR: Malformed HID response")
    return parse_numeric_payload(payload, strict=strict)


def feature_path_payload(path: str, value: bytes = b"") -> bytes:
    encoded = path.encode("utf-8") + b"\x00"
    body_len = len(encoded) + len(value)
    if body_len > 0xFF:
        raise ToolError(f"ERROR: Feature setting payload too large for {path}")
    return bytes([body_len]) + encoded + value


def read_feature_setting(
    handle: Any, report_id: int, path: str, *, timeout: float = 0.5
) -> bytes | None:
    payload = hid_exchange(handle, report_id, 0xED, feature_path_payload(path), timeout=timeout)
    if payload == b"\x00":
        return None
    return payload


def stage_feature_setting(handle: Any, report_id: int, path: str, value: bytes) -> None:
    hid_exchange(handle, report_id, 0xEE, feature_path_payload(path, value), expect_response=False)


def commit_feature_setting(handle: Any, report_id: int, path: str) -> None:
    hid_exchange(handle, report_id, 0xEF, feature_path_payload(path), expect_response=False)


def cfw_nvs_report_selection(device: Device) -> int:
    if device.type_id in (TYPE_TRITON_USB, TYPE_TRITON_BLE):
        _, string_report, _, _ = hid_report_selection(PID_TRITON_USB, device.bcd_version)
        return string_report
    if device.type_id == TYPE_PROTEUS_USB:
        _, string_report, _, _ = hid_report_selection(PID_PROTEUS_USB, device.bcd_version)
        return string_report
    if device.type_id == TYPE_NEREID_USB:
        _, string_report, _, _ = hid_report_selection(PID_NEREID_USB, device.bcd_version)
        return string_report
    raise ToolError(f"ERROR: Device does not expose CFW settings over HID: {device.name}")


@dataclass
class CfwNvsImage:
    data: bytes
    chunk_size: int


def cfw_nvs_read_info(handle: Any, report_id: int) -> tuple[int, int] | None:
    payload = read_feature_setting(
        handle, report_id, CFW_NVS_INFO_PATH, timeout=CFW_NVS_READ_TIMEOUT_SECONDS
    )
    if payload is None:
        return None
    if len(payload) != 8:
        raise ToolError(f"ERROR: Malformed CFW NVS info response: {payload.hex()}")
    size = struct.unpack_from("<I", payload, 0)[0]
    chunk_size = struct.unpack_from("<I", payload, 4)[0]
    if size == 0 or chunk_size == 0 or size % 4 or chunk_size % 4:
        raise ToolError(f"ERROR: Invalid CFW NVS geometry: size={size} chunk={chunk_size}")
    return size, chunk_size


def cfw_nvs_read_chunk(handle: Any, report_id: int, path: str, offset: int) -> bytes:
    last_error: ToolError | None = None

    for attempt in range(1, CFW_NVS_READ_ATTEMPTS + 1):
        try:
            chunk = read_feature_setting(
                handle, report_id, path, timeout=CFW_NVS_READ_TIMEOUT_SECONDS
            )
            if chunk is None:
                raise ToolError(f"ERROR: CFW NVS read failed at offset 0x{offset:04x}")
            return chunk
        except ToolError as exc:
            last_error = exc
            if attempt == CFW_NVS_READ_ATTEMPTS:
                break
            out(
                f"WARNING: CFW NVS read offset 0x{offset:04x} failed "
                f"({exc}); retrying ({attempt + 1}/{CFW_NVS_READ_ATTEMPTS})"
            )
            time.sleep(CFW_NVS_READ_RETRY_DELAY_SECONDS)

    raise ToolError(f"ERROR: CFW NVS read failed at offset 0x{offset:04x}: {last_error}")


def wait_for_hid_device(serial: str, *, timeout: float, verbose: bool = False) -> Device:
    deadline = time.monotonic() + timeout

    while time.monotonic() < deadline:
        for dev in enumerate_hid_devices(verbose=verbose):
            if dev.serial_number == serial and dev.updateable_connection:
                return dev
        time.sleep(0.5)

    raise ToolError(f"ERROR: HID device did not return after update: {serial}")


def cfw_nvs_read_image(handle: Any, report_id: int, *, verbose: bool = False) -> CfwNvsImage | None:
    info = cfw_nvs_read_info(handle, report_id)
    if info is None:
        return None

    size, chunk_size = info
    image = bytearray()
    for offset in range(0, size, chunk_size):
        path = f"{CFW_NVS_PATH_PREFIX}{offset:04x}"
        chunk = cfw_nvs_read_chunk(handle, report_id, path, offset)
        image.extend(chunk)
        if verbose:
            percent = int((len(image) * 100) / size)
            out(f"NVS SAVE: {percent}%")

    del image[size:]
    return CfwNvsImage(bytes(image), chunk_size)


def cfw_nvs_write_image(
    handle: Any, report_id: int, image: CfwNvsImage, *, verbose: bool = False
) -> None:
    info = cfw_nvs_read_info(handle, report_id)
    if info is None:
        raise ToolError("ERROR: Device does not expose CFW NVS restore support")

    size, chunk_size = info
    if len(image.data) != size:
        raise ToolError(
            f"ERROR: CFW NVS image size {len(image.data)} does not match device size {size}"
        )
    if image.chunk_size and image.chunk_size != chunk_size:
        raise ToolError(
            f"ERROR: CFW NVS image chunk size {image.chunk_size} does not match device chunk {chunk_size}"
        )

    commit_feature_setting(handle, report_id, CFW_NVS_ERASE_PATH)
    time.sleep(0.1)
    for offset in range(0, size, chunk_size):
        chunk = image.data[offset : offset + chunk_size]
        path = f"{CFW_NVS_PATH_PREFIX}{offset:04x}"
        stage_feature_setting(handle, report_id, path, chunk)
        if verbose:
            percent = int(((offset + len(chunk)) * 100) / size)
            out(f"NVS RESTORE: {percent}%")

    restored = cfw_nvs_read_image(handle, report_id)
    if restored is None:
        raise ToolError("ERROR: Device does not expose CFW NVS verification support")
    if restored.data != image.data:
        mismatch = next(
            (
                index
                for index, (expected, actual) in enumerate(zip(image.data, restored.data))
                if expected != actual
            ),
            min(len(image.data), len(restored.data)),
        )
        raise ToolError(f"ERROR: CFW NVS restore verification failed at offset 0x{mismatch:04x}")
    if verbose:
        out("NVS VERIFY: 100%")


def save_cfw_nvs_from_device(device: Device, *, verbose: bool = False) -> CfwNvsImage | None:
    handle = None
    try:
        handle = open_hid_device(device.hid_path)
        return cfw_nvs_read_image(handle, cfw_nvs_report_selection(device), verbose=verbose)
    finally:
        if handle is not None:
            close_device(handle)


def restore_cfw_nvs_to_device(
    device: Device, image: CfwNvsImage, *, verbose: bool = False, reboot: bool = True
) -> None:
    handle = None
    try:
        handle = open_hid_device(device.hid_path)
        report_id = cfw_nvs_report_selection(device)
        cfw_nvs_write_image(handle, report_id, image, verbose=verbose)
        if reboot:
            hid_exchange(handle, report_id, 0x95, expect_response=False)
    finally:
        if handle is not None:
            close_device(handle)


def try_save_cfw_nvs_from_device(device: Device, *, verbose: bool = False) -> CfwNvsImage | None:
    try:
        image = save_cfw_nvs_from_device(device, verbose=verbose)
    except ToolError as exc:
        out(f"WARNING: Could not save CFW settings before update: {exc}")
        return None

    if image is not None:
        out(f"SAVED CFW SETTINGS: {len(image.data)} bytes")
    return image


def enumerate_hid_devices(*, info_only: bool = False, verbose: bool = False) -> list[Device]:
    try:
        import hid
    except ImportError as exc:
        raise ToolError("ERROR: Missing hidapi Python module, install requirements.txt") from exc
    devices: list[Device] = []
    seen_dongles: set[str | None] = set()
    for pid, type_id in HID_PIDS.items():
        try:
            entries = hid.enumerate(VALVE_VID, pid)
        except Exception:
            continue
        device_paths: dict[tuple[str, str | bytes], list[bytes | str]] = {}
        device_bcd: dict[tuple[str, str | bytes], int] = {}
        device_serial: dict[tuple[str, str | bytes], str | None] = {}
        for entry in entries:
            if int(entry.get("usage_page") or 0) < 0xFF00:
                continue
            path = entry.get("path")
            if path is None:
                continue
            raw_serial = entry.get("serial_number")
            serial = str(raw_serial) if raw_serial else None
            bcd_version = int(entry.get("release_number") or 0)
            key = ("serial", serial) if serial is not None else ("path", path)
            if key not in device_paths:
                device_paths[key] = []
                device_bcd[key] = bcd_version
                device_serial[key] = serial
            if path not in device_paths[key]:
                device_paths[key].append(path)
        for key, paths in device_paths.items():
            serial = device_serial[key]
            bcd_version = device_bcd[key]
            string_opcode, string_report, numeric_opcode, numeric_report = hid_report_selection(
                pid, bcd_version
            )
            serial_from_device: str | None = None
            attrs: dict[str, int] | None = None
            query_path: bytes | str | None = None
            for path in paths:
                handle = None
                try:
                    handle = open_hid_device(path)
                    if serial_from_device is None:
                        try:
                            serial_from_device = read_hid_string(
                                handle, string_opcode, string_report, 1
                            )
                        except Exception:
                            pass
                    if attrs is None:
                        try:
                            attrs = read_hid_numeric(
                                handle, numeric_opcode, numeric_report, strict=info_only
                            )
                            query_path = path
                        except Exception:
                            pass
                except Exception:
                    continue
                finally:
                    if handle is not None:
                        close_device(handle)
                if attrs is not None:
                    break
            if attrs is None:
                if info_only:
                    raise ToolError("Could not query device")
                continue
            resolved_serial = serial_from_device or serial
            if type_id in (TYPE_PROTEUS_USB, TYPE_NEREID_USB):
                if resolved_serial in seen_dongles:
                    continue
                seen_dongles.add(resolved_serial)
            dev = Device.from_type(
                type_id,
                resolved_serial,
                hardware_id=attrs.get("hw_id", 0),
                current_ts=attrs.get("build_timestamp"),
                hid_path=query_path,
                bcd_version=bcd_version,
                attributes=attrs,
            )
            devices.append(dev)
            if type_id in (TYPE_PROTEUS_USB, TYPE_NEREID_USB):
                paired_serial: str | None = None
                paired_hw_id: int = 0
                paired_ts: int | None = None
                paired_path: bytes | str | None = None
                paired_found = False
                for path in paths:
                    result = read_paired_esb_controller(path)
                    if result is not None:
                        paired_serial, controller_attrs = result
                        if controller_attrs is not None:
                            paired_hw_id = controller_attrs.get("hw_id", 0)
                            paired_ts = controller_attrs.get("build_timestamp")
                        paired_path = path
                        paired_found = True
                        break
                if paired_found:
                    devices.append(
                        Device.from_type(
                            TYPE_TRITON_ESB,
                            paired_serial,
                            hardware_id=paired_hw_id,
                            current_ts=paired_ts,
                            hid_path=paired_path,
                        )
                    )
    return devices


def read_paired_esb_serial(path: bytes | str) -> str | None:
    result = read_paired_esb_controller(path)
    return result[0] if result is not None else None


def read_paired_esb_controller(
    path: bytes | str,
) -> tuple[str | None, dict[str, int] | None] | None:
    handle = None
    try:
        handle = open_hid_device(path)
        serial: str | None = None
        attrs: dict[str, int] | None = None
        try:
            serial = read_hid_string(handle, 0xAE, 0x01, 1)
        except Exception:
            pass
        try:
            candidate = read_hid_numeric(handle, 0x83, 0x01)
            if candidate.get("product_id") == PID_TRITON_USB:
                attrs = candidate
        except Exception:
            pass
        if serial is not None or attrs is not None:
            return (serial, attrs)
    except Exception:
        pass
    finally:
        if handle is not None:
            close_device(handle)
    return None


def send_hid_reboot_to_bootloader(device: Device) -> None:
    handle = None
    try:
        handle = open_hid_device(device.hid_path)
        report_id = 0x02 if device.fw_class == "proteus" and device.bcd_version == 2 else 0x01
        hid_exchange(handle, report_id, 0x90, expect_response=False)
    finally:
        if handle is not None:
            close_device(handle)


def send_hid_normal_reboot(device: Device) -> None:
    handle = None
    try:
        handle = open_hid_device(device.hid_path)
        hid_exchange(handle, 0x01, 0x95, expect_response=False)
    finally:
        if handle is not None:
            close_device(handle)


def encode_frame(payload: bytes) -> bytes:
    encoded = bytearray([SOF_BYTE])
    for byte in payload:
        if byte == ESCAPE_BYTE:
            encoded.extend((ESCAPE_BYTE, 0x00))
        elif byte == SOF_BYTE:
            encoded.extend((ESCAPE_BYTE, 0x01))
        elif byte == EOF_BYTE:
            encoded.extend((ESCAPE_BYTE, 0x02))
        else:
            encoded.append(byte)
    encoded.append(EOF_BYTE)
    return bytes(encoded)


def decode_frame_bytes(frame: bytes) -> bytes:
    raw = bytes(frame)
    try:
        start = raw.index(bytes([SOF_BYTE]))
    except ValueError as exc:
        raise BootloaderFrameError(raw) from exc
    if any(byte == EOF_BYTE for byte in raw[:start]):
        raise BootloaderFrameError(raw)
    try:
        end = raw.index(bytes([EOF_BYTE]), start + 1)
    except ValueError as exc:
        raise BootloaderFrameError(raw) from exc
    body = raw[start + 1 : end]
    decoded = bytearray()
    idx = 0
    while idx < len(body):
        byte = body[idx]
        if byte != ESCAPE_BYTE:
            decoded.append(byte)
            idx += 1
            continue
        if idx + 1 >= len(body):
            raise BootloaderFrameError(raw)
        esc = body[idx + 1]
        if esc == 0x00:
            decoded.append(ESCAPE_BYTE)
        elif esc == 0x01:
            decoded.append(SOF_BYTE)
        elif esc == 0x02:
            decoded.append(EOF_BYTE)
        else:
            raise BootloaderFrameError(raw)
        idx += 2
    return bytes(decoded)


def read_frame(port: Any) -> tuple[bytes, bytes]:
    try:
        import serial
    except ImportError as exc:
        raise ToolError("ERROR: Missing pyserial Python module, install requirements.txt") from exc
    raw = bytearray()
    in_frame = False
    while True:
        try:
            chunk = port.read(1)
        except serial.SerialException as exc:
            raise BootloaderSerialReadError(f"ERROR: Bootloader serial read failed: {exc}") from exc
        if not chunk:
            raise BootloaderTimeoutError(raw)
        byte = chunk[0]
        if not in_frame:
            if byte == SOF_BYTE:
                raw.append(byte)
                in_frame = True
            continue
        if byte == SOF_BYTE:
            raw = bytearray([byte])
            continue
        raw.append(byte)
        if byte == EOF_BYTE:
            return bytes(raw), decode_frame_bytes(bytes(raw))


def check_ack(response: bytes) -> bytes:
    if not response or response[0] != 0:
        raise ToolError(f"ERROR: Invalid response {response.hex()}")
    return response[1:]


def bootloader_command(
    port: Any, message_id: int, body: bytes = b"", *, verbose: bool = False
) -> bytes:
    payload = struct.pack("<H", message_id) + body
    tx = encode_frame(payload)
    if verbose:
        out(f"BL TX 0x{message_id:04X}: payload={len(payload)} encoded={len(tx)}")
    port.write(tx)

    ignored = 0
    while True:
        try:
            raw, response = read_frame(port)
        except BootloaderTimeoutError as exc:
            if exc.partial:
                message = (
                    f"ERROR: Timed out reading bootloader frame for 0x{message_id:04X}: "
                    f"{exc.partial.hex()}"
                )
            else:
                message = f"ERROR: Timed out reading bootloader response for 0x{message_id:04X}"
            raise BootloaderTimeoutError(exc.partial, message) from exc
        except BootloaderSerialReadError as exc:
            raise BootloaderSerialReadError(
                f"ERROR: Bootloader serial read failed for 0x{message_id:04X}: {exc}"
            ) from exc
        except BootloaderFrameError as exc:
            ignored += 1
            if verbose:
                out(
                    f"BL RX 0x{message_id:04X}: ignored malformed frame "
                    f"{exc.raw[:32].hex()}{'...' if len(exc.raw) > 32 else ''}"
                )
            if ignored >= MAX_IGNORED_BOOTLOADER_FRAMES:
                raise
            continue
        if verbose:
            preview = response[:32].hex()
            suffix = "..." if len(response) > 32 else ""
            out(
                f"BL RX 0x{message_id:04X}: raw={len(raw)} decoded={len(response)} "
                f"data={preview}{suffix}"
            )

        if response == b"":
            if verbose:
                out(f"BL RX 0x{message_id:04X}: ignored empty frame")
            continue

        if response[0] != 0 and len(response) > 1:
            ignored += 1
            if verbose:
                out(
                    f"BL RX 0x{message_id:04X}: ignored unexpected non-ACK frame "
                    f"{response[:32].hex()}{'...' if len(response) > 32 else ''}"
                )
            if ignored >= MAX_IGNORED_BOOTLOADER_FRAMES:
                raise ToolError(f"ERROR: Invalid response {response.hex()}")
            continue

        return check_ack(response)


@dataclass
class BootInfo:
    port: str
    fw_class: str
    hw_id: int
    unit_serial: str
    pcba_serial: str
    raw_response: bytes


def decode_serial_field(data: bytes) -> str:
    raw = data.split(b"\x00", 1)[0]
    if not raw:
        return "None"
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError:
        return "None"
    return text or "None"


def parse_boot_info_response(port: str, fw_class: str, response: bytes) -> BootInfo:
    if len(response) != 164:
        raise ToolError(f"ERROR: BAD INFO RESPONSE: {response.hex()}")
    magic = struct.unpack_from("<I", response, 36)[0]
    if magic not in PROVISIONING_MAGICS_FOR_CLASS.get(fw_class, (COMMON_PROVISIONING_MAGIC,)):
        return BootInfo(port, fw_class, 0, "None", "None", response)
    hw_id = struct.unpack_from("<I", response, 40)[0]
    unit_serial = decode_serial_field(response[44:60])
    pcba_serial = decode_serial_field(response[60:76])
    return BootInfo(port, fw_class, hw_id, unit_serial, pcba_serial, response)


def detect_bootloader_class(port_info: Any) -> str | None:
    vid = getattr(port_info, "vid", None)
    pid = getattr(port_info, "pid", None)
    if vid == VALVE_VID and pid == PID_TRITON_BL:
        return "ibex"
    if vid == VALVE_VID and pid == PID_PROTEUS_BL:
        return "proteus"
    hwid = str(getattr(port_info, "hwid", ""))
    upper = hwid.upper()
    if "VID:PID=28DE:1005" in upper:
        return "ibex"
    if "VID:PID=28DE:1007" in upper:
        return "proteus"
    return None


def list_bootloader_ports(verbose: bool = False) -> list[tuple[str, str]]:
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise ToolError("ERROR: Missing pyserial Python module, install requirements.txt") from exc

    ports = list(list_ports.comports())
    if any(detect_bootloader_class(port) == "proteus" for port in ports):
        time.sleep(5)
        ports = list(list_ports.comports())
    result: list[tuple[str, str]] = []
    for port in ports:
        fw_class = detect_bootloader_class(port)
        if fw_class:
            result.append((str(port.device), fw_class))
    return result


def open_serial_port(port_name: str, *, verbose: bool = False) -> Any:
    try:
        import serial
    except ImportError as exc:
        raise ToolError("ERROR: Missing pyserial Python module, install requirements.txt") from exc
    last_exc: Exception | None = None
    for _ in range(10):
        try:
            return serial.Serial(port_name, timeout=60)
        except Exception as exc:
            last_exc = exc
            time.sleep(2)
    raise ToolError("ERROR: Couldn't open BL comport") from last_exc


def query_boot_info(port_name: str, fw_class: str, *, verbose: bool = False) -> BootInfo:
    port = open_serial_port(port_name, verbose=verbose)
    try:
        response = bootloader_command(port, MESSAGE_INFO, verbose=verbose)
        return parse_boot_info_response(port_name, fw_class, response)
    finally:
        try:
            port.close()
        except Exception:
            pass


def close_serial_port(port: Any) -> None:
    try:
        port.close()
    except Exception:
        pass


def reset_serial_input(port: Any) -> None:
    try:
        port.reset_input_buffer()
    except Exception:
        pass


def bootloader_begin_firmware(port_name: str, port: Any, *, verbose: bool = False) -> Any:
    for attempt in range(1, FW_BEGIN_ATTEMPTS + 1):
        try:
            bootloader_command(port, MESSAGE_FW_BEGIN, verbose=verbose)
            return port
        except BootloaderReadError as exc:
            if attempt >= FW_BEGIN_ATTEMPTS:
                raise
            out(f"WARNING: {exc}; retrying erase " f"({attempt + 1}/{FW_BEGIN_ATTEMPTS})")
            close_serial_port(port)
            time.sleep(FW_BEGIN_RETRY_DELAY_SECONDS)
            port = open_serial_port(port_name, verbose=verbose)
            reset_serial_input(port)

    return port


def enumerate_bootloader_devices(*, verbose: bool = False) -> list[Device]:
    devices: list[Device] = []
    for port_name, fw_class in list_bootloader_ports(verbose):
        try:
            info = query_boot_info(port_name, fw_class, verbose=verbose)
        except ToolError:
            if verbose:
                out(f"WARNING: Could not query bootloader port {port_name}")
            continue
        type_id = TYPE_TRITON_BL if fw_class == "ibex" else TYPE_PROTEUS_BL
        dev = Device.from_type(
            type_id,
            info.unit_serial,
            hardware_id=info.hw_id,
            current_ts=None,
            bootloader_port=port_name,
            raw_info=info.raw_response,
            pcba_serial=info.pcba_serial,
        )
        devices.append(dev)
    return devices


def discover_devices(*, verbose: bool = False) -> list[Device]:
    return enumerate_bootloader_devices(verbose=verbose) + enumerate_hid_devices(verbose=verbose)


def flash_bootloader_port(
    port_name: str, firmware: FirmwareImage, *, verbose: bool = False
) -> None:
    port = open_serial_port(port_name, verbose=verbose)
    try:
        reset_serial_input(port)
        out(f"PROGRAMMING: {firmware.path}")
        out("ERASING")
        port = bootloader_begin_firmware(port_name, port, verbose=verbose)
        total = len(firmware.payload)
        sent = 0
        if total == 0:
            out("PROGRESS: 100%")
        # The bootloader parser accepts at most 0x8003 decoded bytes. Keep the
        # 2-byte message ID, 2-byte chunk length, and aligned data below that.
        for offset in range(0, total, FW_DATA_CHUNK_SIZE):
            chunk = firmware.payload[offset : offset + FW_DATA_CHUNK_SIZE]
            body = struct.pack("<H", len(chunk)) + chunk
            bootloader_command(port, MESSAGE_FW_DATA, body, verbose=verbose)
            sent += len(chunk)
            percent = int((sent * 100) / total)
            out(f"PROGRESS: {percent}%")
        bootloader_command(port, MESSAGE_FW_END, firmware.header, verbose=verbose)
        out("RESETTING")
        bootloader_command(port, MESSAGE_RESET, verbose=verbose)
        out("SUCCESS")
    finally:
        close_serial_port(port)


def load_config_for_args(args: argparse.Namespace, *, required: bool = False) -> Config | None:
    config_name = getattr(args, "config", None)
    if config_name:
        return Config.load(Path(config_name))
    default_path = Path(DEFAULT_CONFIG)
    if default_path.exists():
        return Config.load(default_path)
    if required:
        raise ToolError(f"ERROR: Could not read config file: {DEFAULT_CONFIG}")
    return None


def timestamp_sources(
    args: argparse.Namespace, config: Config | None
) -> tuple[dict[str, int], dict[str, int]]:
    targets: dict[str, int] = {}
    mandatory: dict[str, int] = {}
    if config:
        targets["ibex"] = config.target_ts("ibex")
        targets["proteus"] = config.target_ts("proteus")
        mandatory["ibex"] = config.mandatory_ts("ibex")
        mandatory["proteus"] = config.mandatory_ts("proteus")
    if getattr(args, "ibex_ts", None):
        try:
            targets["ibex"] = int(args.ibex_ts, 16)
        except ValueError as exc:
            raise ToolError(f"ERROR: Invalid --ibex-ts value: {args.ibex_ts}") from exc
    if getattr(args, "proteus_ts", None):
        try:
            targets["proteus"] = int(args.proteus_ts, 16)
        except ValueError as exc:
            raise ToolError(f"ERROR: Invalid --proteus-ts value: {args.proteus_ts}") from exc
    return targets, mandatory


def firmware_path_for_class(
    args: argparse.Namespace, config: Config | None, fw_class: str
) -> Path | None:
    explicit = getattr(args, f"{fw_class}_fw", None)
    if explicit:
        return Path(explicit)
    if config:
        return config.firmware_path(fw_class)
    return None


def require_firmware_for_class(
    args: argparse.Namespace,
    config: Config | None,
    fw_class: str,
    *,
    device_class: str | None = None,
) -> FirmwareImage:
    path = firmware_path_for_class(args, config, fw_class)
    if path is None or not path.exists():
        raise ToolError(f"ERROR: Missing firmware file for target {fw_class}")
    return FirmwareImage.load(
        path,
        target=fw_class,
        device_class=device_class,
        verbose=getattr(args, "verbose", False),
    )


def update_status(
    device: Device,
    targets: dict[str, int],
    mandatory: dict[str, int],
    *,
    min_hw_id: int,
    apply_hw_gate: bool,
) -> dict[str, Any]:
    target = targets.get(device.fw_class)
    required_min = mandatory.get(device.fw_class)
    must_update: bool | None
    if device.current_ts is None:
        must_update = True if target is not None else None
    elif required_min is not None:
        must_update = device.current_ts < required_min
    else:
        must_update = None
    return {
        "type": device.type_id,
        "Name": device.name,
        "class": device.fw_class,
        "transport": "serial" if device.transport == "serial" else device.transport,
        "port": device.bootloader_port,
        "hardware_id": device.hardware_id,
        "serial_number": provisioned_text(device.serial_number),
        "current_ts": hex_opt(device.current_ts),
        "update_ts": hex_opt(target),
        "must_update": must_update,
        "updateable": device.is_updateable(min_hw_id, apply_hw_gate=apply_hw_gate),
    }


def print_device_list(
    devices: Iterable[Device],
    targets: dict[str, int],
    mandatory: dict[str, int],
    args: argparse.Namespace,
    *,
    apply_hw_gate: bool,
) -> None:
    if getattr(args, "json_output", False):
        entries = [
            {
                k: v
                for k, v in update_status(
                    dev, targets, mandatory, min_hw_id=args.min_hw_id, apply_hw_gate=apply_hw_gate
                ).items()
                if v is not None
            }
            for dev in devices
        ]
        out(json.dumps({"version": "1.5", "updates_available": entries}, indent=2))
        return
    for dev in devices:
        status = update_status(
            dev, targets, mandatory, min_hw_id=args.min_hw_id, apply_hw_gate=apply_hw_gate
        )
        must = status["must_update"]
        must_text = "unknown" if must is None else str(must)
        out(
            f"Type: {dev.name:<11}  SN: {provisioned_text(dev.serial_number)}  "
            f"hw_id: 0x{dev.hardware_id:X}  ts: {human_ts(dev.current_ts)}  "
            f"must_update: {must_text}"
        )


def cmd_list(args: argparse.Namespace) -> int:
    config = load_config_for_args(args, required=False)
    targets, mandatory = timestamp_sources(args, config)
    devices = discover_devices(verbose=args.verbose)
    print_device_list(devices, targets, mandatory, args, apply_hw_gate=False)
    return 0


def cmd_check_updates(args: argparse.Namespace) -> int:
    config = load_config_for_args(args, required=False)
    targets, mandatory = timestamp_sources(args, config)
    if not targets:
        raise ToolError(
            "ERROR: check-updates requires config timestamps or explicit --ibex-ts/--proteus-ts"
        )
    devices = discover_devices(verbose=args.verbose)
    print_device_list(devices, targets, mandatory, args, apply_hw_gate=True)
    return 0


def cmd_update_all(args: argparse.Namespace) -> int:
    config = load_config_for_args(args, required=False)
    targets, _ = timestamp_sources(args, config)
    for fw_class in ("ibex", "proteus"):
        if getattr(args, f"{fw_class}_fw", None) and fw_class not in targets:
            path = getattr(args, f"{fw_class}_fw")
            raise ToolError(
                f"ERROR: update-all requires --{fw_class}-ts or config timestamp for custom firmware {path}"
            )
    if not targets:
        raise ToolError(
            "ERROR: update-all requires config timestamps or explicit --ibex-ts/--proteus-ts"
        )
    devices = discover_devices(verbose=args.verbose)
    for dev in devices:
        if not dev.is_updateable(args.min_hw_id, apply_hw_gate=True):
            continue
        target = targets.get(dev.fw_class)
        if target is None:
            continue
        if dev.current_ts is not None and dev.current_ts == target:
            continue
        fw = require_firmware_for_class(args, config, dev.fw_class, device_class=dev.fw_class)
        flash_device(dev, fw, args)
    return 0


def flash_device(device: Device, firmware: FirmwareImage, args: argparse.Namespace) -> None:
    cfw_nvs_backup: CfwNvsImage | None = None

    if not device.updateable_connection:
        raise ToolError(
            "ERROR: Device is visible only over BLE/ESB and cannot be flashed from this connection"
        )
    if device.transport == "serial":
        if not device.bootloader_port:
            raise ToolError("ERROR: Couldn't open BL comport")
        flash_bootloader_port(device.bootloader_port, firmware, verbose=args.verbose)
        return

    # Only Ibex has custom firmware for now.
    if device.fw_class == "ibex":
        cfw_nvs_backup = try_save_cfw_nvs_from_device(device, verbose=args.verbose)
    send_hid_reboot_to_bootloader(device)
    time.sleep(4)
    for bl in enumerate_bootloader_devices(verbose=args.verbose):
        if bl.serial_number == device.serial_number:
            flash_bootloader_port(bl.bootloader_port or "", firmware, verbose=args.verbose)
            if cfw_nvs_backup is not None:
                restored = wait_for_hid_device(
                    device.serial_number or "",
                    timeout=CFW_NVS_RESTORE_HID_TIMEOUT_SECONDS,
                    verbose=args.verbose,
                )
                restore_cfw_nvs_to_device(restored, cfw_nvs_backup, verbose=args.verbose)
                out("RESTORED CFW SETTINGS")
                time.sleep(CFW_NVS_REBOOT_DELAY_SECONDS)
            return
    raise ToolError(
        f"ERROR: No device with serial number: {provisioned_text(device.serial_number)}"
    )


def cmd_update(args: argparse.Namespace) -> int:
    config = load_config_for_args(args, required=False)
    devices = discover_devices(verbose=args.verbose)
    matches = [dev for dev in devices if dev.serial_number == args.serial]
    if not matches:
        raise ToolError(f"ERROR: No device with serial number: {args.serial}")
    device = matches[0]
    if not device.updateable_connection:
        raise ToolError(
            "ERROR: Device is visible only over BLE/ESB and cannot be flashed from this connection"
        )
    target = args.target or device.fw_class
    if target != device.fw_class:
        raise ToolError(f"ERROR: Target {target} does not match device class {device.fw_class}")
    fw = require_firmware_for_class(args, config, target, device_class=device.fw_class)
    flash_device(device, fw, args)
    return 0


def hid_info_object(device: Device) -> dict[str, Any]:
    result: dict[str, Any] = {
        "type": device.name,
        "serial_number": provisioned_text(device.serial_number),
        "hw_id": device.hardware_id,
        "build_timestamp": device.current_ts,
    }
    for name in NUMERIC_TAGS.values():
        result.setdefault(name, device.attributes.get(name))
    for key in (
        "build_timestamp",
        "boot_build_timestamp",
        "radio_build_timestamp",
        "secondary_build_timestamp",
        "secondary_boot_build_timestamp",
    ):
        value = result.get(key)
        result[f"{key}_hex"] = hex_opt(value)
        result[f"{key}_utc"] = timestamp_datetime(value)
    return result


def cmd_info(args: argparse.Namespace) -> int:
    devices = enumerate_hid_devices(info_only=True, verbose=args.verbose)
    if args.serial:
        devices = [
            dev
            for dev in devices
            if dev.serial_number == args.serial and dev.type_id != TYPE_TRITON_ESB
        ]
    objects = [hid_info_object(dev) for dev in devices if dev.type_id != TYPE_TRITON_ESB]
    if args.json_output:
        out(json.dumps(objects, indent=2))
    else:
        for obj in objects:
            out(f"type: {obj['type']}")
            out(f"serial_number: {obj['serial_number']}")
            for key in sorted(k for k in obj if k not in ("type", "serial_number")):
                value = obj[key]
                if value is not None:
                    out(f"{key}: {value}")
    return 0


def cmd_enter_bl(args: argparse.Namespace) -> int:
    devices = [
        dev
        for dev in enumerate_hid_devices(verbose=args.verbose)
        if dev.serial_number == args.serial
    ]
    if not devices:
        raise ToolError(f"ERROR: No HID device with serial number: {args.serial}")
    send_hid_reboot_to_bootloader(devices[0])
    out("SUCCESS")
    return 0


def cmd_reboot(args: argparse.Namespace) -> int:
    devices = [
        dev
        for dev in enumerate_hid_devices(verbose=args.verbose)
        if dev.serial_number == args.serial and dev.type_id in (TYPE_TRITON_USB, TYPE_TRITON_BLE)
    ]
    if not devices:
        all_hid = [
            dev
            for dev in enumerate_hid_devices(verbose=args.verbose)
            if dev.serial_number == args.serial
        ]
        if all_hid:
            raise ToolError("ERROR: Normal reboot is only supported for Triton-class devices")
        raise ToolError(f"ERROR: No Triton HID device with serial number: {args.serial}")
    send_hid_normal_reboot(devices[0])
    out("SUCCESS")
    return 0


def find_updateable_hid_by_serial(serial: str, *, verbose: bool = False) -> Device:
    devices = [
        dev
        for dev in enumerate_hid_devices(verbose=verbose)
        if dev.serial_number == serial and dev.updateable_connection
    ]
    if not devices:
        raise ToolError(f"ERROR: No updateable HID device with serial number: {serial}")
    return devices[0]


def cmd_settings_save(args: argparse.Namespace) -> int:
    device = find_updateable_hid_by_serial(args.serial, verbose=args.verbose)
    image = save_cfw_nvs_from_device(device, verbose=args.verbose)
    if image is None:
        raise ToolError("ERROR: Device does not expose CFW NVS save support")

    try:
        Path(args.output).write_bytes(image.data)
    except OSError as exc:
        raise ToolError(f"ERROR: Could not write settings image: {args.output}") from exc

    out(f"SAVED: {args.output} ({len(image.data)} bytes)")
    return 0


def cmd_settings_restore(args: argparse.Namespace) -> int:
    device = find_updateable_hid_by_serial(args.serial, verbose=args.verbose)
    try:
        data = Path(args.input).read_bytes()
    except OSError as exc:
        raise ToolError(f"ERROR: Could not read settings image: {args.input}") from exc
    if len(data) == 0 or len(data) % 4:
        raise ToolError(f"ERROR: Invalid settings image size: {len(data)}")

    restore_cfw_nvs_to_device(
        device,
        CfwNvsImage(data=data, chunk_size=0),
        verbose=args.verbose,
        reboot=not args.no_reboot,
    )
    out(f"RESTORED: {args.input} ({len(data)} bytes)")
    if not args.no_reboot:
        out("REBOOTING")
    return 0


def boot_info_to_json(info: BootInfo) -> dict[str, Any]:
    return {
        "port": info.port,
        "class": info.fw_class,
        "hw_id": info.hw_id,
        "unit_serial": info.unit_serial,
        "pcba_serial": info.pcba_serial,
    }


def cmd_bl_list(args: argparse.Namespace) -> int:
    infos: list[BootInfo] = []
    for port_name, fw_class in list_bootloader_ports(args.verbose):
        try:
            infos.append(query_boot_info(port_name, fw_class, verbose=args.verbose))
        except ToolError:
            if args.verbose:
                out(f"WARNING: Could not query bootloader port {port_name}")
    if args.json_output:
        out(json.dumps([boot_info_to_json(info) for info in infos], indent=2))
    else:
        for info in infos:
            out(
                f"port: {info.port}  class: {info.fw_class}  "
                f"hw_id: 0x{info.hw_id:X} ({info.hw_id})  "
                f"unit_serial: {info.unit_serial}  pcba_serial: {info.pcba_serial}"
            )
    return 0


def find_bootloader_by_serial(serial: str, verbose: bool = False) -> BootInfo:
    for port_name, fw_class in list_bootloader_ports(verbose):
        info = query_boot_info(port_name, fw_class, verbose=verbose)
        if info.unit_serial == serial:
            return info
    raise ToolError(f"ERROR: No device with serial number: {serial}")


def class_for_port(port_name: str, verbose: bool = False) -> str | None:
    try:
        from serial.tools import list_ports
    except ImportError as exc:
        raise ToolError("ERROR: Missing pyserial Python module, install requirements.txt") from exc

    for port in list_ports.comports():
        if str(port.device) == port_name:
            return detect_bootloader_class(port)
    return None


def resolve_bootloader_selection(
    args: argparse.Namespace, *, query: bool
) -> tuple[str, str | None, BootInfo | None]:
    if args.serial:
        info = find_bootloader_by_serial(args.serial, args.verbose)
        return info.port, info.fw_class, info
    fw_class = class_for_port(args.port, args.verbose)
    if query and fw_class:
        info = query_boot_info(args.port, fw_class, verbose=args.verbose)
        return args.port, fw_class, info
    return args.port, fw_class, None


def cmd_bl_info(args: argparse.Namespace) -> int:
    port_name, fw_class, info = resolve_bootloader_selection(args, query=True)
    if info is None:
        if fw_class is None:
            fw_class = args.target or "unknown"
        info = query_boot_info(port_name, fw_class, verbose=args.verbose)
    if args.json_output:
        data = boot_info_to_json(info)
        data["raw_response"] = info.raw_response.hex()
        out(json.dumps(data, indent=2))
    else:
        out(f"port:          {info.port}")
        out(f"device class:  {info.fw_class}")
        out(f"hw_id:         0x{info.hw_id:X} ({info.hw_id})")
        out(f"unit_serial:   {info.unit_serial}")
        out(f"pcba_serial:   {info.pcba_serial}")
        out(f"raw_response:  {info.raw_response.hex()}")
    return 0


def cmd_bl_flash(args: argparse.Namespace) -> int:
    port_name, fw_class, _ = resolve_bootloader_selection(args, query=False)
    device_class = fw_class
    if device_class is None and args.target is None:
        out(
            "WARNING: Could not infer bootloader class from port; firmware/device class match was not verified"
        )
    fw = FirmwareImage.load(
        Path(args.firmware),
        target=args.target,
        device_class=device_class,
        verbose=args.verbose,
    )
    flash_bootloader_port(port_name, fw, verbose=args.verbose)
    return 0


def cmd_bl_provision_uicr(args: argparse.Namespace) -> int:
    var = "BRICK_MY_CONTROLLER"
    mag = "Yes, please!"
    if os.environ.get(var) != mag:
        raise ToolError(
            "!!!WARNING!!!: UICR provisioning erases and rewrites the device UICR. "
            f"To proceed, set {var} to the exact value: {mag}"
        )
    port_name, fw_class, _ = resolve_bootloader_selection(args, query=True)
    if fw_class != "ibex":
        raise ToolError(
            "ERROR: UICR provisioning is only supported for an identified Ibex bootloader"
        )
    try:
        customer = Path(args.customer).read_bytes()
    except OSError as exc:
        raise ToolError(f"ERROR: Could not read UICR CUSTOMER file: {args.customer}") from exc
    if len(customer) != UICR_CUSTOMER_SIZE:
        raise ToolError(f"ERROR: UICR CUSTOMER file must be exactly {UICR_CUSTOMER_SIZE} bytes")

    body = struct.pack("<I", UICR_PROVISION_KEY) + customer
    port = open_serial_port(port_name, verbose=args.verbose)
    try:
        bootloader_command(port, MESSAGE_UICR_PROVISION, body, verbose=args.verbose)
    finally:
        try:
            port.close()
        except Exception:
            pass
    out("SUCCESS")
    return 0


def cmd_bl_exit(args: argparse.Namespace) -> int:
    port_name, _, _ = resolve_bootloader_selection(args, query=False)
    port = open_serial_port(port_name, verbose=args.verbose)
    try:
        bootloader_command(port, MESSAGE_RESET, verbose=args.verbose)
    finally:
        try:
            port.close()
        except Exception:
            pass
    out("SUCCESS")
    return 0


def add_common_options(parser: argparse.ArgumentParser) -> None:
    parser.add_argument("--config", default=argparse.SUPPRESS)
    parser.add_argument("--ibex-fw", default=argparse.SUPPRESS)
    parser.add_argument("--proteus-fw", default=argparse.SUPPRESS)
    parser.add_argument("--ibex-ts", default=argparse.SUPPRESS)
    parser.add_argument("--proteus-ts", default=argparse.SUPPRESS)
    parser.add_argument("--target", choices=("ibex", "proteus"), default=argparse.SUPPRESS)
    parser.add_argument(
        "--json", action="store_true", dest="json_output", default=argparse.SUPPRESS
    )
    parser.add_argument("--verbose", "-v", action="store_true", default=argparse.SUPPRESS)
    parser.add_argument("--min-hw-id", type=int, default=argparse.SUPPRESS)


def build_parser() -> argparse.ArgumentParser:
    common = argparse.ArgumentParser(add_help=False)
    add_common_options(common)
    parser = argparse.ArgumentParser(prog="flash.py", parents=[common])
    sub = parser.add_subparsers(dest="command", required=True)

    p = sub.add_parser("list", parents=[common])
    p.set_defaults(func=cmd_list)

    p = sub.add_parser("check-updates", parents=[common])
    p.set_defaults(func=cmd_check_updates)

    p = sub.add_parser("update-all", parents=[common])
    p.set_defaults(func=cmd_update_all)

    p = sub.add_parser("update", parents=[common])
    p.add_argument("--serial", required=True)
    p.set_defaults(func=cmd_update)

    p = sub.add_parser("info", parents=[common])
    p.add_argument("--serial")
    p.set_defaults(func=cmd_info)

    p = sub.add_parser("enter-bl", parents=[common])
    p.add_argument("--serial", required=True)
    p.set_defaults(func=cmd_enter_bl)

    p = sub.add_parser("reboot", parents=[common])
    p.add_argument("--serial", required=True)
    p.set_defaults(func=cmd_reboot)

    p = sub.add_parser("settings-save", parents=[common])
    p.add_argument("--serial", required=True)
    p.add_argument("--output", required=True)
    p.set_defaults(func=cmd_settings_save)

    p = sub.add_parser("settings-restore", parents=[common])
    p.add_argument("--serial", required=True)
    p.add_argument("--input", required=True)
    p.add_argument("--no-reboot", action="store_true", default=False)
    p.set_defaults(func=cmd_settings_restore)

    p = sub.add_parser("bl-list", parents=[common])
    p.set_defaults(func=cmd_bl_list)

    p = sub.add_parser("bl-info", parents=[common])
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--port")
    group.add_argument("--serial")
    p.set_defaults(func=cmd_bl_info)

    p = sub.add_parser("bl-flash", parents=[common])
    p.add_argument("--firmware", required=True)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--port")
    group.add_argument("--serial")
    p.set_defaults(func=cmd_bl_flash)

    p = sub.add_parser("bl-provision-uicr", parents=[common])
    p.add_argument("--customer", required=True)
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--port")
    group.add_argument("--serial")
    p.set_defaults(func=cmd_bl_provision_uicr)

    p = sub.add_parser("bl-exit", parents=[common])
    group = p.add_mutually_exclusive_group(required=True)
    group.add_argument("--port")
    group.add_argument("--serial")
    p.set_defaults(func=cmd_bl_exit)
    return parser


def normalize_args(args: argparse.Namespace) -> argparse.Namespace:
    for name, value in (
        ("config", None),
        ("ibex_fw", None),
        ("proteus_fw", None),
        ("ibex_ts", None),
        ("proteus_ts", None),
        ("target", None),
        ("json_output", False),
        ("verbose", False),
        ("min_hw_id", DEFAULT_MIN_HW_ID),
    ):
        if not hasattr(args, name):
            setattr(args, name, value)
    return args


def main(argv: list[str] | None = None) -> int:
    parser = build_parser()
    args = normalize_args(parser.parse_args(argv))
    try:
        return int(args.func(args))
    except ToolError as exc:
        out(str(exc))
        return 1
    except KeyboardInterrupt:
        out("ERROR: Interrupted")
        return 130


if __name__ == "__main__":
    raise SystemExit(main())
