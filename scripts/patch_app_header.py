#!/usr/bin/env python3
"""Generate and validate the VectorFOC bootloader application header."""

from __future__ import annotations

import argparse
import os
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Sequence

APP_MAGIC = 0x56464F43
APP_IMAGE_SIZE = 108 * 1024
APP_HEADER_OFFSET = 0x200
APP_HEADER_STRUCT = struct.Struct("<IIIII3I")
APP_PAYLOAD_OFFSET = APP_HEADER_OFFSET + APP_HEADER_STRUCT.size


@dataclass(frozen=True)
class AppHeader:
    magic: int
    version: int
    size: int
    crc32: int
    build_time: int
    reserved: tuple[int, int, int]


def calc_crc32(data: bytes) -> int:
    """Return the reflected IEEE 802.3 CRC used by the bootloader."""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ (0xEDB88320 if crc & 1 else 0)
    return crc ^ 0xFFFFFFFF


def crc_image_region(image: bytes, payload_size: int | None = None) -> int:
    """CRC the vector table and post-header payload, excluding the app header."""
    if payload_size is None:
        payload_size = len(image) - APP_PAYLOAD_OFFSET
    if payload_size <= 0:
        raise ValueError("app payload size is zero")
    end = APP_PAYLOAD_OFFSET + payload_size
    if len(image) < end:
        raise ValueError("image is shorter than the declared app payload")
    return calc_crc32(image[:APP_HEADER_OFFSET] + image[APP_PAYLOAD_OFFSET:end])


def parse_version(text: str) -> tuple[int, int, int]:
    parts = text.split(".")
    if len(parts) != 3:
        raise ValueError("version must use MAJOR.MINOR.PATCH format")
    try:
        major, minor, patch = (int(part, 10) for part in parts)
    except ValueError as exc:
        raise ValueError("version components must be decimal integers") from exc
    if not (0 <= major <= 0xFFFF and 0 <= minor <= 0xFF and 0 <= patch <= 0xFF):
        raise ValueError("version components exceed header field widths")
    return major, minor, patch


def pack_version(version: Sequence[int]) -> int:
    major, minor, patch = version
    return (major << 16) | (minor << 8) | patch


def default_build_time() -> int:
    value = os.environ.get("SOURCE_DATE_EPOCH")
    if value is None:
        return 0
    timestamp = int(value, 10)
    if not 0 <= timestamp <= 0xFFFFFFFF:
        raise ValueError("SOURCE_DATE_EPOCH must fit in uint32")
    return timestamp


def unpack_header(image: bytes) -> AppHeader:
    if len(image) < APP_PAYLOAD_OFFSET:
        raise ValueError("image is too short to contain the vector table and app header")
    values = APP_HEADER_STRUCT.unpack_from(image, APP_HEADER_OFFSET)
    return AppHeader(
        magic=values[0],
        version=values[1],
        size=values[2],
        crc32=values[3],
        build_time=values[4],
        reserved=(values[5], values[6], values[7]),
    )


def make_header(image: bytes, version: Sequence[int], build_time: int) -> bytes:
    if len(image) > APP_IMAGE_SIZE:
        raise ValueError(
            f"image is {len(image)} bytes, exceeding the {APP_IMAGE_SIZE}-byte app region"
        )
    if len(image) <= APP_PAYLOAD_OFFSET:
        raise ValueError("image contains no payload after the app header")
    payload = image[APP_PAYLOAD_OFFSET:]
    return APP_HEADER_STRUCT.pack(
        APP_MAGIC,
        pack_version(version),
        len(payload),
        crc_image_region(image, len(payload)),
        build_time,
        0,
        0,
        0,
    )


def validate_image(image: bytes) -> AppHeader:
    if len(image) > APP_IMAGE_SIZE:
        raise ValueError(
            f"image is {len(image)} bytes, exceeding the {APP_IMAGE_SIZE}-byte app region"
        )
    header = unpack_header(image)
    if header.magic != APP_MAGIC:
        raise ValueError(f"invalid app magic 0x{header.magic:08X}")
    if header.size == 0:
        raise ValueError("app payload size is zero")
    expected_length = APP_PAYLOAD_OFFSET + header.size
    if expected_length != len(image):
        raise ValueError(
            f"header declares {header.size} payload bytes, but image length implies "
            f"{len(image) - APP_PAYLOAD_OFFSET}"
        )
    calculated = crc_image_region(image, header.size)
    if calculated != header.crc32:
        raise ValueError(
            f"payload CRC mismatch: header=0x{header.crc32:08X}, "
            f"calculated=0x{calculated:08X}"
        )
    if header.reserved != (0, 0, 0):
        raise ValueError("reserved app-header fields must be zero")
    return header


def command_generate(args: argparse.Namespace) -> None:
    image = args.image.read_bytes()
    build_time = default_build_time() if args.build_time is None else args.build_time
    if not 0 <= build_time <= 0xFFFFFFFF:
        raise ValueError("build time must fit in uint32")
    header = make_header(image, parse_version(args.version), build_time)
    args.header_out.write_bytes(header)
    parsed = AppHeader(*APP_HEADER_STRUCT.unpack(header)[:5], tuple(APP_HEADER_STRUCT.unpack(header)[5:]))
    print(
        f"generated app header: size={parsed.size}, crc32=0x{parsed.crc32:08X}, "
        f"version=0x{parsed.version:08X}"
    )


def command_verify(args: argparse.Namespace) -> None:
    header = validate_image(args.image.read_bytes())
    print(
        f"verified app image: size={header.size}, crc32=0x{header.crc32:08X}, "
        f"version=0x{header.version:08X}"
    )


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    generate = subparsers.add_parser("generate", help="generate a 32-byte header file")
    generate.add_argument("--image", type=Path, required=True, help="unpatched raw app binary")
    generate.add_argument("--header-out", type=Path, required=True)
    generate.add_argument("--version", required=True, help="MAJOR.MINOR.PATCH")
    generate.add_argument("--build-time", type=int)
    generate.set_defaults(handler=command_generate)

    verify = subparsers.add_parser("verify", help="validate a packaged app binary")
    verify.add_argument("--image", type=Path, required=True)
    verify.set_defaults(handler=command_verify)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    args = parser.parse_args(argv)
    try:
        args.handler(args)
    except (OSError, ValueError, struct.error) as exc:
        parser.error(str(exc))
    return 0


if __name__ == "__main__":
    sys.exit(main())
