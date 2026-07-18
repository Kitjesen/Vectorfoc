#!/usr/bin/env python3
"""Tests for VectorFOC application-image packaging and OTA preflight."""

from __future__ import annotations

import struct
import sys
import unittest
from pathlib import Path
from unittest import mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from scripts import ota_upload, patch_app_header as header_tools


class AppHeaderToolsTest(unittest.TestCase):
    @staticmethod
    def packaged_image(payload: bytes = b"vectorfoc-payload") -> bytes:
        raw = bytes(header_tools.APP_PAYLOAD_OFFSET) + payload
        header = header_tools.make_header(raw, (1, 2, 3), 0)
        image = bytearray(raw)
        image[header_tools.APP_HEADER_OFFSET:header_tools.APP_PAYLOAD_OFFSET] = header
        return bytes(image)

    def test_crc_known_vector(self) -> None:
        self.assertEqual(header_tools.calc_crc32(b"123456789"), 0xCBF43926)

    def test_round_trip_header_validation(self) -> None:
        image = self.packaged_image()
        parsed = header_tools.validate_image(image)
        self.assertEqual(parsed.version, 0x00010203)
        self.assertEqual(parsed.size, len(b"vectorfoc-payload"))
        self.assertEqual(parsed.build_time, 0)

    def test_payload_corruption_is_rejected(self) -> None:
        image = bytearray(self.packaged_image())
        image[-1] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC mismatch"):
            header_tools.validate_image(bytes(image))

    def test_vector_table_corruption_is_rejected(self) -> None:
        image = bytearray(self.packaged_image())
        image[0] ^= 0x01
        with self.assertRaisesRegex(ValueError, "CRC mismatch"):
            header_tools.validate_image(bytes(image))

    def test_declared_size_mismatch_is_rejected(self) -> None:
        image = bytearray(self.packaged_image())
        struct.pack_into("<I", image, header_tools.APP_HEADER_OFFSET + 8, 1)
        with self.assertRaisesRegex(ValueError, "header declares"):
            header_tools.validate_image(bytes(image))

    def test_oversize_image_is_rejected(self) -> None:
        image = bytes(header_tools.APP_IMAGE_SIZE + 1)
        with self.assertRaisesRegex(ValueError, "exceeding"):
            header_tools.make_header(image, (1, 0, 0), 0)

    def test_ota_preflight_reuses_packaged_header(self) -> None:
        image = self.packaged_image(b"ota")
        path = Path("VectorFoc.bin")
        with mock.patch.object(Path, "is_file", return_value=True), \
             mock.patch.object(Path, "read_bytes", return_value=image):
            loaded, parsed = ota_upload.load_firmware(path)
        self.assertEqual(loaded, image)
        self.assertEqual(parsed.size, 3)

    def test_boot_info_preflight_accepts_matching_layout(self) -> None:
        self.assertEqual(
            ota_upload.parse_boot_info("boot_info,app_start=08004000,app_size=131072"),
            (ota_upload.APP_ADDR_START, 131072),
        )

    def test_boot_info_preflight_rejects_layout_mismatch(self) -> None:
        class FakeSerial:
            in_waiting = 45

            def write(self, payload: bytes) -> int:
                return len(payload)

            def flush(self) -> None:
                pass

            def read(self, count: int) -> bytes:
                self.in_waiting = 0
                return b"boot_info,app_start=08005000,app_size=110592\n"[:count]

        with self.assertRaisesRegex(RuntimeError, "app_start mismatch"):
            ota_upload.query_boot_info(FakeSerial(), len(self.packaged_image(b"ota")))

    def test_unpatched_image_is_rejected_before_serial_open(self) -> None:
        path = Path("VectorFoc.bin")
        with mock.patch.object(Path, "is_file", return_value=True), \
             mock.patch.object(Path, "read_bytes", return_value=bytes(header_tools.APP_PAYLOAD_OFFSET + 4)):
            with self.assertRaisesRegex(ValueError, "invalid app magic"):
                ota_upload.load_firmware(path)


if __name__ == "__main__":
    unittest.main()
