#!/usr/bin/env python3
"""Upload a pre-packaged VectorFOC application image over USB CDC."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path
from typing import Any, Sequence

try:
    import serial  # type: ignore[import-not-found]
except ImportError:  # Keep image validation usable without pyserial installed.
    serial = None

try:
    from .patch_app_header import APP_IMAGE_SIZE, AppHeader, validate_image
except ImportError:
    from patch_app_header import APP_IMAGE_SIZE, AppHeader, validate_image

APP_ADDR_START = 0x08004000
BLOCK_SIZE = 256


def load_firmware(path: Path) -> tuple[bytes, AppHeader]:
    if not path.is_file():
        raise ValueError(f"firmware file not found: {path}")
    image = path.read_bytes()
    header = validate_image(image)
    return image, header


def wait_response(ser: Any, expected: str, timeout: float = 5.0) -> str:
    deadline = time.monotonic() + timeout
    pending = ""
    while time.monotonic() < deadline:
        waiting = int(getattr(ser, "in_waiting", 0))
        if waiting:
            pending += ser.read(waiting).decode("utf-8", errors="ignore")
            lines = pending.split("\n")
            pending = lines.pop()
            for raw_line in lines:
                line = raw_line.strip()
                if line:
                    print(f"  <- {line}")
                    if expected in line:
                        return line
        time.sleep(0.01)
    raise TimeoutError(f"timeout waiting for '{expected}'")


def send_command(ser: Any, command: str) -> None:
    print(f"  -> {command}")
    payload = (command + "\n").encode("ascii")
    written = ser.write(payload)
    if written != len(payload):
        raise OSError(f"short serial write: {written}/{len(payload)} bytes")
    ser.flush()


def open_serial(port: str, baud: int, timeout: float = 8.0) -> Any:
    if serial is None:
        raise RuntimeError("pyserial is required: python -m pip install pyserial")
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            return serial.Serial(port, baud, timeout=0.1, write_timeout=2.0)
        except serial.SerialException as exc:
            last_error = exc
            time.sleep(0.25)
    raise RuntimeError(f"cannot open {port}: {last_error}")


def reconnect_bootloader(ser: Any, port: str, baud: int) -> Any:
    try:
        wait_response(ser, "boot_ready", timeout=2.0)
        return ser
    except (TimeoutError, OSError):
        try:
            ser.close()
        except Exception:
            pass

    boot_ser = open_serial(port, baud)
    boot_ser.reset_input_buffer()
    send_command(boot_ser, "boot_info")
    wait_response(boot_ser, "boot_info", timeout=3.0)
    return boot_ser


def require_ack(response: str, operation: str) -> None:
    if not response.startswith("boot_ack,0"):
        raise RuntimeError(f"{operation} failed: {response}")


def upload_firmware(port: str, firmware_path: str, baud: int = 115200) -> bool:
    try:
        image, header = load_firmware(Path(firmware_path))
    except (OSError, ValueError) as exc:
        print(f"Error: {exc}")
        return False

    print(
        f"Firmware: {Path(firmware_path).name}, image={len(image)} bytes, "
        f"payload={header.size} bytes, CRC32=0x{header.crc32:08X}"
    )

    ser: Any | None = None
    try:
        ser = open_serial(port, baud)
        time.sleep(0.25)

        print("\n[1/5] Entering bootloader...")
        send_command(ser, "boot_enter")
        ser = reconnect_bootloader(ser, port, baud)

        print("\n[2/5] Erasing application area...")
        send_command(ser, "boot_erase")
        require_ack(wait_response(ser, "boot_ack", timeout=10.0), "erase")

        print(f"\n[3/5] Writing image ({len(image)} bytes)...")
        image_offset = 0
        address = APP_ADDR_START
        app_end_exclusive = APP_ADDR_START + APP_IMAGE_SIZE

        while image_offset < len(image):
            raw_chunk = image[image_offset:image_offset + BLOCK_SIZE]
            padded_chunk = raw_chunk + b"\xFF" * ((-len(raw_chunk)) % 8)
            if address + len(padded_chunk) > app_end_exclusive:
                raise ValueError("aligned final block exceeds the application region")

            send_command(ser, f"boot_write,{address:08x},{len(padded_chunk)}")
            require_ack(wait_response(ser, "boot_ack", timeout=2.0), "write prepare")

            written = ser.write(padded_chunk)
            if written != len(padded_chunk):
                raise OSError(f"short firmware write: {written}/{len(padded_chunk)} bytes")
            ser.flush()
            require_ack(wait_response(ser, "boot_ack", timeout=5.0), "flash write")

            image_offset += len(raw_chunk)
            address += len(padded_chunk)
            progress = image_offset * 100 // len(image)
            print(f"\r  Progress: {progress}% ({image_offset}/{len(image)} bytes)", end="")
        print()

        print("\n[4/5] Verifying packaged payload CRC...")
        send_command(ser, f"boot_verify,{header.crc32:08x},{header.size}")
        require_ack(wait_response(ser, "boot_ack", timeout=8.0), "CRC verify")

        print("\n[5/5] Rebooting to application...")
        send_command(ser, "boot_reboot")
        try:
            require_ack(wait_response(ser, "boot_ack", timeout=2.0), "reboot")
        except TimeoutError:
            pass

        print("\nFirmware upload complete.")
        return True
    except Exception as exc:
        print(f"\nError: {exc}")
        return False
    finally:
        if ser is not None:
            try:
                ser.close()
            except Exception:
                pass


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("firmware", help="packaged VectorFoc.bin")
    parser.add_argument("--port", "-p", default="COM3")
    parser.add_argument("--baud", "-b", type=int, default=115200)
    args = parser.parse_args(argv)
    return 0 if upload_firmware(args.port, args.firmware, args.baud) else 1


if __name__ == "__main__":
    sys.exit(main())
