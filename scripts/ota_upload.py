#!/usr/bin/env python3
"""
VectorFOC OTA Firmware Uploader

Usage:
    python ota_upload.py <firmware.bin> [--port COM3] [--baud 115200]

Protocol:
    1. Send "boot_enter" to App (if running)
    2. Wait for "boot_ready" from Bootloader
    3. Send "boot_erase" to erase App area
    4. Send firmware in chunks: "boot_write,addr,len" + binary data
    5. Send "boot_verify,crc" to verify
    6. Send "boot_reboot" to restart
"""

import argparse
import serial
import struct
import time
import sys
from pathlib import Path

# Flash layout
APP_ADDR_START = 0x08004000
APP_HEADER_OFFSET = 0x200
APP_MAGIC = 0x56464F43  # "VFOC"
BLOCK_SIZE = 256

def calc_crc32(data: bytes) -> int:
    """Calculate CRC32 (IEEE 802.3)"""
    crc = 0xFFFFFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xEDB88320
            else:
                crc >>= 1
    return crc ^ 0xFFFFFFFF

def wait_response(ser: serial.Serial, expected: str, timeout: float = 5.0) -> str:
    """Wait for a specific response"""
    start = time.time()
    buffer = ""
    while time.time() - start < timeout:
        if ser.in_waiting:
            data = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
            buffer += data
            lines = buffer.split('\n')
            for line in lines[:-1]:
                line = line.strip()
                if line:
                    print(f"  <- {line}")
                    if expected in line:
                        return line
            buffer = lines[-1]
        time.sleep(0.01)
    raise TimeoutError(f"Timeout waiting for '{expected}'")

def send_command(ser: serial.Serial, cmd: str):
    """Send a text command"""
    print(f"  -> {cmd}")
    ser.write((cmd + '\n').encode('utf-8'))
    ser.flush()

def create_app_header(firmware_data: bytes, version: tuple = (1, 0, 0)) -> bytes:
    """Create App Header structure"""
    # Calculate CRC of firmware (excluding header)
    crc = calc_crc32(firmware_data)
    
    # Header structure: magic(4) + version(4) + size(4) + crc(4) + build_time(4) + reserved(12)
    version_packed = (version[0] << 16) | (version[1] << 8) | version[2]
    build_time = int(time.time())
    
    header = struct.pack('<IIIII3I',
        APP_MAGIC,
        version_packed,
        len(firmware_data),
        crc,
        build_time,
        0, 0, 0  # reserved
    )
    return header

def upload_firmware(port: str, firmware_path: str, baud: int = 115200):
    """Upload firmware via OTA"""
    
    # Read firmware file
    firmware_path = Path(firmware_path)
    if not firmware_path.exists():
        print(f"Error: Firmware file not found: {firmware_path}")
        return False
    
    firmware_data = firmware_path.read_bytes()
    print(f"Firmware: {firmware_path.name}, Size: {len(firmware_data)} bytes")
    
    # Open serial port
    print(f"Opening {port} at {baud} baud...")
    ser = serial.Serial(port, baud, timeout=0.1)
    time.sleep(0.5)  # Wait for connection
    
    try:
        # Step 1: Enter bootloader
        print("\n[1/5] Entering bootloader...")
        send_command(ser, "boot_enter")
        
        try:
            wait_response(ser, "boot_ready", timeout=3.0)
        except TimeoutError:
            # Maybe already in bootloader, try again
            print("  Retrying...")
            time.sleep(1)
            ser.reset_input_buffer()
            # Send any command to trigger ready message
            send_command(ser, "boot_info")
            try:
                wait_response(ser, "boot_", timeout=2.0)
            except TimeoutError:
                print("Error: Cannot enter bootloader")
                return False
        
        # Step 2: Erase App area
        print("\n[2/5] Erasing App area...")
        send_command(ser, "boot_erase")
        resp = wait_response(ser, "boot_ack", timeout=10.0)
        if "boot_ack,0" not in resp:
            print(f"Error: Erase failed: {resp}")
            return False
        
        # Step 3: Write firmware
        print(f"\n[3/5] Writing firmware ({len(firmware_data)} bytes)...")
        
        # Prepare data with header
        # The header goes at APP_HEADER_OFFSET (0x200)
        # We need to pad the beginning if firmware doesn't include vector table
        
        # For simplicity, assume firmware.bin starts at APP_ADDR_START
        # and we write it as-is
        
        total_written = 0
        addr = APP_ADDR_START
        
        while total_written < len(firmware_data):
            chunk_size = min(BLOCK_SIZE, len(firmware_data) - total_written)
            chunk = firmware_data[total_written:total_written + chunk_size]
            
            # Pad to 8-byte alignment
            if len(chunk) % 8 != 0:
                chunk += b'\xFF' * (8 - len(chunk) % 8)
            
            # Send write command
            send_command(ser, f"boot_write,{addr:08x},{len(chunk)}")
            resp = wait_response(ser, "boot_ack", timeout=2.0)
            if "boot_ack,0" not in resp:
                print(f"Error: Write prepare failed at 0x{addr:08X}: {resp}")
                return False
            
            # Send binary data
            ser.write(chunk)
            ser.flush()
            
            # Wait for write complete
            resp = wait_response(ser, "boot_ack", timeout=5.0)
            if "boot_ack,0" not in resp:
                print(f"Error: Write failed at 0x{addr:08X}: {resp}")
                return False
            
            total_written += chunk_size
            addr += len(chunk)
            
            # Progress
            progress = total_written * 100 // len(firmware_data)
            print(f"\r  Progress: {progress}% ({total_written}/{len(firmware_data)} bytes)", end='')
        
        print()  # New line after progress
        
        # Step 4: Verify CRC
        print("\n[4/5] Verifying CRC...")
        crc = calc_crc32(firmware_data)
        send_command(ser, f"boot_verify,{crc:08x},{len(firmware_data)}")
        resp = wait_response(ser, "boot_ack", timeout=5.0)
        if "boot_ack,0" not in resp:
            print(f"Error: CRC verification failed: {resp}")
            return False
        
        # Step 5: Reboot
        print("\n[5/5] Rebooting to App...")
        send_command(ser, "boot_reboot")
        try:
            wait_response(ser, "boot_ack", timeout=2.0)
        except TimeoutError:
            pass  # Device may reboot before sending response
        
        print("\n✓ Firmware upload complete!")
        return True
        
    except Exception as e:
        print(f"\nError: {e}")
        return False
    finally:
        ser.close()

def main():
    parser = argparse.ArgumentParser(description='VectorFOC OTA Firmware Uploader')
    parser.add_argument('firmware', help='Firmware binary file (.bin)')
    parser.add_argument('--port', '-p', default='COM3', help='Serial port (default: COM3)')
    parser.add_argument('--baud', '-b', type=int, default=115200, help='Baud rate (default: 115200)')
    
    args = parser.parse_args()
    
    success = upload_firmware(args.port, args.firmware, args.baud)
    sys.exit(0 if success else 1)

if __name__ == '__main__':
    main()
