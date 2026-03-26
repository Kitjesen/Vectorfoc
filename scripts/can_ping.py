"""
CAN ping test for VectorFOC / X-STAR-S (inovxio protocol)

CAN ID format (29-bit extended):
  bits[28:24] = cmd_type
  bits[23:8]  = data
  bits[7:0]   = target_id

Commands used:
  GET_ID  (0)  -> send to device 0x01; firmware replies with:
                  cmd=0, data=node_id (0x01), target=0xFE (broadcast)
                  payload: 8 bytes of STM32 hardware UID
  GET_VER (26) -> ask firmware version
"""

import can
import struct
import time

DEVICE_ID   = 0x01      # default CAN ID on firmware
PCAN_CH     = "PCAN_USBBUS1"
BAUDRATE    = 1_000_000

CMD_GET_ID      = 0
CMD_FEEDBACK    = 2
CMD_GET_VERSION = 26
CMD_ENABLE      = 3
CMD_STOP        = 4

def make_id(cmd, data=0, target=DEVICE_ID):
    return ((cmd & 0x1F) << 24) | ((data & 0xFFFF) << 8) | (target & 0xFF)

def parse_id(arb_id):
    cmd    = (arb_id >> 24) & 0x1F
    data   = (arb_id >> 8)  & 0xFFFF
    dev_id = arb_id & 0xFF
    return cmd, data, dev_id

def ping(bus):
    """Send GET_ID, wait for response, return True if got reply."""
    tx = can.Message(
        arbitration_id = make_id(CMD_GET_ID, target=DEVICE_ID),
        is_extended_id = True,
        data = [],
    )
    bus.send(tx)
    print(f"  TX  ID=0x{tx.arbitration_id:08X}  GET_ID -> device 0x{DEVICE_ID:02X}")

    deadline = time.time() + 0.5
    while time.time() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        cmd, data, dev_id = parse_id(msg.arbitration_id)
        print(f"  RX  ID=0x{msg.arbitration_id:08X}  cmd={cmd} dev=0x{dev_id:02X}  data={msg.data.hex()}")
        # Firmware GET_ID response: cmd=0, target=0xFE, data=node_id
        if cmd == CMD_GET_ID and dev_id == 0xFE:
            node_id = data & 0xFF
            uid_hex = msg.data.hex() if msg.data else "(no payload)"
            print(f"  *** PONG  node_id=0x{node_id:02X}  UID={uid_hex}")
            return True
    return False

def get_version(bus):
    tx = can.Message(
        arbitration_id = make_id(CMD_GET_VERSION, target=DEVICE_ID),
        is_extended_id = True,
        data = [],
    )
    bus.send(tx)
    print(f"  TX  GET_VERSION -> device 0x{DEVICE_ID:02X}")
    deadline = time.time() + 0.5
    while time.time() < deadline:
        msg = bus.recv(timeout=0.1)
        if msg is None:
            continue
        cmd, _, dev_id = parse_id(msg.arbitration_id)
        if dev_id == DEVICE_ID or dev_id == 0xFD:
            print(f"  RX  ID=0x{msg.arbitration_id:08X}  data={msg.data.hex()}  cmd={cmd}")

def main():
    print(f"Connecting to PCAN: channel={PCAN_CH}, bitrate={BAUDRATE//1000}kbps ...")
    try:
        bus = can.Bus(interface="pcan", channel=PCAN_CH, bitrate=BAUDRATE)
    except Exception as e:
        print(f"ERROR: cannot open PCAN: {e}")
        print("Tip: make sure PCAN driver is installed and device is connected.")
        return

    print("Connected.\n")

    print("=== Ping (GET_ID) ===")
    ok = ping(bus)
    if ok:
        print("  -> Board is alive!\n")
    else:
        print("  -> No response (check CAN wiring / termination / baud rate)\n")

    print("=== GET_VERSION ===")
    get_version(bus)

    print("\n=== Sniff for 3s (any traffic) ===")
    end = time.time() + 3
    while time.time() < end:
        msg = bus.recv(timeout=0.2)
        if msg:
            cmd, data, dev = parse_id(msg.arbitration_id)
            print(f"  ID=0x{msg.arbitration_id:08X}  cmd={cmd:2d}  dev=0x{dev:02X}  {msg.data.hex()}")

    bus.shutdown()
    print("Done.")

if __name__ == "__main__":
    main()
