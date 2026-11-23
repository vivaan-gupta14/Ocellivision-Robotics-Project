#!/usr/bin/env python
"""Ping every servo ID (0-252) at every common Feetech baudrate on a port.

Use this when a motor bus connect fails with "found motor list: {}" — it
tells you whether the motors are alive at all, and if so, at what baudrate
and IDs, without needing any lerobot calibration config.

Usage: scan_motors.py <port>
"""

import argparse
import sys

from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

# Feetech STS/SCS servos support these standard baudrates.
BAUDRATES = [1_000_000, 500_000, 250_000, 128_000, 115_200, 57_600, 38_400, 19_200, 9_600]

# Protocol end 0 = SCS (used by STS/SMS servos, e.g. the SO-101's).
PROTOCOL_END = 0


def scan(port: str, baudrate: int) -> list[tuple[int, int]]:
    port_handler = PortHandler(port)
    if not port_handler.openPort():
        print(f"  Failed to open port {port}")
        return []

    found = []
    try:
        if not port_handler.setBaudRate(baudrate):
            print(f"  Failed to set baudrate {baudrate}")
            return []

        packet_handler = PacketHandler(PROTOCOL_END)
        for motor_id in range(253):
            model_number, comm_result, _ = packet_handler.ping(port_handler, motor_id)
            if comm_result == COMM_SUCCESS:
                found.append((motor_id, model_number))
    finally:
        port_handler.closePort()

    return found


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("port", help="Serial port, e.g. /dev/ttyACM0")
    args = parser.parse_args()

    any_found = False
    for baudrate in BAUDRATES:
        print(f"Scanning {args.port} @ {baudrate} ...")
        found = scan(args.port, baudrate)
        if found:
            any_found = True
            for motor_id, model_number in found:
                print(f"  [ID:{motor_id:03d}] found, model number: {model_number}")
        else:
            print("  nothing")

    if not any_found:
        print("\nNo servos responded at any baudrate/ID. Check power and wiring.")
        sys.exit(1)


if __name__ == "__main__":
    main()
