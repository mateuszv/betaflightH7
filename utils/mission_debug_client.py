#!/usr/bin/env python3

import argparse
import struct
import time

import serial


MSP2_MISSION_DEBUG = 0x3012
MSP2_REQUEST_PREAMBLE = b"$X<"
MSP2_RESPONSE_PREAMBLES = (b"$X>", b"$X!")


def crc8_dvb_s2(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0xD5) & 0xFF if crc & 0x80 else (crc << 1) & 0xFF
    return crc


def read_exact(port: serial.Serial, length: int) -> bytes:
    data = port.read(length)
    if len(data) != length:
        raise TimeoutError("timeout while reading MSP response")
    return data


def send_request(port: serial.Serial) -> None:
    header = struct.pack("<BHH", 0, MSP2_MISSION_DEBUG, 0)
    port.write(MSP2_REQUEST_PREAMBLE + header + bytes((crc8_dvb_s2(header),)))


def read_response(port: serial.Serial) -> bytes:
    sync = bytearray()
    while bytes(sync[-3:]) not in MSP2_RESPONSE_PREAMBLES:
        sync.extend(read_exact(port, 1))

    preamble = bytes(sync[-3:])
    header = read_exact(port, 5)
    _, command, payload_length = struct.unpack("<BHH", header)
    payload = read_exact(port, payload_length)
    received_crc = read_exact(port, 1)[0]

    if crc8_dvb_s2(header + payload) != received_crc:
        raise ValueError("invalid MSP checksum")
    if preamble == b"$X!":
        raise RuntimeError(f"FC rejected MSP command 0x{command:04X}")
    if command != MSP2_MISSION_DEBUG:
        raise ValueError(f"unexpected MSP command 0x{command:04X}")
    return payload


def decode_payload(payload: bytes):
    if len(payload) < 7:
        raise ValueError("mission debug response is too short")

    version, message_present, queued, dropped = struct.unpack_from("<BBBI", payload)
    if version != 1:
        raise ValueError(f"unsupported mission debug version {version}")
    if not message_present:
        return queued, dropped, None
    if len(payload) < 12:
        raise ValueError("mission debug message header is too short")

    timestamp_ms, text_length = struct.unpack_from("<IB", payload, 7)
    if len(payload) != 12 + text_length:
        raise ValueError("invalid mission debug text length")
    text = payload[12:].decode("utf-8", errors="replace")
    return queued, dropped, (timestamp_ms, text)


def main() -> None:
    parser = argparse.ArgumentParser(description="Read MSP mission debug messages from Betaflight")
    parser.add_argument("port", help="USB VCP port, for example /dev/ttyACM0 or COM5")
    parser.add_argument("--interval", type=float, default=0.05, help="poll interval in seconds")
    args = parser.parse_args()

    with serial.Serial(args.port, 115200, timeout=1) as port:
        while True:
            send_request(port)
            queued, dropped, message = decode_payload(read_response(port))
            if message:
                timestamp_ms, text = message
                print(f"[{timestamp_ms:10d} ms] {text} (queued={queued}, dropped={dropped})", flush=True)
            time.sleep(0 if queued else args.interval)


if __name__ == "__main__":
    main()
