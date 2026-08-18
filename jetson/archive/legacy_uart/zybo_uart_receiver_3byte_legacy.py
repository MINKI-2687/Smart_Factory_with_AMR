#!/usr/bin/env python3
import serial

PORT = "/dev/ttyTHS1"
SOF = b"\xAA\x55"
TYPE_TARGET = 0x01
TYPE_ACK = 0x02
FRAME_SIZE = 9


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0x07) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc


def make_ack(sequence):
    payload = bytes([TYPE_ACK, sequence])
    return SOF + payload + bytes([crc8(payload)])


uart = serial.Serial(
    port=PORT,
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.02,
    write_timeout=0.2,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
)

rx_buffer = bytearray()

print(f"UART ready: {PORT}, 115200 8-N-1")
print("Automatic ACK: ON")
print("Waiting for Zybo TARGET...")

try:
    while True:
        data = uart.read(uart.in_waiting or 1)

        if not data:
            continue

        print("RAW RX:", data.hex(" ").upper())
        rx_buffer.extend(data)

        while True:
            header = rx_buffer.find(SOF)

            if header < 0:
                if rx_buffer[-1:] == b"\xAA":
                    rx_buffer[:] = b"\xAA"
                else:
                    rx_buffer.clear()
                break

            if header > 0:
                del rx_buffer[:header]

            if len(rx_buffer) < FRAME_SIZE:
                break

            frame = bytes(rx_buffer[:FRAME_SIZE])
            expected_crc = crc8(frame[2:8])

            if frame[2] != TYPE_TARGET or frame[8] != expected_crc:
                print(
                    "INVALID:",
                    frame.hex(" ").upper(),
                    f"EXPECTED_CRC={expected_crc:02X}",
                )
                del rx_buffer[0]
                continue

            del rx_buffer[:FRAME_SIZE]

            sequence = frame[3]
            x = int.from_bytes(frame[4:6], "little")
            y = int.from_bytes(frame[6:8], "little")

            ack = make_ack(sequence)
            uart.write(ack)
            uart.flush()

            print(f"TARGET OK: SEQ={sequence} X={x} Y={y}")
            print("ACK TX:", ack.hex(" ").upper())

except KeyboardInterrupt:
    print("\nStopped")

finally:
    uart.close()
