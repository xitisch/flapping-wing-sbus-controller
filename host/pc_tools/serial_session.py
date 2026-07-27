"""Shared serial transport helpers for GUI and headless clients."""

from __future__ import annotations

import time

import serial

from .protocol import (
    SerialLineBuffer,
    parse_transmitter_identity,
    transmitter_identity_error,
)


SERIAL_BAUD_RATE = 115200
SERIAL_READ_CHUNK_BYTES = 4096
IDENTIFY_COMMAND = "IDENTIFY"
PING_COMMAND = "PING"


def open_transmitter_serial(port: str):
    """Open a transmitter COM port with the complete protocol settings."""

    connection = serial.Serial(
        port=port,
        baudrate=SERIAL_BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0,
        write_timeout=0.25,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )
    connection.reset_input_buffer()
    return connection


def write_serial_line(connection, text: str) -> int:
    """Write one newline-ended ASCII command."""

    if "\r" in text or "\n" in text:
        raise ValueError("serial command text must not contain line endings")
    return connection.write((text + "\n").encode("ascii"))


def read_serial_lines(
    connection,
    line_buffer: SerialLineBuffer,
) -> list[str]:
    """Read currently available bytes and return complete ASCII lines."""

    waiting = connection.in_waiting
    if not waiting:
        return []
    data = connection.read(min(waiting, SERIAL_READ_CHUNK_BYTES))
    return line_buffer.feed(data)


def identify_transmitter(
    connection,
    line_buffer: SerialLineBuffer | None = None,
    timeout_seconds: float = 5.0,
    request_period_seconds: float = 0.5,
):
    """Block until the expected transmitter identity is verified."""

    if line_buffer is None:
        line_buffer = SerialLineBuffer()
    deadline = time.monotonic() + timeout_seconds
    next_request = 0.0

    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_request:
            write_serial_line(connection, IDENTIFY_COMMAND)
            next_request = now + request_period_seconds

        for line in read_serial_lines(connection, line_buffer):
            identity = parse_transmitter_identity(line)
            if identity is None:
                continue
            error = transmitter_identity_error(identity)
            if error is not None:
                raise RuntimeError(error)
            return identity
        time.sleep(0.01)

    raise TimeoutError("compatible transmitter identity was not received")
