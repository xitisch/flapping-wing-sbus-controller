"""Protocol definitions shared by the GUI and headless data logger."""

from __future__ import annotations

from dataclasses import dataclass
import re


HOST_PROTOCOL_VERSION = 3
TRANSMITTER_DEVICE_ID = "FLAPPING_WING_TRANSMITTER"
CONTROL_CHANNEL_NAMES = ("ch1", "ch2", "ch3", "ch5", "ch6", "ch8")
SAFE_CHANNELS = (1500, 1500, 1000, 1500, 1500, 1000)

IDENTITY_RE = re.compile(
    r"^DEVICE:([^;]+);PROTOCOL:(\d+);RADIO:([01])$"
)
PONG_RE = re.compile(
    r"^PONG;PROTOCOL:(\d+);HOST_FAILSAFE:([01]);RADIO:([01]);"
    r"UPTIME_MS:(\d+)$"
)
RECEIVER_STATUS_RE = re.compile(
    r"^RECEIVER_STATUS mac=([0-9A-Fa-f:]{17}) packets=(\d+) "
    r"sequence=(\d+) link=([01]) failsafe=([01]) "
    r"ch1=(\d+) ch2=(\d+) ch3=(\d+) ch5=(\d+) ch6=(\d+) ch8=(\d+) "
    r"battery_raw=(\d+) battery_pin_mv=(\d+)$"
)


@dataclass(frozen=True)
class ReceiverStatus:
    """One receiver-confirmed status sample returned through USB serial."""

    mac: str
    packets: int
    sequence: int
    link_active: bool
    failsafe: bool
    channels: tuple[int, int, int, int, int, int]
    battery_raw: int
    battery_pin_mv: int

    def battery_voltage(self, divider_ratio: float = 4.0) -> float:
        return self.battery_pin_mv / 1000.0 * divider_ratio


def normalize_channels(values) -> tuple[int, int, int, int, int, int]:
    """Validate and clamp a six-value PC command."""

    if len(values) != len(CONTROL_CHANNEL_NAMES):
        raise ValueError("exactly six channel values are required")
    return tuple(max(1000, min(2000, int(value))) for value in values)


def encode_channel_command(values) -> bytes:
    """Encode six host-unit channel values as one newline-ended ASCII command."""

    normalized = normalize_channels(values)
    return (
        "<" + ",".join(str(value) for value in normalized) + ">\n"
    ).encode("ascii")


def parse_receiver_status(line: str) -> ReceiverStatus | None:
    """Parse a protocol-v3 RECEIVER_STATUS line, or return None."""

    match = RECEIVER_STATUS_RE.fullmatch(line)
    if match is None:
        return None

    groups = match.groups()
    return ReceiverStatus(
        mac=groups[0].upper(),
        packets=int(groups[1]),
        sequence=int(groups[2]),
        link_active=groups[3] == "1",
        failsafe=groups[4] == "1",
        channels=tuple(int(value) for value in groups[5:11]),
        battery_raw=int(groups[11]),
        battery_pin_mv=int(groups[12]),
    )


class SerialLineBuffer:
    """Turn arbitrary serial byte chunks into complete ASCII lines."""

    def __init__(self, maximum_bytes: int = 8192):
        self.maximum_bytes = maximum_bytes
        self.buffer = bytearray()

    def clear(self) -> None:
        self.buffer.clear()

    def feed(self, data: bytes) -> list[str]:
        self.buffer.extend(data)
        if len(self.buffer) > self.maximum_bytes:
            self.buffer.clear()
            raise ValueError("serial receive buffer exceeded safety limit")

        lines = []
        while b"\n" in self.buffer:
            raw_line, _, remainder = self.buffer.partition(b"\n")
            self.buffer[:] = remainder
            line = raw_line.decode("ascii", errors="replace").strip("\r ").strip()
            if line:
                lines.append(line)
        return lines
