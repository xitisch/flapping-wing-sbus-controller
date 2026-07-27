"""Protocol definitions shared by the GUI and headless data logger."""

from __future__ import annotations

from dataclasses import dataclass
import re


HOST_PROTOCOL_VERSION = 2
TRANSMITTER_DEVICE_ID = "FLAPPING_WING_TRANSMITTER"
CONTROL_CHANNEL_NAMES = ("ch1", "ch2", "ch3", "ch5", "ch6", "ch8")
SAFE_CHANNELS = (1500, 1500, 1000, 1500, 1500, 1000)

# Battery divider: battery+ -- R_TOP -- GPIO3 -- R_BOTTOM -- GND.
BATTERY_R_TOP_OHMS = 300_000.0
BATTERY_R_BOTTOM_OHMS = 100_000.0
BATTERY_DIVIDER_RATIO = (
    BATTERY_R_TOP_OHMS + BATTERY_R_BOTTOM_OHMS
) / BATTERY_R_BOTTOM_OHMS
# ESP32-C3 ADC1 at 11 dB is documented for approximately 0-2.5 V.
BATTERY_ADC_NEAR_LIMIT_MV = 2450

IDENTITY_RE = re.compile(
    r"^DEVICE:([^;]+);PROTOCOL:(\d+);RADIO:([01])$"
)
PONG_RE = re.compile(
    r"^PONG;PROTOCOL:(\d+);HOST_FAILSAFE:([01]);RADIO:([01]);"
    r"UPTIME_MS:(\d+)$"
)
RECEIVER_STATUS_RE = re.compile(
    r"^RECEIVER_STATUS mac=([0-9A-Fa-f:]{17}) packets=(\d+) "
    r"link=([01]) failsafe=([01]) "
    r"battery_raw=(\d+) battery_pin_mv=(\d+)$"
)


@dataclass(frozen=True)
class TransmitterIdentity:
    """Identity returned by the USB-connected transmitter."""

    device_id: str
    protocol: int
    radio_ready: bool


@dataclass(frozen=True)
class TransmitterHealth:
    """Health state returned by a transmitter PONG line."""

    protocol: int
    host_failsafe: bool
    radio_ready: bool
    uptime_ms: int


@dataclass(frozen=True)
class ReceiverStatus:
    """One receiver health and battery sample returned through USB serial."""

    mac: str
    packets: int
    link_active: bool
    failsafe: bool
    battery_raw: int
    battery_pin_mv: int

    def battery_voltage(
        self, divider_ratio: float = BATTERY_DIVIDER_RATIO
    ) -> float:
        return self.battery_pin_mv / 1000.0 * divider_ratio


def parse_transmitter_identity(line: str) -> TransmitterIdentity | None:
    """Parse a transmitter identity line, or return None."""

    match = IDENTITY_RE.fullmatch(line)
    if match is None:
        return None
    device_id, protocol, radio_ready = match.groups()
    return TransmitterIdentity(
        device_id=device_id,
        protocol=int(protocol),
        radio_ready=radio_ready == "1",
    )


def transmitter_identity_error(
    identity: TransmitterIdentity,
) -> str | None:
    """Return why an identity is unsafe to use, or None when compatible."""

    if identity.device_id != TRANSMITTER_DEVICE_ID:
        return f"wrong serial device: {identity.device_id}"
    if identity.protocol != HOST_PROTOCOL_VERSION:
        return (
            f"protocol {identity.protocol} is incompatible; "
            f"version {HOST_PROTOCOL_VERSION} is required"
        )
    if not identity.radio_ready:
        return "transmitter ESP-NOW radio is not ready"
    return None


def parse_transmitter_health(line: str) -> TransmitterHealth | None:
    """Parse a transmitter PONG line, or return None."""

    match = PONG_RE.fullmatch(line)
    if match is None:
        return None
    protocol, host_failsafe, radio_ready, uptime_ms = match.groups()
    return TransmitterHealth(
        protocol=int(protocol),
        host_failsafe=host_failsafe == "1",
        radio_ready=radio_ready == "1",
        uptime_ms=int(uptime_ms),
    )


def transmitter_health_error(health: TransmitterHealth) -> str | None:
    """Return why a health response is incompatible, or None when valid."""

    if health.protocol != HOST_PROTOCOL_VERSION:
        return (
            f"protocol {health.protocol} is incompatible; "
            f"version {HOST_PROTOCOL_VERSION} is required"
        )
    if not health.radio_ready:
        return "transmitter ESP-NOW radio is not ready"
    return None


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
    """Parse a protocol-v2 RECEIVER_STATUS line, or return None."""

    match = RECEIVER_STATUS_RE.fullmatch(line)
    if match is None:
        return None

    groups = match.groups()
    return ReceiverStatus(
        mac=groups[0].upper(),
        packets=int(groups[1]),
        link_active=groups[2] == "1",
        failsafe=groups[3] == "1",
        battery_raw=int(groups[4]),
        battery_pin_mv=int(groups[5]),
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
