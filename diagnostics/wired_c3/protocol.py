"""USB protocol helpers for the one-board ESP32-C3 wired diagnostic."""

from __future__ import annotations

from dataclasses import dataclass
import re


WIRED_DEVICE_ID = "FLAPPING_WING_WIRED_C3"
WIRED_PROTOCOL_VERSION = 1

IDENTITY_RE = re.compile(r"^DEVICE:([^;]+);PROTOCOL:(\d+)$")
STATUS_RE = re.compile(
    r"^WIRED_STATUS sequence=(\d+) link=([01]) failsafe=([01]) "
    r"ch1=(\d+) ch2=(\d+) ch3=(\d+) ch5=(\d+) ch6=(\d+) ch8=(\d+) "
    r"battery_raw=(\d+) battery_pin_mv=(\d+)$"
)


@dataclass(frozen=True)
class WiredStatus:
    sequence: int
    link_active: bool
    failsafe: bool
    channels: tuple[int, int, int, int, int, int]
    battery_raw: int
    battery_pin_mv: int

    @property
    def battery_voltage(self) -> float:
        return self.battery_pin_mv / 1000.0 * 4.0


def identity_is_compatible(line: str) -> bool:
    match = IDENTITY_RE.fullmatch(line)
    if match is None:
        return False
    device_id, version = match.groups()
    return (
        device_id == WIRED_DEVICE_ID
        and int(version) == WIRED_PROTOCOL_VERSION
    )


def parse_wired_status(line: str) -> WiredStatus | None:
    match = STATUS_RE.fullmatch(line)
    if match is None:
        return None
    values = match.groups()
    return WiredStatus(
        sequence=int(values[0]),
        link_active=values[1] == "1",
        failsafe=values[2] == "1",
        channels=tuple(int(value) for value in values[3:9]),
        battery_raw=int(values[9]),
        battery_pin_mv=int(values[10]),
    )
