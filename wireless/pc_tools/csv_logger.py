"""CSV logging for receiver-confirmed channels and battery telemetry."""

from __future__ import annotations

import csv
from datetime import datetime
from pathlib import Path
import time

from .protocol import CONTROL_CHANNEL_NAMES, ReceiverStatus, normalize_channels


CSV_FIELDS = (
    "record_type",
    "pc_time",
    "elapsed_s",
    "event",
    "receiver_mac",
    "receiver_packets",
    "confirmed_sequence",
    "link",
    "failsafe",
    *(f"commanded_{name}" for name in CONTROL_CHANNEL_NAMES),
    *(f"confirmed_{name}" for name in CONTROL_CHANNEL_NAMES),
    "command_matches_confirmation",
    "battery_raw",
    "battery_pin_mv",
    "battery_voltage_v",
)


def default_log_path(directory: str | Path = "logs") -> Path:
    timestamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
    return Path(directory) / f"flight-log-{timestamp}.csv"


class LiveCsvLogger:
    """Append and flush timestamped samples while a controller is running."""

    def __init__(self, divider_ratio: float = 4.0):
        self.divider_ratio = float(divider_ratio)
        self.path: Path | None = None
        self.row_count = 0
        self.sample_count = 0
        self._started_at = 0.0
        self._file = None
        self._writer = None

    @property
    def active(self) -> bool:
        return self._file is not None

    def start(self, path: str | Path) -> Path:
        if self.active:
            raise RuntimeError("a recording is already active")

        destination = Path(path).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        file_handle = destination.open("w", newline="", encoding="utf-8")
        try:
            writer = csv.DictWriter(file_handle, fieldnames=CSV_FIELDS)
            writer.writeheader()
            file_handle.flush()
        except Exception:
            file_handle.close()
            raise

        self.path = destination
        self.row_count = 0
        self.sample_count = 0
        self._started_at = time.monotonic()
        self._file = file_handle
        self._writer = writer
        return destination

    def _base_row(self, record_type: str, event: str = "") -> dict:
        row = {field: "" for field in CSV_FIELDS}
        row.update({
            "record_type": record_type,
            "pc_time": datetime.now().astimezone().isoformat(
                timespec="milliseconds"
            ),
            "elapsed_s": f"{time.monotonic() - self._started_at:.3f}",
            "event": event,
        })
        return row

    def _write(self, row: dict) -> None:
        if not self.active or self._writer is None or self._file is None:
            return
        try:
            self._writer.writerow(row)
            self._file.flush()
        except Exception:
            self._file.close()
            self._file = None
            self._writer = None
            raise
        self.row_count += 1

    @staticmethod
    def _add_commanded(row: dict, commanded_channels) -> tuple:
        values = normalize_channels(commanded_channels)
        for name, value in zip(CONTROL_CHANNEL_NAMES, values):
            row[f"commanded_{name}"] = value
        return values

    def log_event(self, event: str, commanded_channels=None) -> None:
        if not self.active:
            return
        row = self._base_row("event", event)
        if commanded_channels is not None:
            self._add_commanded(row, commanded_channels)
        self._write(row)

    def log_receiver_status(
        self, status: ReceiverStatus, commanded_channels
    ) -> None:
        if not self.active:
            return

        row = self._base_row("receiver_status")
        commanded = self._add_commanded(row, commanded_channels)
        for name, value in zip(CONTROL_CHANNEL_NAMES, status.channels):
            row[f"confirmed_{name}"] = value
        row.update({
            "receiver_mac": status.mac,
            "receiver_packets": status.packets,
            "confirmed_sequence": status.sequence,
            "link": int(status.link_active),
            "failsafe": int(status.failsafe),
            "command_matches_confirmation": int(
                commanded == status.channels
            ),
            "battery_raw": status.battery_raw,
            "battery_pin_mv": status.battery_pin_mv,
            "battery_voltage_v": f"{status.battery_voltage(self.divider_ratio):.6f}",
        })
        self._write(row)
        self.sample_count += 1

    def stop(self, event: str = "recording_stopped",
             commanded_channels=None) -> Path | None:
        if not self.active:
            return self.path
        try:
            self.log_event(event, commanded_channels)
        finally:
            if self._file is not None:
                self._file.flush()
                self._file.close()
            self._file = None
            self._writer = None
        return self.path
