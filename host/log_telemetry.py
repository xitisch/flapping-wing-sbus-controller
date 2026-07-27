"""Headless receiver-confirmed channel and voltage recorder.

Example:
    py -m host.log_telemetry --port COM10 --duration 60

The GUI and this process cannot use the transmitter COM port simultaneously.
"""

import argparse
from pathlib import Path
import sys
import time

import serial
import serial.tools.list_ports

if __package__:
    from .pc_tools import (
        LiveCsvLogger,
        SAFE_CHANNELS,
        SerialLineBuffer,
        default_log_path,
        encode_channel_command,
        identify_transmitter,
        open_transmitter_serial,
        parse_receiver_status,
        read_serial_lines,
    )
else:
    from pc_tools import (
        LiveCsvLogger,
        SAFE_CHANNELS,
        SerialLineBuffer,
        default_log_path,
        encode_channel_command,
        identify_transmitter,
        open_transmitter_serial,
        parse_receiver_status,
        read_serial_lines,
    )


CONTROL_PERIOD_SECONDS = 0.100


def parse_arguments():
    parser = argparse.ArgumentParser(
        description=(
            "Record receiver-confirmed channels and battery voltage to CSV."
        )
    )
    parser.add_argument("--port", help="transmitter COM port, for example COM10")
    parser.add_argument(
        "--output", type=Path,
        help="CSV destination (default: logs/flight-log-<timestamp>.csv)"
    )
    parser.add_argument(
        "--duration", type=float, default=0.0,
        help="recording duration in seconds; zero records until Ctrl+C"
    )
    parser.add_argument(
        "--list-ports", action="store_true",
        help="list serial ports and exit"
    )
    return parser.parse_args()


def list_ports():
    for port in serial.tools.list_ports.comports():
        print(f"{port.device:8} {port.description or 'Unknown device'}")


def run(args):
    if not args.port:
        raise ValueError("--port is required (use --list-ports to inspect ports)")
    if args.duration < 0:
        raise ValueError("--duration cannot be negative")

    output = args.output or default_log_path()
    channels = list(SAFE_CHANNELS)
    safe_command = encode_channel_command(channels)
    line_buffer = SerialLineBuffer()
    logger = LiveCsvLogger()

    with open_transmitter_serial(args.port) as ser:
        identify_transmitter(ser, line_buffer)
        destination = logger.start(output)
        logger.log_event("recording_started", channels)
        print(f"Recording to {destination}")
        print("Press Ctrl+C to stop safely.")

        started_at = time.monotonic()
        next_control = started_at
        try:
            while args.duration == 0 or (
                    time.monotonic() - started_at < args.duration):
                now = time.monotonic()
                if now >= next_control:
                    ser.write(safe_command)
                    next_control = now + CONTROL_PERIOD_SECONDS

                for line in read_serial_lines(ser, line_buffer):
                    status = parse_receiver_status(line)
                    if status is None:
                        continue
                    logger.log_receiver_status(status, channels)
                    print(
                        "\r"
                        f"seq={status.sequence} "
                        f"CH={status.channels} "
                        f"battery={status.battery_voltage():.3f} V "
                        f"link={int(status.link_active)} "
                        f"failsafe={int(status.failsafe)} "
                        f"samples={logger.sample_count}",
                        end="",
                        flush=True,
                    )
                time.sleep(0.005)
        except KeyboardInterrupt:
            print("\nStop requested.")
        finally:
            channels[-1] = 1000
            safe_command = encode_channel_command(channels)
            for _ in range(3):
                try:
                    ser.write(safe_command)
                except serial.SerialException:
                    break
                time.sleep(0.05)
            logger.stop("recording_stopped", channels)
            print(f"\nSaved {logger.sample_count} confirmed samples to {logger.path}")


def main():
    args = parse_arguments()
    if args.list_ports:
        list_ports()
        return
    try:
        run(args)
    except (OSError, ValueError, RuntimeError, TimeoutError,
            serial.SerialException) as error:
        raise SystemExit(f"Logging failed: {error}") from error


if __name__ == "__main__":
    main()
