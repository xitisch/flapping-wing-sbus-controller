import csv
from pathlib import Path
import tempfile
import unittest

from host.pc_tools.csv_logger import (
    DEFAULT_LOG_DIRECTORY,
    LiveCsvLogger,
    log_path_from_filename,
    validate_log_filename,
)
from host.pc_tools.protocol import (
    BATTERY_DIVIDER_RATIO,
    ReceiverStatus,
    SerialLineBuffer,
    encode_channel_command,
    parse_receiver_status,
    parse_transmitter_health,
    parse_transmitter_identity,
    transmitter_health_error,
    transmitter_identity_error,
)
from host.pc_tools.serial_session import (
    read_serial_lines,
    write_serial_line,
)
from diagnostics.wired_c3.protocol import (
    identity_is_compatible,
    parse_wired_status,
)


STATUS_LINE = (
    "RECEIVER_STATUS mac=AA:BB:CC:DD:EE:FF packets=1234 "
    "link=1 failsafe=0 battery_raw=1800 battery_pin_mv=1100"
)


class ProtocolTests(unittest.TestCase):
    def test_transmitter_identity_validation(self):
        identity = parse_transmitter_identity(
            "DEVICE:FLAPPING_WING_TRANSMITTER;PROTOCOL:2;RADIO:1"
        )
        self.assertIsNotNone(identity)
        self.assertIsNone(transmitter_identity_error(identity))

        wrong_protocol = parse_transmitter_identity(
            "DEVICE:FLAPPING_WING_TRANSMITTER;PROTOCOL:3;RADIO:1"
        )
        self.assertIsNotNone(wrong_protocol)
        self.assertIn(
            "incompatible", transmitter_identity_error(wrong_protocol)
        )

    def test_transmitter_health_validation(self):
        health = parse_transmitter_health(
            "PONG;PROTOCOL:2;HOST_FAILSAFE:1;RADIO:1;UPTIME_MS:12345"
        )
        self.assertIsNotNone(health)
        self.assertTrue(health.host_failsafe)
        self.assertEqual(health.uptime_ms, 12345)
        self.assertIsNone(transmitter_health_error(health))

    def test_channel_command_is_clamped_and_ordered(self):
        encoded = encode_channel_command((999, 1500, 1200, 1501, 1502, 2001))
        self.assertEqual(
            encoded, b"<1000,1500,1200,1501,1502,2000>\n"
        )

    def test_receiver_status_parser(self):
        status = parse_receiver_status(STATUS_LINE)
        self.assertIsNotNone(status)
        self.assertEqual(status.mac, "AA:BB:CC:DD:EE:FF")
        self.assertEqual(status.packets, 1234)
        self.assertTrue(status.link_active)
        self.assertAlmostEqual(status.battery_voltage(), 4.4)

    def test_line_buffer_preserves_partial_input(self):
        buffer = SerialLineBuffer()
        self.assertEqual(buffer.feed(b"first"), [])
        self.assertEqual(
            buffer.feed(b" line\nsecond line\npartial"),
            ["first line", "second line"],
        )
        self.assertEqual(buffer.feed(b" end\n"), ["partial end"])

    def test_default_battery_divider_ratio(self):
        self.assertEqual(BATTERY_DIVIDER_RATIO, 4.0)


class FakeSerial:
    def __init__(self, incoming=b""):
        self.incoming = bytearray(incoming)
        self.written = bytearray()

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, count):
        data = bytes(self.incoming[:count])
        del self.incoming[:count]
        return data

    def write(self, data):
        self.written.extend(data)
        return len(data)


class SerialSessionTests(unittest.TestCase):
    def test_read_serial_lines_uses_shared_buffer(self):
        connection = FakeSerial(b"one\ntwo")
        buffer = SerialLineBuffer()
        self.assertEqual(read_serial_lines(connection, buffer), ["one"])

        connection.incoming.extend(b" complete\n")
        self.assertEqual(
            read_serial_lines(connection, buffer), ["two complete"]
        )

    def test_write_serial_line_adds_one_newline(self):
        connection = FakeSerial()
        write_serial_line(connection, "PING")
        self.assertEqual(connection.written, b"PING\n")
        with self.assertRaises(ValueError):
            write_serial_line(connection, "PING\n")


class RepositoryStructureTests(unittest.TestCase):
    def test_esp_now_header_has_one_authoritative_copy(self):
        repository = Path(__file__).resolve().parents[1]
        headers = sorted(
            path.relative_to(repository).as_posix()
            for path in (repository / "firmware").rglob("esp_now_link.h")
            if ".pio" not in path.parts
        )
        self.assertEqual(headers, ["firmware/link/esp_now_link.h"])

    def test_wireless_telemetry_is_battery_only(self):
        repository = Path(__file__).resolve().parents[1]
        header = (
            repository / "firmware" / "link" / "esp_now_link.h"
        ).read_text(encoding="utf-8")
        self.assertIn("battery_adc_raw", header)
        self.assertIn("battery_pin_mv", header)
        self.assertNotIn("last_sequence", header)
        self.assertNotIn("applied_ch", header)


class WiredDiagnosticProtocolTests(unittest.TestCase):
    def test_wired_identity_is_distinct_and_versioned(self):
        self.assertTrue(
            identity_is_compatible(
                "DEVICE:FLAPPING_WING_WIRED_C3;PROTOCOL:1"
            )
        )
        self.assertFalse(
            identity_is_compatible(
                "DEVICE:FLAPPING_WING_TRANSMITTER;PROTOCOL:2"
            )
        )

    def test_wired_status_includes_gpio3_voltage(self):
        status = parse_wired_status(
            "WIRED_STATUS sequence=42 link=1 failsafe=0 "
            "ch1=1500 ch2=1500 ch3=1000 ch5=1500 ch6=1500 ch8=1000 "
            "battery_raw=1800 battery_pin_mv=1100"
        )
        self.assertIsNotNone(status)
        self.assertEqual(
            status.channels, (1500, 1500, 1000, 1500, 1500, 1000)
        )
        self.assertAlmostEqual(status.battery_voltage, 4.4)


class CsvLoggerTests(unittest.TestCase):
    def test_default_log_directory_is_repository_logs_folder(self):
        repository = Path(__file__).resolve().parents[1]
        self.assertEqual(DEFAULT_LOG_DIRECTORY, repository / "logs")

    def test_log_filename_is_validated_and_confined(self):
        self.assertEqual(validate_log_filename("experiment"), "experiment.csv")
        self.assertEqual(
            log_path_from_filename("experiment.csv"),
            DEFAULT_LOG_DIRECTORY / "experiment.csv",
        )
        for invalid in (
            "",
            "../escape.csv",
            r"folder\escape.csv",
            "bad?.csv",
            "CON.csv",
            "COM1.session.csv",
            "not-csv.txt",
        ):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    validate_log_filename(invalid)

    def test_existing_log_is_not_overwritten(self):
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "existing.csv"
            destination.write_text("original", encoding="utf-8")
            logger = LiveCsvLogger()
            with self.assertRaises(FileExistsError):
                logger.start(destination)
            self.assertEqual(
                destination.read_text(encoding="utf-8"), "original"
            )

    def test_command_and_battery_sample_is_written(self):
        commanded = (1500, 1500, 1000, 1500, 1500, 1000)
        status = ReceiverStatus(
            mac="AA:BB:CC:DD:EE:FF",
            packets=10,
            link_active=True,
            failsafe=False,
            battery_raw=1800,
            battery_pin_mv=1100,
        )
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "sample.csv"
            logger = LiveCsvLogger()
            logger.start(destination)
            logger.log_receiver_status(status, commanded)
            logger.stop()

            with destination.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(rows[0]["record_type"], "receiver_status")
        self.assertEqual(rows[0]["commanded_ch3"], "1000")
        self.assertNotIn("confirmed_ch3", rows[0])
        self.assertEqual(rows[0]["battery_voltage_v"], "4.400000")
        self.assertEqual(rows[1]["record_type"], "event")


if __name__ == "__main__":
    unittest.main()
