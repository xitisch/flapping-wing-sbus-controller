import csv
from pathlib import Path
import tempfile
import unittest

from wireless.pc_tools.csv_logger import LiveCsvLogger
from wireless.pc_tools.protocol import (
    ReceiverStatus,
    SerialLineBuffer,
    encode_channel_command,
    parse_receiver_status,
)


STATUS_LINE = (
    "RECEIVER_STATUS mac=AA:BB:CC:DD:EE:FF packets=1234 "
    "sequence=77 link=1 failsafe=0 "
    "ch1=1500 ch2=1499 ch3=1200 ch5=1501 ch6=1502 ch8=1000 "
    "battery_raw=1800 battery_pin_mv=1100"
)


class ProtocolTests(unittest.TestCase):
    def test_channel_command_is_clamped_and_ordered(self):
        encoded = encode_channel_command((999, 1500, 1200, 1501, 1502, 2001))
        self.assertEqual(
            encoded, b"<1000,1500,1200,1501,1502,2000>\n"
        )

    def test_receiver_status_parser(self):
        status = parse_receiver_status(STATUS_LINE)
        self.assertIsNotNone(status)
        self.assertEqual(status.mac, "AA:BB:CC:DD:EE:FF")
        self.assertEqual(status.sequence, 77)
        self.assertEqual(
            status.channels, (1500, 1499, 1200, 1501, 1502, 1000)
        )
        self.assertAlmostEqual(status.battery_voltage(), 4.4)

    def test_line_buffer_preserves_partial_input(self):
        buffer = SerialLineBuffer()
        self.assertEqual(buffer.feed(b"first"), [])
        self.assertEqual(
            buffer.feed(b" line\nsecond line\npartial"),
            ["first line", "second line"],
        )
        self.assertEqual(buffer.feed(b" end\n"), ["partial end"])


class CsvLoggerTests(unittest.TestCase):
    def test_receiver_confirmed_sample_is_written(self):
        status = ReceiverStatus(
            mac="AA:BB:CC:DD:EE:FF",
            packets=10,
            sequence=7,
            link_active=True,
            failsafe=False,
            channels=(1500, 1500, 1000, 1500, 1500, 1000),
            battery_raw=1800,
            battery_pin_mv=1100,
        )
        with tempfile.TemporaryDirectory() as directory:
            destination = Path(directory) / "sample.csv"
            logger = LiveCsvLogger()
            logger.start(destination)
            logger.log_receiver_status(status, status.channels)
            logger.stop()

            with destination.open(newline="", encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))

        self.assertEqual(rows[0]["record_type"], "receiver_status")
        self.assertEqual(rows[0]["confirmed_sequence"], "7")
        self.assertEqual(rows[0]["confirmed_ch3"], "1000")
        self.assertEqual(rows[0]["command_matches_confirmation"], "1")
        self.assertEqual(rows[0]["battery_voltage_v"], "4.400000")
        self.assertEqual(rows[1]["record_type"], "event")


if __name__ == "__main__":
    unittest.main()
