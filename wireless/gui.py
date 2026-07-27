"""Safety-oriented GUI for the wireless flapping-wing SBUS controller.

The GUI verifies the USB transmitter firmware before sending channel data,
keeps CH8 locked across every connection transition, monitors receiver
telemetry, and treats serial or wireless faults as reasons to stop tests and
lock locally. Warning banners and confirmations are non-blocking so the 100 ms
host heartbeat continues to run.
"""

from pathlib import Path
import time
import tkinter as tk
from tkinter import filedialog, ttk

try:
    import serial
    import serial.tools.list_ports
except ModuleNotFoundError as error:
    raise SystemExit(
        "pyserial is required. Install it with: py -m pip install pyserial"
    ) from error

if __package__:
    from .pc_tools import (
        HOST_PROTOCOL_VERSION,
        IDENTITY_RE,
        LiveCsvLogger,
        PONG_RE,
        TRANSMITTER_DEVICE_ID,
        default_log_path,
        parse_receiver_status,
    )
else:
    from pc_tools import (
        HOST_PROTOCOL_VERSION,
        IDENTITY_RE,
        LiveCsvLogger,
        PONG_RE,
        TRANSMITTER_DEVICE_ID,
        default_log_path,
        parse_receiver_status,
    )

BAUD_RATE = 115200
GUI_HEARTBEAT_MS = 100
CONNECTION_SERVICE_MS = 100
SERIAL_POLL_MS = 40
PORT_REFRESH_SECONDS = 1.5
HANDSHAKE_TIMEOUT_SECONDS = 5.0
IDENTIFY_INTERVAL_SECONDS = 0.5
PING_INTERVAL_SECONDS = 1.0
PONG_TIMEOUT_SECONDS = 3.0
RECONNECT_DELAY_SECONDS = 1.5
RECEIVER_INITIAL_TIMEOUT_SECONDS = 3.0
RECEIVER_STATUS_TIMEOUT_SECONDS = 1.25
CONFIRMATION_SECONDS = 4.0

# Battery divider: battery+ -- R_TOP -- GPIO3 -- R_BOTTOM -- GND.
BATTERY_R_TOP_OHMS = 300_000.0
BATTERY_R_BOTTOM_OHMS = 100_000.0
BATTERY_DIVIDER_RATIO = (
    BATTERY_R_TOP_OHMS + BATTERY_R_BOTTOM_OHMS
) / BATTERY_R_BOTTOM_OHMS
# ESP32-C3 ADC1 with 11 dB attenuation is specified for approximately 0-2.5 V.
BATTERY_ADC_NEAR_LIMIT_MV = 2450

THROTTLE_RAMP_RATE = 250.0
THROTTLE_RAMP_INTERVAL_MS = 50
TEST_STEP_MS = 8000
TEST_BREAK_MS = 5000
TEST_START_MS = 5000
TEST_BREAK_VALUES = {"ch3": 1000, "ch1": 1500, "ch2": 1500}

WIRELESS_TEST_STEPS = (
    ("Throttle armed; throttle 1000", TEST_START_MS,
     {"ch8": 2000, "ch3": 1000, "ch1": 1500, "ch2": 1500}),
    ("Throttle 1200", TEST_STEP_MS,
     {"ch8": 2000, "ch3": 1200, "ch1": 1500, "ch2": 1500}),
    ("Neutral break", TEST_BREAK_MS, TEST_BREAK_VALUES),
    ("Throttle 1600", TEST_STEP_MS, {"ch3": 1600}),
    ("Neutral break", TEST_BREAK_MS, TEST_BREAK_VALUES),
    ("Throttle 2000", TEST_STEP_MS, {"ch3": 2000}),
    ("Neutral break", TEST_BREAK_MS, TEST_BREAK_VALUES),
    ("Throttle 1200; yaw 1000", TEST_STEP_MS,
     {"ch3": 1200, "ch1": 1000}),
    ("Neutral break", TEST_BREAK_MS, TEST_BREAK_VALUES),
    ("Throttle 1200; yaw 1500", TEST_STEP_MS,
     {"ch3": 1200, "ch1": 1500}),
    ("Neutral break", TEST_BREAK_MS, TEST_BREAK_VALUES),
    ("Throttle 1200; yaw 2000", TEST_STEP_MS,
     {"ch3": 1200, "ch1": 2000}),
    ("Neutral break", TEST_BREAK_MS, TEST_BREAK_VALUES),
    ("Throttle 1200; pitch 1000", TEST_STEP_MS,
     {"ch3": 1200, "ch2": 1000}),
    ("Neutral break", TEST_BREAK_MS, TEST_BREAK_VALUES),
    ("Throttle 1200; pitch 1500", TEST_STEP_MS,
     {"ch3": 1200, "ch2": 1500}),
    ("Neutral break", TEST_BREAK_MS, TEST_BREAK_VALUES),
    ("Throttle 1200; pitch 2000", TEST_STEP_MS,
     {"ch3": 1200, "ch2": 2000}),
)

WIRELESS_TEST_FINAL_VALUES = {
    "ch1": 1500, "ch2": 1500, "ch3": 1000, "ch8": 1000
}

KNOWN_TRANSMITTER_CHIPS = ("cp210", "ch340", "ch341", "ftdi", "uart")


class ControllerGUI:
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.connection_state = "disconnected"
        self.connected_port = None
        self.connecting_port_info = None
        self.preferred_port_key = None
        self.auto_reconnect = True
        self.next_reconnect_at = 0.0
        self.handshake_deadline = 0.0
        self.next_identify_at = 0.0
        self.next_ping_at = 0.0
        self.last_pong_at = 0.0
        self.connected_at = 0.0
        self.next_port_refresh_at = 0.0
        self.serial_buffer = bytearray()
        self.port_by_label = {}

        self.receiver_records = {}
        self.receiver_online = False
        self.receiver_ever_seen = False
        self.last_receiver_mac = None
        self.last_receiver_packets = None
        self.data_logger = LiveCsvLogger(BATTERY_DIVIDER_RATIO)

        self.faults = {}
        self.status_message = ("Not connected", "gray")
        self.closing = False
        self.suppress_slider_send = False
        self.wireless_test_running = False
        self.wireless_test_after_id = None
        self.throttle_ramp_after_id = None
        self.unlock_confirm_until = 0.0
        self.test_confirm_until = 0.0

        self.build_ui()
        self.refresh_ports()
        self.refresh_control_states()

        self.root.protocol("WM_DELETE_WINDOW", self.close_gui)
        self.root.after(SERIAL_POLL_MS, self.poll_serial)
        self.root.after(CONNECTION_SERVICE_MS, self.service_connection)
        self.root.after(GUI_HEARTBEAT_MS, self.gui_heartbeat)
        self.root.after(300, self.auto_connect_initial)

    def build_ui(self):
        self.root.title("Flapping-Wing SBUS Controller")
        self.root.geometry("720x860")
        self.root.minsize(640, 780)

        tk.Label(
            self.root, text="ESP32 Flapping-Wing Controller",
            font=("Arial", 15, "bold")
        ).pack(pady=(10, 6))

        connection = ttk.LabelFrame(self.root, text="Connection")
        connection.pack(fill="x", padx=16, pady=(0, 8))

        port_row = tk.Frame(connection)
        port_row.pack(fill="x", padx=8, pady=6)
        tk.Label(port_row, text="Serial port:", width=11, anchor="w").pack(
            side="left"
        )
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            port_row, textvariable=self.port_var, state="readonly", width=48
        )
        self.port_combo.pack(side="left", fill="x", expand=True, padx=(0, 6))
        ttk.Button(
            port_row, text="Refresh", command=self.refresh_ports
        ).pack(side="left", padx=(0, 6))
        self.connect_btn = ttk.Button(
            port_row, text="Connect", command=self.toggle_connection
        )
        self.connect_btn.pack(side="left")

        self.connection_label = tk.Label(
            connection, text="Transmitter: disconnected", anchor="w", fg="red"
        )
        self.connection_label.pack(fill="x", padx=8)
        self.receiver_label = tk.Label(
            connection, text="Receiver: unavailable", anchor="w", fg="gray"
        )
        self.receiver_label.pack(fill="x", padx=8)
        self.confirmed_channels_label = tk.Label(
            connection,
            text="Confirmed channels: waiting for receiver telemetry",
            anchor="w", fg="gray"
        )
        self.confirmed_channels_label.pack(fill="x", padx=8)
        self.battery_panel = tk.Frame(
            connection, bg="#f5f5f5", bd=1, relief="groove"
        )
        self.battery_panel.pack(fill="x", padx=8, pady=(4, 7))

        self.battery_header = tk.Frame(self.battery_panel, bg="#f5f5f5")
        self.battery_header.pack(fill="x", padx=9, pady=(5, 0))
        self.battery_title_label = tk.Label(
            self.battery_header, text="BATTERY VOLTAGE", anchor="w",
            font=("Arial", 9, "bold"), bg="#f5f5f5", fg="#555555"
        )
        self.battery_title_label.pack(side="left")
        self.battery_voltage_label = tk.Label(
            self.battery_header, text="--.-- V", anchor="e",
            font=("Arial", 17, "bold"), bg="#f5f5f5", fg="gray"
        )
        self.battery_voltage_label.pack(side="right")
        self.battery_label = tk.Label(
            self.battery_panel,
            text="Waiting for receiver telemetry",
            anchor="w", bg="#f5f5f5", fg="gray"
        )
        self.battery_label.pack(fill="x", padx=9, pady=(0, 5))

        recording_row = tk.Frame(connection)
        recording_row.pack(fill="x", padx=8, pady=(0, 7))
        self.recording_btn = tk.Button(
            recording_row,
            text="START RECORDING",
            command=self.toggle_recording,
            font=("Arial", 9, "bold"),
            width=18,
        )
        self.recording_btn.pack(side="left", padx=(0, 8))
        self.recording_label = tk.Label(
            recording_row,
            text="No recording",
            anchor="w",
            fg="gray",
        )
        self.recording_label.pack(side="left", fill="x", expand=True)

        self.ch8_var = tk.IntVar(value=1000)
        self.ch8_btn = tk.Button(
            self.root, command=self.toggle_ch8,
            font=("Arial", 13, "bold"), height=2, relief="raised", bd=3
        )
        self.ch8_btn.pack(fill="x", padx=20, pady=(0, 10))

        self.wireless_test_btn = tk.Button(
            self.root, text="Run Wireless Communications Test",
            command=self.toggle_wireless_test,
            font=("Arial", 11, "bold"), height=2, relief="raised", bd=2
        )
        self.wireless_test_btn.pack(fill="x", padx=20, pady=(0, 10))

        tk.Label(
            self.root, text="Flight Controls", font=("Arial", 10, "bold"),
            fg="#333333", anchor="w"
        ).pack(fill="x", padx=20)

        big_label = ("Arial", 12, "bold")
        big_value = ("Arial", 12, "bold")
        self.slider_ch1 = self.create_slider(
            "Yaw  (CH1)", 1500, big_label, big_value, 14, 7
        )
        self.slider_ch2 = self.create_slider(
            "Pitch  (CH2)", 1500, big_label, big_value, 14, 7
        )
        self.slider_ch3 = self.create_slider(
            "Throttle  (CH3)", 1000, big_label, big_value, 14, 7
        )

        ttk.Separator(self.root, orient="horizontal").pack(
            fill="x", padx=20, pady=(10, 4)
        )
        tk.Label(
            self.root, text="Trim (servo center)", font=("Arial", 9),
            fg="gray", anchor="w"
        ).pack(fill="x", padx=20)

        small_label = ("Arial", 9)
        small_value = ("Arial", 9)
        self.slider_ch5 = self.create_slider(
            "Trim 1 (CH5)", 1500, small_label, small_value, 12, 1
        )
        self.slider_ch6 = self.create_slider(
            "Trim 2 (CH6)", 1500, small_label, small_value, 12, 1
        )

        self.sliders = (
            self.slider_ch1, self.slider_ch2, self.slider_ch3,
            self.slider_ch5, self.slider_ch6
        )
        self.status_label = tk.Label(
            self.root, text="Not connected", bd=1, relief="sunken",
            anchor="w", padx=6
        )
        self.status_label.pack(side="bottom", fill="x", padx=10, pady=8)
        self.refresh_ch8_button()

    def set_battery_card(self, voltage_text, detail_text, state):
        palettes = {
            "normal": ("#e8f5e9", "#1b5e20", "#2e7d32"),
            "warning": ("#ffebee", "#b71c1c", "#b71c1c"),
            "unavailable": ("#f5f5f5", "gray", "#666666"),
        }
        background, voltage_color, detail_color = palettes[state]
        self.battery_panel.configure(bg=background)
        self.battery_header.configure(bg=background)
        self.battery_title_label.configure(
            bg=background, fg=detail_color
        )
        self.battery_voltage_label.configure(
            text=voltage_text, bg=background, fg=voltage_color
        )
        self.battery_label.configure(
            text=detail_text, bg=background, fg=detail_color
        )

    def set_battery_unavailable(self, detail="Waiting for receiver telemetry"):
        self.set_battery_card("--.-- V", detail, "unavailable")

    def set_confirmed_channels_unavailable(
            self, detail="Waiting for receiver telemetry"):
        self.confirmed_channels_label.configure(
            text=f"Confirmed channels: {detail}", fg="gray"
        )

    def refresh_recording_controls(self):
        if self.data_logger.active:
            self.recording_btn.configure(
                text="STOP RECORDING",
                state="normal",
                bg="#c62828",
                activebackground="#c62828",
                fg="white",
            )
            filename = (
                self.data_logger.path.name
                if self.data_logger.path is not None
                else "CSV"
            )
            self.recording_label.configure(
                text=(
                    f"{filename} | "
                    f"{self.data_logger.sample_count} confirmed samples"
                ),
                fg="#1b5e20",
            )
        else:
            enabled = self.connection_state == "connected"
            self.recording_btn.configure(
                text="START RECORDING",
                state="normal" if enabled else "disabled",
                bg="#2e7d32",
                activebackground="#2e7d32",
                fg="white",
            )
            self.recording_label.configure(text="No recording", fg="gray")

    def toggle_recording(self):
        if self.data_logger.active:
            self.stop_recording("recording_stopped_by_user")
            return
        if self.connection_state != "connected":
            self.set_fault(
                "recording",
                "Recording requires a verified transmitter connection.",
                2,
            )
            return

        logs_directory = Path(__file__).resolve().parent.parent / "logs"
        try:
            logs_directory.mkdir(parents=True, exist_ok=True)
        except OSError as error:
            self.set_fault(
                "recording", f"Cannot create the logs directory: {error}", 3
            )
            return

        suggested = default_log_path(logs_directory)
        selected = filedialog.asksaveasfilename(
            parent=self.root,
            title="Save receiver-confirmed data",
            initialdir=str(logs_directory),
            initialfile=suggested.name,
            defaultextension=".csv",
            filetypes=(("CSV data", "*.csv"), ("All files", "*.*")),
        )
        if not selected:
            return

        try:
            destination = self.data_logger.start(selected)
            self.data_logger.log_event(
                "recording_started", self.current_channel_values()
            )
        except (OSError, RuntimeError, ValueError) as error:
            self.set_fault(
                "recording", f"Cannot start recording: {error}", 3
            )
            self.refresh_recording_controls()
            return

        self.clear_fault("recording")
        self.set_status(f"Recording receiver-confirmed data to {destination}")
        self.refresh_recording_controls()

    def stop_recording(self, event="recording_stopped"):
        if not self.data_logger.active:
            self.refresh_recording_controls()
            return
        destination = self.data_logger.path
        samples = self.data_logger.sample_count
        try:
            self.data_logger.stop(event, self.current_channel_values())
        except OSError as error:
            self.set_fault(
                "recording", f"Recording ended after a file error: {error}", 3
            )
        else:
            self.clear_fault("recording")
            self.set_status(
                f"Saved {samples} confirmed samples to {destination}", "green"
            )
        self.refresh_recording_controls()

    def log_recording_event(self, event):
        if not self.data_logger.active:
            return
        try:
            self.data_logger.log_event(event, self.current_channel_values())
        except OSError as error:
            self.set_fault(
                "recording", f"Recording stopped after a file error: {error}", 3
            )
        self.refresh_recording_controls()

    def log_receiver_sample(self, status):
        if not self.data_logger.active:
            return
        try:
            self.data_logger.log_receiver_status(
                status, self.current_channel_values()
            )
        except (OSError, ValueError) as error:
            self.set_fault(
                "recording", f"Recording stopped after a file error: {error}", 3
            )
        self.refresh_recording_controls()

    def create_slider(self, label_text, default_value, label_font, value_font,
                      label_width, pady):
        frame = tk.Frame(self.root)
        frame.pack(fill="x", padx=20, pady=pady)
        tk.Label(
            frame, text=label_text, width=label_width, anchor="w",
            font=label_font
        ).pack(side="left")

        value_var = tk.StringVar(value=str(default_value))
        entry = tk.Entry(
            frame, textvariable=value_var, width=6, justify="center",
            font=value_font
        )
        entry.pack(side="right")

        slider = ttk.Scale(
            frame, from_=1000, to=2000, orient="horizontal"
        )
        slider.set(default_value)
        slider.pack(side="left", fill="x", expand=True, padx=8)

        def on_slider_change(value):
            value_var.set(str(int(float(value))))
            if not self.suppress_slider_send:
                self.send_channels()

        def on_entry(_event=None):
            try:
                value = int(float(value_var.get()))
            except ValueError:
                value = int(slider.get())
            value = max(1000, min(2000, value))
            value_var.set(str(value))
            slider.set(value)

        slider.configure(command=on_slider_change)
        entry.bind("<Return>", on_entry)
        entry.bind("<FocusOut>", on_entry)
        slider.value_var = value_var
        slider.entry_widget = entry
        return slider

    def set_fault(self, key, message, priority=2):
        self.faults[key] = (priority, message)
        self.render_alert()

    def clear_fault(self, key):
        if key in self.faults:
            del self.faults[key]
            self.render_alert()

    def set_status(self, message, color="black"):
        self.status_message = (message, color)
        self.render_alert()

    def render_alert(self):
        if self.faults:
            _, message = max(self.faults.values(), key=lambda item: item[0])
            priority = max(item[0] for item in self.faults.values())
            if priority >= 3:
                colors = ("#ffebee", "#b71c1c")
            elif priority == 2:
                colors = ("#fff3e0", "#e65100")
            else:
                colors = ("#fffde7", "#795548")
            self.status_label.configure(
                text=message, bg=colors[0], fg=colors[1]
            )
        else:
            message, color = self.status_message
            self.status_label.configure(
                text=message, bg="#f5f5f5", fg=color
            )

    @staticmethod
    def port_key(port):
        if port.serial_number:
            return ("usb", port.vid, port.pid, port.serial_number)
        if port.location:
            return ("location", port.vid, port.pid, port.location)
        return ("port", port.device, port.hwid)

    @staticmethod
    def port_score(port):
        description = (port.description or "").lower()
        if "cp210" in description:
            return 100
        if any(name in description for name in KNOWN_TRANSMITTER_CHIPS):
            return 60
        if "usb serial device" in description:
            return 20
        return 0

    @staticmethod
    def format_port(port):
        description = port.description or "Unknown device"
        serial_number = f" | SN {port.serial_number}" if port.serial_number else ""
        return f"{port.device} | {description}{serial_number}"

    def refresh_ports(self):
        ports = sorted(
            serial.tools.list_ports.comports(), key=lambda item: item.device
        )
        current_label = self.port_var.get()
        self.port_by_label = {
            self.format_port(port): port for port in ports
        }
        labels = list(self.port_by_label)
        self.port_combo.configure(values=labels)

        selected = None
        if current_label in self.port_by_label:
            selected = current_label
        if self.preferred_port_key is not None:
            for label, port in self.port_by_label.items():
                if self.port_key(port) == self.preferred_port_key:
                    selected = label
                    break
        if selected is None and labels:
            selected = max(
                labels, key=lambda label: self.port_score(
                    self.port_by_label[label]
                )
            )
        self.port_var.set(selected or "")
        return ports

    def auto_connect_initial(self):
        if self.closing or self.connection_state != "disconnected":
            return
        if self.port_var.get():
            self.begin_connection(automatic=True)
        else:
            self.set_fault(
                "serial", "No serial devices found. Connect the transmitter.",
                2
            )

    def toggle_connection(self):
        if self.connection_state in ("connected", "connecting"):
            self.disconnect_serial(
                "Disconnected by user", unexpected=False, send_lock=True
            )
        else:
            self.auto_reconnect = True
            self.begin_connection(automatic=False)

    def begin_connection(self, automatic=False):
        if self.closing or self.connection_state != "disconnected":
            return
        label = self.port_var.get()
        port = self.port_by_label.get(label)
        if port is None:
            self.refresh_ports()
            port = self.port_by_label.get(self.port_var.get())
        if port is None:
            self.set_fault(
                "serial", "No serial port selected. Connect or select a device.",
                2
            )
            return

        try:
            self.ser = serial.Serial(
                port.device, BAUD_RATE, timeout=0, write_timeout=0.25
            )
            self.ser.reset_input_buffer()
        except (OSError, serial.SerialException) as error:
            self.ser = None
            self.connection_state = "disconnected"
            self.set_fault(
                "serial",
                f"Cannot open {port.device}: {error}. Close any Serial Monitor "
                "or other program using the port.",
                3
            )
            self.connection_label.configure(
                text=f"Transmitter: cannot open {port.device}", fg="red"
            )
            self.next_reconnect_at = time.monotonic() + RECONNECT_DELAY_SECONDS
            self.auto_reconnect = automatic
            return

        now = time.monotonic()
        self.connection_state = "connecting"
        self.connecting_port_info = port
        self.connected_port = port.device
        self.handshake_deadline = now + HANDSHAKE_TIMEOUT_SECONDS
        self.next_identify_at = now + 0.25
        self.serial_buffer.clear()
        self.connect_btn.configure(text="Cancel")
        self.connection_label.configure(
            text=f"Transmitter: verifying {port.device}...", fg="#e65100"
        )
        self.set_status("Waiting for transmitter identity", "#e65100")
        self.set_fault(
            "connecting",
            "Verifying device identity; no control commands are being sent.",
            1
        )
        self.clear_fault("serial")
        self.refresh_control_states()

    def complete_connection(self):
        now = time.monotonic()
        port = self.connecting_port_info
        self.connection_state = "connected"
        self.connected_at = now
        self.last_pong_at = now
        self.next_ping_at = now
        self.auto_reconnect = True
        if port is not None:
            self.preferred_port_key = self.port_key(port)
            self.connected_port = port.device

        self.receiver_records.clear()
        self.receiver_online = False
        self.receiver_ever_seen = False
        self.last_receiver_mac = None
        self.last_receiver_packets = None
        self.receiver_label.configure(
            text="Receiver: waiting for telemetry", fg="#e65100"
        )
        self.set_confirmed_channels_unavailable()
        self.set_battery_unavailable()
        self.connect_btn.configure(text="Disconnect")
        self.connection_label.configure(
            text=f"Transmitter: verified on {self.connected_port}", fg="green"
        )
        self.clear_fault("connecting")
        self.clear_fault("serial")
        self.clear_fault("wrong_device")
        self.clear_fault("manual_disconnect")
        self.clear_fault("receiver_missing")
        self.clear_fault("receiver_failsafe")
        self.clear_fault("multiple_receivers")

        self.force_lock(
            "Connected safely: CH8 locked; waiting for receiver telemetry.",
            transmit=False
        )
        self.refresh_control_states()
        if self.send_channels(update_status=False):
            self.write_line("PING")
            self.log_recording_event("transmitter_connected")
            self.set_status(
                f"Connected to verified transmitter on {self.connected_port}",
                "green"
            )

    def close_serial_handle(self):
        if self.ser is not None:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except (OSError, serial.SerialException):
                pass
        self.ser = None
        self.serial_buffer.clear()

    def disconnect_serial(self, reason, unexpected=True, send_lock=False):
        was_connected = self.connection_state == "connected"
        if send_lock and was_connected:
            self.force_lock(reason, transmit=True)
            if self.ser is not None:
                try:
                    self.ser.flush()
                except (OSError, serial.SerialException):
                    pass
        elif self.wireless_test_running:
            self.abort_wireless_test(reason, transmit=False)
        else:
            self.set_lock_local()

        self.log_recording_event(f"transmitter_disconnected: {reason}")
        self.close_serial_handle()
        self.connection_state = "disconnected"
        self.connecting_port_info = None
        self.receiver_online = False
        self.connect_btn.configure(text="Connect")
        self.connection_label.configure(
            text=f"Transmitter: disconnected - {reason}", fg="red"
        )
        self.receiver_label.configure(text="Receiver: unavailable", fg="gray")
        self.set_confirmed_channels_unavailable("transmitter disconnected")
        self.set_battery_unavailable("Transmitter disconnected")
        self.refresh_control_states()

        if unexpected:
            self.set_fault(
                "serial",
                f"USB transmitter connection lost: {reason}. CH8 was locked "
                "locally; automatic reconnection is active.",
                3
            )
            self.auto_reconnect = True
            self.next_reconnect_at = (
                time.monotonic() + RECONNECT_DELAY_SECONDS
            )
        else:
            self.auto_reconnect = False
            self.clear_fault("serial")
            self.set_fault("manual_disconnect", reason, 1)

    def reject_device(self, reason):
        port_name = self.connected_port or "selected port"
        self.log_recording_event(f"serial_device_rejected: {reason}")
        self.close_serial_handle()
        self.connection_state = "disconnected"
        self.connecting_port_info = None
        self.connect_btn.configure(text="Connect")
        self.connection_label.configure(
            text=f"Transmitter: rejected {port_name}", fg="red"
        )
        self.set_fault(
            "wrong_device",
            f"{reason} No commands were sent. Select the transmitter port and "
            "ensure its current firmware is flashed.",
            3
        )
        self.auto_reconnect = False
        self.refresh_control_states()

    def write_line(self, text, allow_connecting=False):
        allowed = self.connection_state == "connected"
        if allow_connecting:
            allowed = allowed or self.connection_state == "connecting"
        if not allowed or self.ser is None or not self.ser.is_open:
            return False
        try:
            self.ser.write((text + "\n").encode("ascii"))
            return True
        except (OSError, serial.SerialException) as error:
            self.disconnect_serial(str(error), unexpected=True)
            return False

    def poll_serial(self):
        if self.closing:
            return
        if self.ser is not None and self.connection_state in (
                "connecting", "connected"):
            try:
                waiting = self.ser.in_waiting
                if waiting:
                    self.serial_buffer.extend(self.ser.read(min(waiting, 4096)))
                if len(self.serial_buffer) > 8192:
                    self.serial_buffer.clear()
                    self.set_fault(
                        "serial_data",
                        "Malformed or excessive serial data was discarded.",
                        2
                    )
                while b"\n" in self.serial_buffer:
                    raw_line, _, remainder = self.serial_buffer.partition(b"\n")
                    self.serial_buffer = bytearray(remainder)
                    line = raw_line.decode(
                        "utf-8", errors="replace"
                    ).strip("\r ").strip()
                    if line:
                        self.handle_serial_line(line)
            except (OSError, serial.SerialException) as error:
                self.disconnect_serial(str(error), unexpected=True)
        self.root.after(SERIAL_POLL_MS, self.poll_serial)

    def handle_serial_line(self, line):
        identity = IDENTITY_RE.fullmatch(line)
        if identity:
            device_id = identity.group(1)
            protocol = int(identity.group(2))
            radio_ready = identity.group(3) == "1"
            if device_id != TRANSMITTER_DEVICE_ID:
                self.reject_device(
                    f"Wrong device identity '{device_id}'."
                )
                return
            if protocol != HOST_PROTOCOL_VERSION:
                self.reject_device(
                    f"Incompatible protocol {protocol}; GUI requires "
                    f"{HOST_PROTOCOL_VERSION}."
                )
                return
            if not radio_ready:
                self.reject_device(
                    "The transmitter identified correctly, but ESP-NOW failed "
                    "to initialize."
                )
                return
            if self.connection_state == "connecting":
                self.complete_connection()
            elif self.connection_state == "connected":
                # Boot output and the IDENTIFY response can arrive together.
                # Treat identity as a reboot only after the handshake grace.
                if time.monotonic() - self.connected_at > 1.0:
                    self.handle_transmitter_reboot()
            return

        pong = PONG_RE.fullmatch(line)
        if pong:
            protocol = int(pong.group(1))
            host_failsafe = pong.group(2) == "1"
            radio_ready = pong.group(3) == "1"
            if protocol != HOST_PROTOCOL_VERSION or not radio_ready:
                self.disconnect_serial(
                    "Transmitter reported incompatible or failed firmware",
                    unexpected=True
                )
                return
            self.last_pong_at = time.monotonic()
            self.clear_fault("liveness")
            if host_failsafe:
                self.handle_host_failsafe()
            return

        receiver_status = parse_receiver_status(line)
        if receiver_status is not None:
            self.log_receiver_sample(receiver_status)
            self.handle_receiver_status(receiver_status)
            return

        if "GUI connection lost - CH8 locked and failsafe engaged" in line:
            self.handle_host_failsafe()
        elif "GUI connection restored - failsafe cleared" in line:
            self.clear_fault("host_failsafe")
            self.set_status(
                "Host failsafe cleared while CH8 remains locked.", "green"
            )

    def handle_transmitter_reboot(self):
        self.log_recording_event("transmitter_restarted")
        self.receiver_records.clear()
        self.receiver_online = False
        self.receiver_ever_seen = False
        self.connected_at = time.monotonic()
        self.last_pong_at = self.connected_at
        self.force_lock(
            "Transmitter restarted. CH8 locked; rechecking receiver telemetry.",
            transmit=True
        )
        self.set_fault(
            "transmitter_reboot",
            "The transmitter restarted. CH8 is locked and receiver status must "
            "be verified again.",
            3
        )
        self.receiver_label.configure(
            text="Receiver: waiting after transmitter restart", fg="#e65100"
        )
        self.set_confirmed_channels_unavailable(
            "waiting after transmitter restart"
        )
        self.set_battery_unavailable("Waiting after transmitter restart")

    def handle_host_failsafe(self):
        if "host_failsafe" not in self.faults:
            self.log_recording_event("host_failsafe_engaged")
        self.force_lock(
            "Host heartbeat failsafe engaged. Verify the robot before "
            "unlocking again.",
            transmit=True
        )
        self.set_fault(
            "host_failsafe",
            "HOST FAILSAFE: CH8 is locked. Verify the robot and connection "
            "before deliberately unlocking.",
            3
        )

    def handle_receiver_status(self, status):
        now = time.monotonic()
        mac = status.mac
        packets = status.packets
        previous = self.receiver_records.get(mac)
        if previous is not None and packets < previous["packets"]:
            self.force_lock(
                "Receiver restarted. CH8 locked pending link verification.",
                transmit=True
            )
            self.set_fault(
                "receiver_reboot",
                "The receiver packet counter restarted. CH8 was locked.",
                3
            )
            self.root.after(
                5000, lambda: self.clear_fault("receiver_reboot")
            )

        self.receiver_records[mac] = {
            "last": now,
            "packets": packets,
            "sequence": status.sequence,
            "channels": status.channels,
            "link": status.link_active,
            "failsafe": status.failsafe,
            "battery_raw": status.battery_raw,
            "battery_pin_mv": status.battery_pin_mv,
        }
        self.receiver_ever_seen = True
        self.check_receiver_state(now)

    def check_receiver_state(self, now=None):
        if now is None:
            now = time.monotonic()
        if self.connection_state != "connected":
            return

        active = {
            mac: record for mac, record in self.receiver_records.items()
            if now - record["last"] <= RECEIVER_STATUS_TIMEOUT_SECONDS
        }
        self.receiver_records = active

        if not active:
            grace_elapsed = now - self.connected_at
            if not self.receiver_ever_seen and (
                    grace_elapsed < RECEIVER_INITIAL_TIMEOUT_SECONDS):
                self.receiver_label.configure(
                    text="Receiver: waiting for telemetry", fg="#e65100"
                )
                return
            was_online = self.receiver_online
            self.receiver_online = False
            label = (
                "Receiver: telemetry lost" if self.receiver_ever_seen
                else "Receiver: not detected"
            )
            self.receiver_label.configure(text=label, fg="red")
            self.set_confirmed_channels_unavailable(label.lower())
            self.set_battery_unavailable(label)
            self.set_fault(
                "receiver_missing",
                "Receiver telemetry is missing. Confirm receiver power, "
                "ESP-NOW channel, and distance. Arming and testing are blocked.",
                3 if self.receiver_ever_seen else 2
            )
            if was_online or self.ch8_var.get() == 2000:
                self.force_lock(
                    "Receiver telemetry lost. CH8 locked.", transmit=True
                )
            self.refresh_control_states()
            return

        if len(active) > 1:
            self.receiver_online = False
            macs = ", ".join(sorted(active))
            self.receiver_label.configure(
                text=f"Receiver: multiple devices ({macs})", fg="red"
            )
            self.set_confirmed_channels_unavailable(
                "multiple receivers detected"
            )
            self.set_battery_unavailable("Multiple receivers detected")
            self.set_fault(
                "multiple_receivers",
                "Multiple receivers are responding to broadcast control. CH8 "
                "is locked; power off unintended receivers.",
                3
            )
            self.force_lock(
                "Multiple receivers detected. CH8 locked.", transmit=True
            )
            self.refresh_control_states()
            return

        mac, status = next(iter(active.items()))
        self.last_receiver_mac = mac
        self.last_receiver_packets = status["packets"]
        self.receiver_label.configure(
            text=(
                f"Receiver: {mac} | packets {status['packets']} | "
                f"sequence {status['sequence']} | "
                f"link {int(status['link'])} | failsafe "
                f"{int(status['failsafe'])}"
            ),
            fg="green" if status["link"] and not status["failsafe"] else "red"
        )
        channels = status["channels"]
        self.confirmed_channels_label.configure(
            text=(
                f"Confirmed: CH1 {channels[0]} | CH2 {channels[1]} | "
                f"CH3 {channels[2]} | CH5 {channels[3]} | "
                f"CH6 {channels[4]} | CH8 {channels[5]}"
            ),
            fg="green" if status["link"] and not status["failsafe"] else "red",
        )
        battery_raw = status["battery_raw"]
        battery_pin_mv = status["battery_pin_mv"]
        battery_valid = (
            0 <= battery_raw <= 4095 and 0 <= battery_pin_mv <= 3300
        )
        if not battery_valid:
            self.set_battery_card(
                "ERROR",
                f"Invalid telemetry: GPIO3={battery_pin_mv} mV, "
                f"raw={battery_raw}",
                "warning"
            )
        else:
            battery_voltage = (
                battery_pin_mv / 1000.0 * BATTERY_DIVIDER_RATIO
            )
            near_limit = battery_pin_mv >= BATTERY_ADC_NEAR_LIMIT_MV
            warning = " | ADC NEAR LIMIT" if near_limit else ""
            self.set_battery_card(
                f"{battery_voltage:.2f} V",
                f"GPIO3 {battery_pin_mv / 1000.0:.3f} V | "
                f"raw ADC {battery_raw}{warning}",
                "warning" if near_limit else "normal"
            )

        if not status["link"] or status["failsafe"]:
            was_online = self.receiver_online
            self.receiver_online = False
            self.set_fault(
                "receiver_failsafe",
                "Receiver reports link loss or failsafe. CH8 is locked until "
                "healthy telemetry returns.",
                3
            )
            if was_online or self.ch8_var.get() == 2000:
                self.force_lock(
                    "Receiver failsafe reported. CH8 locked.", transmit=True
                )
        else:
            self.receiver_online = True
            self.clear_fault("receiver_missing")
            self.clear_fault("receiver_failsafe")
            self.clear_fault("multiple_receivers")
            self.clear_fault("transmitter_reboot")
        self.refresh_control_states()
        self.render_alert()

    def service_connection(self):
        if self.closing:
            return
        now = time.monotonic()

        if now >= self.next_port_refresh_at:
            self.refresh_ports()
            self.next_port_refresh_at = now + PORT_REFRESH_SECONDS

        if self.connection_state == "connecting":
            if now >= self.handshake_deadline:
                self.reject_device(
                    "No compatible transmitter identity was received."
                )
            elif now >= self.next_identify_at:
                self.write_line("IDENTIFY", allow_connecting=True)
                self.next_identify_at = now + IDENTIFY_INTERVAL_SECONDS

        elif self.connection_state == "connected":
            if now >= self.next_ping_at:
                self.write_line("PING")
                self.next_ping_at = now + PING_INTERVAL_SECONDS
            if now - self.last_pong_at > PONG_TIMEOUT_SECONDS:
                self.set_fault(
                    "liveness",
                    "The transmitter stopped answering health checks.",
                    3
                )
                self.disconnect_serial(
                    "health-check timeout", unexpected=True
                )
            else:
                self.check_receiver_state(now)

        elif self.auto_reconnect and now >= self.next_reconnect_at:
            matching_label = None
            if self.preferred_port_key is not None:
                for label, port in self.port_by_label.items():
                    if self.port_key(port) == self.preferred_port_key:
                        matching_label = label
                        break
            elif self.port_by_label:
                known_labels = [
                    label for label, port in self.port_by_label.items()
                    if self.port_score(port) >= 60
                ]
                if known_labels:
                    matching_label = max(
                        known_labels,
                        key=lambda label: self.port_score(
                            self.port_by_label[label]
                        )
                    )
            if matching_label is not None:
                self.port_var.set(matching_label)
                self.begin_connection(automatic=True)
            self.next_reconnect_at = now + RECONNECT_DELAY_SECONDS

        if self.unlock_confirm_until and now > self.unlock_confirm_until:
            self.unlock_confirm_until = 0.0
            self.clear_fault("unlock_confirm")
            self.refresh_ch8_button()
        if self.test_confirm_until and now > self.test_confirm_until:
            self.test_confirm_until = 0.0
            self.clear_fault("test_confirm")
            self.refresh_test_button()

        self.root.after(CONNECTION_SERVICE_MS, self.service_connection)

    def refresh_control_states(self):
        connected = self.connection_state == "connected"
        slider_state = "normal" if connected else "disabled"
        for slider in getattr(self, "sliders", ()):
            slider.configure(state=slider_state)
            slider.entry_widget.configure(state=slider_state)
        self.ch8_btn.configure(state="normal" if connected else "disabled")
        test_enabled = connected and self.receiver_online
        if self.wireless_test_running:
            test_enabled = connected
        self.wireless_test_btn.configure(
            state="normal" if test_enabled else "disabled"
        )
        self.port_combo.configure(
            state="disabled"
            if self.connection_state in ("connected", "connecting")
            else "readonly"
        )
        self.refresh_recording_controls()
        self.refresh_ch8_button()
        self.refresh_test_button()

    def refresh_ch8_button(self):
        if self.ch8_var.get() == 2000:
            self.ch8_btn.configure(
                text="THROTTLE LOCK (CH8):   UNLOCKED - ARMED",
                bg="#2e7d32", activebackground="#2e7d32", fg="white"
            )
        elif self.unlock_confirm_until > time.monotonic():
            self.ch8_btn.configure(
                text="CONFIRM UNLOCK: click again within 4 seconds",
                bg="#ef6c00", activebackground="#ef6c00", fg="white"
            )
        else:
            self.ch8_btn.configure(
                text="THROTTLE LOCK (CH8):   LOCKED - DISARMED",
                bg="#c62828", activebackground="#c62828", fg="white"
            )

    def refresh_test_button(self):
        if self.wireless_test_running:
            text = "STOP Wireless Test"
        elif self.test_confirm_until > time.monotonic():
            text = "CONFIRM: Start Wireless Test"
        else:
            text = "Run Wireless Communications Test"
        self.wireless_test_btn.configure(text=text)

    def set_lock_local(self):
        self.ch8_var.set(1000)
        self.unlock_confirm_until = 0.0
        self.clear_fault("unlock_confirm")
        self.refresh_ch8_button()

    def force_lock(self, reason, transmit=True):
        if self.wireless_test_running:
            self.abort_wireless_test(reason, transmit=transmit)
            return
        self.set_lock_local()
        if transmit and self.connection_state == "connected":
            self.send_channels(update_status=False)
        self.set_status(reason, "red")

    def toggle_ch8(self):
        if self.connection_state != "connected":
            return
        if self.wireless_test_running:
            self.abort_wireless_test(
                "Wireless test stopped manually; CH8 locked.", transmit=True
            )
            return

        if self.ch8_var.get() == 2000:
            self.set_lock_local()
            self.send_channels()
            self.set_status("CH8 locked immediately.", "red")
            return

        if not self.receiver_online:
            self.set_fault(
                "arm_blocked",
                "Unlock blocked: one healthy receiver must be reporting link=1 "
                "and failsafe=0.",
                2
            )
            return
        if int(self.slider_ch3.get()) != 1000:
            self.set_fault(
                "arm_blocked",
                "Unlock blocked: set CH3 throttle to 1000 before arming.",
                2
            )
            return

        now = time.monotonic()
        if now > self.unlock_confirm_until:
            self.unlock_confirm_until = now + CONFIRMATION_SECONDS
            self.refresh_ch8_button()
            self.set_fault(
                "unlock_confirm",
                "Unlock requested. Verify the robot and click the orange CH8 "
                "button again within four seconds.",
                2
            )
            return

        self.unlock_confirm_until = 0.0
        self.clear_fault("unlock_confirm")
        self.clear_fault("arm_blocked")
        self.ch8_var.set(2000)
        self.refresh_ch8_button()
        self.send_channels()
        self.set_status(
            "CH8 unlocked by two-step confirmation.", "#2e7d32"
        )

    def current_channel_values(self):
        return (
            int(self.slider_ch1.get()),
            int(self.slider_ch2.get()),
            int(self.slider_ch3.get()),
            int(self.slider_ch5.get()),
            int(self.slider_ch6.get()),
            int(self.ch8_var.get()),
        )

    def send_channels(self, *_, update_status=True):
        if self.connection_state != "connected":
            return False
        values = tuple(
            max(1000, min(2000, int(value)))
            for value in self.current_channel_values()
        )
        command = "<" + ",".join(str(value) for value in values) + ">"
        sent = self.write_line(command)
        if sent and update_status:
            self.set_status(f"Sent: {command}", "green")
        return sent

    def gui_heartbeat(self):
        if self.closing:
            return
        if self.connection_state == "connected":
            self.send_channels(update_status=False)
        self.root.after(GUI_HEARTBEAT_MS, self.gui_heartbeat)

    def set_slider_value(self, slider, value):
        value = max(1000, min(2000, int(value)))
        slider.value_var.set(str(value))
        slider.set(value)

    def apply_channel_values(self, values, update_status=True):
        self.suppress_slider_send = True
        try:
            if "ch1" in values:
                self.set_slider_value(self.slider_ch1, values["ch1"])
            if "ch2" in values:
                self.set_slider_value(self.slider_ch2, values["ch2"])
            if "ch3" in values:
                self.set_slider_value(self.slider_ch3, values["ch3"])
            if "ch5" in values:
                self.set_slider_value(self.slider_ch5, values["ch5"])
            if "ch6" in values:
                self.set_slider_value(self.slider_ch6, values["ch6"])
            if "ch8" in values:
                self.ch8_var.set(
                    max(1000, min(2000, int(values["ch8"])))
                )
                self.refresh_ch8_button()
        finally:
            self.suppress_slider_send = False
        return self.send_channels(update_status=update_status)

    def can_run_test(self):
        return (
            self.connection_state == "connected"
            and self.receiver_online
            and not self.closing
        )

    def toggle_wireless_test(self):
        if self.wireless_test_running:
            self.abort_wireless_test(
                "Wireless test stopped manually; CH8 locked.", transmit=True
            )
            return
        if not self.can_run_test():
            self.set_fault(
                "test_blocked",
                "Wireless test blocked: a verified transmitter and one healthy "
                "receiver are required.",
                2
            )
            return
        if self.ch8_var.get() != 1000 or int(self.slider_ch3.get()) != 1000:
            self.set_fault(
                "test_blocked",
                "Wireless test blocked: lock CH8 and set CH3 throttle to 1000 "
                "before starting.",
                2
            )
            return

        now = time.monotonic()
        if now > self.test_confirm_until:
            self.test_confirm_until = now + CONFIRMATION_SECONDS
            self.refresh_test_button()
            self.set_fault(
                "test_confirm",
                "Benchmark requested. Clear the area, then click the benchmark "
                "button again within four seconds.",
                2
            )
            return

        self.test_confirm_until = 0.0
        self.clear_fault("test_confirm")
        self.clear_fault("test_blocked")
        self.wireless_test_running = True
        self.refresh_test_button()
        self.run_wireless_test(0)

    def ramp_test_throttle(self, target, step_text, on_complete):
        start_value = int(self.slider_ch3.get())
        target = max(1000, min(2000, int(target)))
        if start_value == target:
            self.throttle_ramp_after_id = None
            on_complete()
            return

        direction = 1 if target > start_value else -1
        start_time = time.monotonic()

        def update_ramp():
            if not self.wireless_test_running:
                self.throttle_ramp_after_id = None
                return
            if not self.can_run_test():
                self.abort_wireless_test(
                    "Wireless test aborted by connection or receiver fault.",
                    transmit=self.connection_state == "connected"
                )
                return

            elapsed = time.monotonic() - start_time
            distance = int(THROTTLE_RAMP_RATE * elapsed)
            new_value = start_value + direction * distance
            new_value = (
                min(new_value, target) if direction > 0
                else max(new_value, target)
            )
            if not self.apply_channel_values(
                    {"ch3": new_value}, update_status=False):
                self.abort_wireless_test(
                    "Wireless test aborted because a command could not be sent.",
                    transmit=False
                )
                return

            self.set_status(
                f"Wireless test ramping at 250 units/s: {step_text} "
                f"(CH3={new_value})",
                "blue"
            )
            if new_value == target:
                self.throttle_ramp_after_id = None
                on_complete()
            else:
                self.throttle_ramp_after_id = self.root.after(
                    THROTTLE_RAMP_INTERVAL_MS, update_ramp
                )

        update_ramp()

    def run_wireless_test(self, step_index=0):
        self.wireless_test_after_id = None
        if not self.wireless_test_running:
            return
        if not self.can_run_test():
            self.abort_wireless_test(
                "Wireless test aborted by connection or receiver fault.",
                transmit=self.connection_state == "connected"
            )
            return
        if step_index >= len(WIRELESS_TEST_STEPS):
            self.finish_wireless_test()
            return

        step_text, duration_ms, values = WIRELESS_TEST_STEPS[step_index]
        immediate_values = dict(values)
        throttle_target = immediate_values.pop("ch3", None)
        if not self.apply_channel_values(
                immediate_values, update_status=False):
            self.abort_wireless_test(
                "Wireless test aborted because a command could not be sent.",
                transmit=False
            )
            return

        def hold_step():
            if not self.wireless_test_running:
                return
            if not self.can_run_test():
                self.abort_wireless_test(
                    "Wireless test aborted by connection or receiver fault.",
                    transmit=self.connection_state == "connected"
                )
                return
            self.set_status(f"Wireless test holding: {step_text}", "blue")
            self.wireless_test_after_id = self.root.after(
                duration_ms, lambda: self.run_wireless_test(step_index + 1)
            )

        if throttle_target is None:
            hold_step()
        else:
            self.ramp_test_throttle(
                throttle_target, step_text, hold_step
            )

    def finish_wireless_test(self):
        def lock_and_finish():
            self.apply_channel_values(
                WIRELESS_TEST_FINAL_VALUES, update_status=False
            )
            self.wireless_test_running = False
            self.set_status(
                "Wireless test complete: throttle minimum and CH8 locked",
                "green"
            )
            self.refresh_test_button()
            self.refresh_control_states()

        self.ramp_test_throttle(
            1000, "returning throttle to minimum", lock_and_finish
        )

    def cancel_after(self, after_id):
        if after_id is None:
            return
        try:
            self.root.after_cancel(after_id)
        except tk.TclError:
            pass

    def abort_wireless_test(self, reason, transmit=True):
        self.wireless_test_running = False
        self.cancel_after(self.wireless_test_after_id)
        self.cancel_after(self.throttle_ramp_after_id)
        self.wireless_test_after_id = None
        self.throttle_ramp_after_id = None
        self.test_confirm_until = 0.0
        self.set_lock_local()
        if transmit and self.connection_state == "connected":
            self.send_channels(update_status=False)
        self.set_status(reason, "red")
        self.refresh_test_button()
        self.refresh_control_states()

    def close_gui(self):
        if self.closing:
            return
        self.closing = True
        if self.wireless_test_running:
            self.abort_wireless_test(
                "GUI closing; wireless test stopped and CH8 locked.",
                transmit=False
            )
        else:
            self.set_lock_local()

        if (
            self.connection_state == "connected"
            and self.ser is not None
            and self.ser.is_open
        ):
            values = self.current_channel_values()
            command = "<" + ",".join(str(value) for value in values) + ">\n"
            try:
                for _ in range(3):
                    self.ser.write(command.encode("ascii"))
                self.ser.flush()
            except (OSError, serial.SerialException):
                pass
        if self.data_logger.active:
            try:
                self.data_logger.stop(
                    "gui_closed", self.current_channel_values()
                )
            except OSError:
                pass
        self.close_serial_handle()
        self.root.destroy()


def main():
    root = tk.Tk()
    ControllerGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
