"""GUI for the one-board ESP32-C3 USB-to-SBUS hardware diagnostic."""

from pathlib import Path
import sys
import time
import tkinter as tk
from tkinter import ttk

import serial
import serial.tools.list_ports

if __package__:
    from .protocol import identity_is_compatible, parse_wired_status
else:
    repository = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(repository))
    from diagnostics.wired_c3.protocol import (
        identity_is_compatible,
        parse_wired_status,
    )

from host.pc_tools import (
    SerialLineBuffer,
    encode_channel_command,
    read_serial_lines,
    write_serial_line,
)


HEARTBEAT_MS = 100
SERIAL_POLL_MS = 40
STATUS_TIMEOUT_SECONDS = 1.0
HANDSHAKE_TIMEOUT_SECONDS = 5.0
SAFE_CHANNELS = (1500, 1500, 1000, 1500, 1500, 1000)


class WiredDiagnosticGUI:
    def __init__(self, root):
        self.root = root
        self.ser = None
        self.serial_buffer = SerialLineBuffer()
        self.port_by_label = {}
        self.connected = False
        self.link_active = False
        self.last_status_at = 0.0
        self.status_stale = False
        self.closing = False

        self.build_ui()
        self.refresh_ports()
        self.root.protocol("WM_DELETE_WINDOW", self.close)
        self.root.after(SERIAL_POLL_MS, self.poll_serial)
        self.root.after(HEARTBEAT_MS, self.heartbeat)

    def build_ui(self):
        self.root.title("ESP32-C3 Wired SBUS Diagnostic")
        self.root.geometry("660x760")
        self.root.minsize(600, 700)

        tk.Label(
            self.root,
            text="WIRED DIAGNOSTIC — ESP-NOW BYPASSED",
            font=("Arial", 14, "bold"),
            fg="#b71c1c",
        ).pack(pady=(12, 3))
        tk.Label(
            self.root,
            text="PC USB → ESP32-C3 → GPIO4 inverted SBUS",
            fg="#555555",
        ).pack(pady=(0, 10))

        connection = ttk.LabelFrame(self.root, text="Diagnostic connection")
        connection.pack(fill="x", padx=16, pady=(0, 8))

        port_row = tk.Frame(connection)
        port_row.pack(fill="x", padx=8, pady=7)
        tk.Label(port_row, text="Serial port:", width=11, anchor="w").pack(
            side="left"
        )
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(
            port_row, textvariable=self.port_var, state="readonly", width=42
        )
        self.port_combo.pack(side="left", fill="x", expand=True, padx=(0, 6))
        ttk.Button(
            port_row, text="Refresh", command=self.refresh_ports
        ).pack(side="left", padx=(0, 6))
        self.connect_btn = ttk.Button(
            port_row, text="Connect", command=self.toggle_connection
        )
        self.connect_btn.pack(side="left")

        self.device_label = tk.Label(
            connection, text="Device: disconnected", anchor="w", fg="gray"
        )
        self.device_label.pack(fill="x", padx=8)
        self.sbus_label = tk.Label(
            connection,
            text="SBUS: unavailable",
            anchor="w",
            fg="gray",
        )
        self.sbus_label.pack(fill="x", padx=8)
        self.channels_label = tk.Label(
            connection,
            text="Confirmed channels: unavailable",
            anchor="w",
            fg="gray",
        )
        self.channels_label.pack(fill="x", padx=8)

        battery = tk.Frame(connection, bg="#f5f5f5", bd=1, relief="groove")
        battery.pack(fill="x", padx=8, pady=(5, 8))
        tk.Label(
            battery,
            text="GPIO3 BATTERY VOLTAGE",
            font=("Arial", 9, "bold"),
            bg="#f5f5f5",
            fg="#555555",
        ).pack(side="left", padx=9, pady=6)
        self.battery_label = tk.Label(
            battery,
            text="--.-- V",
            font=("Arial", 16, "bold"),
            bg="#f5f5f5",
            fg="gray",
        )
        self.battery_label.pack(side="right", padx=9, pady=3)
        self.battery_detail_label = tk.Label(
            battery,
            text="Waiting for wired status",
            bg="#f5f5f5",
            fg="gray",
        )
        self.battery_detail_label.pack(side="right", padx=8)

        self.ch8_var = tk.IntVar(value=1000)
        self.ch8_btn = tk.Button(
            self.root,
            command=self.toggle_ch8,
            font=("Arial", 13, "bold"),
            height=2,
            relief="raised",
            bd=3,
        )
        self.ch8_btn.pack(fill="x", padx=20, pady=(2, 12))

        tk.Label(
            self.root,
            text="Flight controls",
            font=("Arial", 10, "bold"),
            anchor="w",
        ).pack(fill="x", padx=20)

        self.slider_ch1 = self.create_slider("Yaw (CH1)", 1500)
        self.slider_ch2 = self.create_slider("Pitch (CH2)", 1500)
        self.slider_ch3 = self.create_slider("Throttle (CH3)", 1000)

        ttk.Separator(self.root, orient="horizontal").pack(
            fill="x", padx=20, pady=(10, 5)
        )
        tk.Label(
            self.root, text="Trim (servo center)", fg="gray", anchor="w"
        ).pack(fill="x", padx=20)
        self.slider_ch5 = self.create_slider("Trim 1 (CH5)", 1500, compact=True)
        self.slider_ch6 = self.create_slider("Trim 2 (CH6)", 1500, compact=True)

        self.status_label = tk.Label(
            self.root,
            text="Connect the diagnostic C3; SBUS remains silent initially.",
            bd=1,
            relief="sunken",
            anchor="w",
            padx=6,
        )
        self.status_label.pack(side="bottom", fill="x", padx=10, pady=8)
        self.refresh_ch8_button()

    def create_slider(self, label_text, default_value, compact=False):
        frame = tk.Frame(self.root)
        frame.pack(fill="x", padx=20, pady=2 if compact else 7)
        tk.Label(
            frame,
            text=label_text,
            width=15,
            anchor="w",
            font=("Arial", 9 if compact else 11, "bold" if not compact else "normal"),
        ).pack(side="left")

        value_var = tk.StringVar(value=str(default_value))
        entry = ttk.Entry(frame, textvariable=value_var, width=7, justify="center")
        entry.pack(side="right")
        slider = ttk.Scale(frame, from_=1000, to=2000, orient="horizontal")
        slider.set(default_value)
        slider.pack(side="left", fill="x", expand=True, padx=8)
        slider.configure(
            command=lambda value: value_var.set(str(round(float(value))))
        )
        slider.bind("<ButtonRelease-1>", lambda _event: self.send_channels())

        def apply_entry(_event=None):
            try:
                value = int(value_var.get())
            except ValueError:
                value = round(slider.get())
            value = max(1000, min(2000, value))
            slider.set(value)
            value_var.set(str(value))
            self.send_channels()

        entry.bind("<Return>", apply_entry)
        slider.value_var = value_var
        return slider

    def refresh_ports(self):
        selected_device = None
        selected_label = self.port_var.get()
        if selected_label in self.port_by_label:
            selected_device = self.port_by_label[selected_label].device

        ports = list(serial.tools.list_ports.comports())
        self.port_by_label = {
            f"{port.device} — {port.description or 'Unknown device'}": port
            for port in ports
        }
        labels = list(self.port_by_label)
        self.port_combo.configure(values=labels)

        chosen = next(
            (
                label for label, port in self.port_by_label.items()
                if port.device == selected_device
            ),
            labels[0] if labels else "",
        )
        self.port_var.set(chosen)

    def set_status(self, message, color="black"):
        self.status_label.configure(text=message, fg=color)

    def toggle_connection(self):
        if self.connected:
            self.disconnect("Disconnected by user", send_lock=True)
        else:
            self.connect()

    def connect(self):
        label = self.port_var.get()
        port = self.port_by_label.get(label)
        if port is None:
            self.set_status("Select an available serial port.", "red")
            return

        try:
            connection = serial.Serial(
                port.device,
                115200,
                timeout=0,
                write_timeout=0.5,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False,
            )
        except (OSError, serial.SerialException) as error:
            self.set_status(f"Cannot open {port.device}: {error}", "red")
            return

        buffer = SerialLineBuffer()
        deadline = time.monotonic() + HANDSHAKE_TIMEOUT_SECONDS
        next_identify = 0.0
        compatible = False
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_identify:
                    write_serial_line(connection, "IDENTIFY")
                    next_identify = now + 0.3
                for line in read_serial_lines(connection, buffer):
                    if identity_is_compatible(line):
                        compatible = True
                        break
                if compatible:
                    break
                self.root.update_idletasks()
                time.sleep(0.04)
        except (OSError, ValueError, serial.SerialException):
            compatible = False

        if not compatible:
            connection.close()
            self.set_status(
                (
                    f"{port.device} is not running the ESP32-C3 wired "
                    "diagnostic firmware."
                ),
                "red",
            )
            return

        self.ser = connection
        self.serial_buffer = buffer
        self.connected = True
        self.link_active = False
        self.status_stale = False
        self.last_status_at = time.monotonic()
        self.ch8_var.set(1000)
        self.connect_btn.configure(text="Disconnect")
        self.port_combo.configure(state="disabled")
        self.device_label.configure(
            text=f"Device: ESP32-C3 wired diagnostic on {port.device}",
            fg="green",
        )
        self.set_status(
            "Connected safely; waiting for five-second guard and locked packets.",
            "#e65100",
        )
        self.refresh_ch8_button()
        self.send_channels()

    def disconnect(self, reason, send_lock=False):
        if send_lock and self.ser is not None and self.ser.is_open:
            self.ch8_var.set(1000)
            command = encode_channel_command(self.current_channels())
            for _ in range(3):
                try:
                    self.ser.write(command)
                except (OSError, serial.SerialException):
                    break
                time.sleep(0.03)
        if self.ser is not None:
            try:
                self.ser.close()
            except (OSError, serial.SerialException):
                pass
        self.ser = None
        self.connected = False
        self.link_active = False
        self.ch8_var.set(1000)
        self.connect_btn.configure(text="Connect")
        self.port_combo.configure(state="readonly")
        self.device_label.configure(text="Device: disconnected", fg="gray")
        self.sbus_label.configure(text="SBUS: unavailable", fg="gray")
        self.channels_label.configure(
            text="Confirmed channels: unavailable", fg="gray"
        )
        self.set_battery_unavailable()
        self.set_status(reason, "red")
        self.refresh_ch8_button()

    def current_channels(self):
        return (
            round(self.slider_ch1.get()),
            round(self.slider_ch2.get()),
            round(self.slider_ch3.get()),
            round(self.slider_ch5.get()),
            round(self.slider_ch6.get()),
            self.ch8_var.get(),
        )

    def send_channels(self):
        if not self.connected or self.ser is None:
            return False
        try:
            self.ser.write(encode_channel_command(self.current_channels()))
            return True
        except (OSError, ValueError, serial.SerialException) as error:
            self.disconnect(f"Serial connection lost: {error}")
            return False

    def heartbeat(self):
        if self.closing:
            return
        if self.connected:
            self.send_channels()
            if (
                self.last_status_at
                and time.monotonic() - self.last_status_at
                > STATUS_TIMEOUT_SECONDS
                and not self.status_stale
            ):
                self.status_stale = True
                self.link_active = False
                self.ch8_var.set(1000)
                self.refresh_ch8_button()
                self.sbus_label.configure(
                    text="SBUS: status telemetry stale", fg="red"
                )
                self.set_status(
                    "Wired status lost; CH8 locked locally.", "red"
                )
        self.root.after(HEARTBEAT_MS, self.heartbeat)

    def poll_serial(self):
        if self.closing:
            return
        if self.connected and self.ser is not None:
            try:
                lines = read_serial_lines(self.ser, self.serial_buffer)
            except (OSError, serial.SerialException) as error:
                self.disconnect(f"Serial connection lost: {error}")
            else:
                for line in lines:
                    status = parse_wired_status(line)
                    if status is not None:
                        self.handle_status(status)
                    elif line == "ERROR:LOCK_REQUIRED":
                        self.ch8_var.set(1000)
                        self.refresh_ch8_button()
                        self.set_status(
                            "Firmware requires a locked CH8 packet first.",
                            "red",
                        )
        self.root.after(SERIAL_POLL_MS, self.poll_serial)

    def handle_status(self, status):
        self.last_status_at = time.monotonic()
        self.status_stale = False
        self.link_active = status.link_active and not status.failsafe
        color = "green" if self.link_active else "#e65100"
        self.sbus_label.configure(
            text=(
                f"SBUS: {'ACTIVE' if self.link_active else 'SILENT'} | "
                f"sequence {status.sequence} | failsafe "
                f"{int(status.failsafe)}"
            ),
            fg=color,
        )
        channels = status.channels
        self.channels_label.configure(
            text=(
                f"Confirmed: CH1 {channels[0]} | CH2 {channels[1]} | "
                f"CH3 {channels[2]} | CH5 {channels[3]} | "
                f"CH6 {channels[4]} | CH8 {channels[5]}"
            ),
            fg=color,
        )
        self.set_battery(
            status.battery_voltage,
            status.battery_pin_mv,
            status.battery_raw,
        )
        if self.link_active:
            self.set_status(
                "Direct USB-to-SBUS link active on GPIO4.", "green"
            )
        self.refresh_ch8_button()

    def set_battery(self, voltage, pin_mv, raw):
        warning = pin_mv >= 3000
        self.battery_label.configure(
            text=f"{voltage:.2f} V",
            fg="#b71c1c" if warning else "#1b5e20",
        )
        suffix = " | ADC NEAR LIMIT" if warning else ""
        self.battery_detail_label.configure(
            text=f"GPIO3 {pin_mv / 1000.0:.3f} V | raw {raw}{suffix}",
            fg="#b71c1c" if warning else "#2e7d32",
        )

    def set_battery_unavailable(self):
        self.battery_label.configure(text="--.-- V", fg="gray")
        self.battery_detail_label.configure(
            text="Waiting for wired status", fg="gray"
        )

    def refresh_ch8_button(self):
        if self.ch8_var.get() == 2000:
            self.ch8_btn.configure(
                text="THROTTLE LOCK (CH8): ON — ARMED",
                bg="#2e7d32",
                activebackground="#2e7d32",
                fg="white",
            )
        else:
            self.ch8_btn.configure(
                text="THROTTLE LOCK (CH8): OFF — LOCKED",
                bg="#c62828",
                activebackground="#c62828",
                fg="white",
            )
        self.ch8_btn.configure(state="normal" if self.connected else "disabled")

    def toggle_ch8(self):
        if not self.connected:
            return
        if self.ch8_var.get() == 2000:
            self.ch8_var.set(1000)
            self.refresh_ch8_button()
            self.send_channels()
            self.set_status("CH8 locked immediately.", "green")
            return
        if not self.link_active:
            self.set_status(
                "Unlock blocked until the wired SBUS link is active.", "red"
            )
            return

        self.ch8_var.set(2000)
        self.refresh_ch8_button()
        self.send_channels()
        self.set_status("CH8 armed for the wired diagnostic.", "#e65100")

    def close(self):
        self.closing = True
        if self.connected:
            self.disconnect("GUI closed", send_lock=True)
        self.root.destroy()


def main():
    root = tk.Tk()
    WiredDiagnosticGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()
