# Flapping-Wing SBUS Controller

Wireless control and battery-telemetry link for a flapping-wing robot. A PC
controls a USB-connected ESP32 transmitter. An ESP32-C3 SuperMini receiver
converts the wireless commands to inverted SBUS and returns link and battery
status to the PC.

```text
Control:
Python / optional GUI -> USB serial -> ESP32 transmitter -> ESP-NOW
                      -> ESP32-C3 receiver -> GPIO4 SBUS -> flight controller

Telemetry:
GPIO3 ADC -> ESP32-C3 receiver -> ESP-NOW -> ESP32 transmitter
          -> USB serial -> Python / optional GUI
```

For software automation details, see
[PC serial protocol](docs/serial-protocol.md). The full engineering report is
in [docs/flapping-wing-sbus-controller.tex](docs/flapping-wing-sbus-controller.tex).

### Quick entry points

Run these from the repository root:

```powershell
py -m pip install -r wireless\requirements.txt
py -m wireless.gui
py -m wireless.log_telemetry --list-ports
py -m wireless.log_telemetry --port COM10 --duration 60
```

The GUI's **START RECORDING** button records the currently commanded channels,
receiver-confirmed channels, link/failsafe state, and returned battery voltage
to a timestamped CSV. The standalone recorder sends only the safe locked
channel state. It is used instead of the GUI because two programs cannot open
the same COM port simultaneously.

## 1. System summary

The PC sends these six values in this exact order:

| Order | Channel | Function | Safe default |
|---:|---|---|---:|
| 1 | CH1 | Yaw | 1500 |
| 2 | CH2 | Pitch | 1500 |
| 3 | CH3 | Throttle | 1000 |
| 4 | CH5 | Trim 1 / servo center | 1500 |
| 5 | CH6 | Trim 2 / servo center | 1500 |
| 6 | CH8 | Throttle lock / arm | 1000 (locked) |

The complete host command is:

```text
<CH1,CH2,CH3,CH5,CH6,CH8>\n
```

Safe example:

```text
<1500,1500,1000,1500,1500,1000>\n
```

Host values are dimensionless control units from 1000 to 2000, not PWM pulse
widths. The receiver maps them to raw SBUS values: 1000 to 172, 1500 to about
992, and 2000 to 1811.

Normal operational connections are:

| Device | Connection |
|---|---|
| PC to transmitter | Transmitter USB COM port |
| Receiver GPIO4 | Flight-controller SBUS input |
| Receiver GPIO3 | Battery-divider midpoint |
| Receiver GND | Flight-controller GND and battery negative |

The receiver USB port is used for flashing and diagnostics, not for normal PC
control.

## 2. SBUS control transmission pipeline

### PC to transmitter

Open the **transmitter** COM port at 115200 baud, 8N1, and send newline-ended
ASCII. `8N1` means 8 data bits, no parity bit, and 1 stop bit. The UART also
supplies the usual start bit. `115200 baud` means 115200 signaling intervals per
second; for this binary UART that is nominally 115200 bit/s. An 8N1 character
uses 10 bits including its start and stop bits, so the theoretical maximum is
about 11520 characters/s before software and USB overhead.

Before sending channel data, send `IDENTIFY\n` and require this exact response:

```text
DEVICE:FLAPPING_WING_TRANSMITTER;PROTOCOL:3;RADIO:1
```

Then send all six channels at least every 100 ms. There is no acknowledgement
for an individual channel command. The transmitter forwards the latest command
over ESP-NOW every 50 ms.

### Transmitter to receiver

The transmitter broadcasts a packed binary `SbusPacket` on ESP-NOW channel 1.
The packet contains 16 host-value slots, a 32-bit sequence, and transmitter
failsafe state. It is 37 bytes, unencrypted, and is an internal firmware
interface rather than a PC text format. A new sequence is assigned for every
valid PC command and for a transmitter-side failsafe change; 50 ms heartbeat
retransmissions keep the same sequence.

### Receiver to flight controller

After receiver startup qualification, the receiver maps the host values to raw
SBUS counts and sends inverted SBUS from GPIO4 about every 10 ms. SBUS uses
100000 baud, 8E2: 8 data bits, even parity, and 2 stop bits. This is different
from the PC USB-serial format.

```text
ESP32-C3 GPIO4 ---------------- Flight-controller SBUS input
ESP32-C3 GND ------------------ Flight-controller GND
```

The firmware does not ramp or smooth channel values. Manual changes are
immediate. Only the optional GUI benchmark ramps CH3, at 250 host units/s.

## 3. Voltage measurement and return pipeline

### Divider wiring

Use the completed 300 kOhm / 100 kOhm divider on receiver GPIO3:

```text
Battery + ---- 300 kOhm ----+-- ESP32-C3 GPIO3
                            |
                          100 kOhm
                            |
Battery - ------------------+-- ESP32-C3 GND
```

The divider ratio is 4:1:

```text
GPIO3 voltage = battery voltage / 4
battery voltage = GPIO3 voltage * 4
```

Never connect the battery directly to GPIO3. With the ESP32-C3 ADC configured
for 11 dB attenuation, the documented measurable range is approximately
0-2.5 V; the nominal divider therefore corresponds to about 0-10 V at the
battery. Keep margin for resistor tolerance, ADC error, noise, and battery
transients. Verify against a multimeter before relying on the result.

### Receiver to PC

The receiver averages 16 ADC samples and returns these measurements alongside
the latest receiver-confirmed control state:

- `battery_raw`: averaged 12-bit ADC count, nominally 0-4095.
- `battery_pin_mv`: averaged calibrated voltage at GPIO3, in millivolts.

The receiver places them in a packed 31-byte binary
`ReceiverStatusPacket` every 250 ms. The transmitter validates the packet and
converts it to one newline-ended ASCII line for the PC:

```text
RECEIVER_STATUS mac=AA:BB:CC:DD:EE:FF packets=1234 sequence=77 link=1 failsafe=0 ch1=1500 ch2=1500 ch3=1200 ch5=1500 ch6=1500 ch8=1000 battery_raw=1800 battery_pin_mv=1100
```

`sequence` and the six `ch...` fields confirm the latest forward packet
accepted and mapped by the receiver. They represent values actually placed on
the SBUS wire only while `link=1` and `failsafe=0`. Since telemetry is 4 Hz and
PC commands are normally 10 Hz, intermediate commands may be applied without
appearing in a separate telemetry line.

A Python program reconstructs the source voltage as:

```python
battery_v = int(fields["battery_pin_mv"]) * 4 / 1000
```

For example, `battery_pin_mv=1100` represents 1.100 V at GPIO3 and an estimated
4.40 V at the battery. The firmware does not estimate percentage because that
requires known battery chemistry, cell count, load behavior, calibration, and
chosen thresholds.

## 4. Direct Python control with PySerial

The GUI is not required. Install PySerial (the package is named `pyserial`, but
the Python import is `serial`) and list the available ports:

```powershell
py -m pip install pyserial
py -m serial.tools.list_ports -v
```

Only one program can open a COM port at a time. Close PlatformIO Serial Monitor,
Arduino Serial Monitor, and the GUI before running another client.

This example identifies the correct MCU, keeps the robot locked, and reads
asynchronous telemetry from the same serial connection:

```python
import time
import serial

PORT = "COM10"
SAFE = [1500, 1500, 1000, 1500, 1500, 1000]
EXPECTED = "DEVICE:FLAPPING_WING_TRANSMITTER;PROTOCOL:3;RADIO:1"


def encode_channels(values):
    if len(values) != 6:
        raise ValueError("six channel values are required")
    return ("<" + ",".join(str(v) for v in values) + ">\n").encode("ascii")


with serial.Serial(
    port=PORT,
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.05,
    write_timeout=0.25,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
) as ser:
    ser.reset_input_buffer()

    # Opening a COM port can reset the ESP32. Identify repeatedly for 5 s.
    deadline = time.monotonic() + 5.0
    next_identify = 0.0
    while True:
        now = time.monotonic()
        if now >= deadline:
            raise RuntimeError("expected transmitter not found")
        if now >= next_identify:
            ser.write(b"IDENTIFY\n")
            next_identify = now + 0.5
        line = ser.readline().decode("ascii", errors="replace").strip()
        if line == EXPECTED:
            break

    safe_packet = encode_channels(SAFE)
    next_control = time.monotonic()
    try:
        while True:
            now = time.monotonic()
            if now >= next_control:
                ser.write(safe_packet)
                next_control = now + 0.100

            line = ser.readline().decode("ascii", errors="replace").strip()
            if line.startswith("RECEIVER_STATUS "):
                try:
                    fields = dict(
                        part.split("=", 1) for part in line.split()[1:]
                    )
                    battery_v = int(fields["battery_pin_mv"]) * 4 / 1000
                except (ValueError, KeyError):
                    continue
                print(
                    f"seq={fields['sequence']} "
                    f"confirmed_CH3={fields['ch3']} "
                    f"link={fields['link']} failsafe={fields['failsafe']} "
                    f"battery={battery_v:.2f} V"
                )
    finally:
        # Send the locked state explicitly before the port closes.
        for _ in range(3):
            try:
                ser.write(safe_packet)
            except serial.SerialException:
                break
            time.sleep(0.05)
```

`ser.write(...)` requires bytes, so the command is ASCII-encoded. Data arriving
from the transmitter accumulates in the OS/PySerial receive buffer.
`ser.readline()` returns bytes through and including the next `\n`, or returns
the available bytes when the timeout expires. Decode only after reading. A
production client must tolerate unrelated boot text and malformed lines.

See [docs/serial-protocol.md](docs/serial-protocol.md) for all commands, field
definitions, parser rules, timing requirements, and a state-aware client.

### Live CSV recording

The reusable PC functions are in `wireless/pc_tools/`. For a custom Python
automation program:

```python
from wireless.pc_tools import LiveCsvLogger, parse_receiver_status

status = parse_receiver_status(line)
if status is not None:
    logger.log_receiver_status(status, commanded_channels)
```

The logger writes one `receiver_status` row for each returned sample, with:

- PC wall time and elapsed monotonic time;
- commanded CH1, CH2, CH3, CH5, CH6, and CH8;
- receiver-confirmed sequence and channel values;
- a `command_matches_confirmation` comparison;
- receiver MAC, packet count, link, and failsafe;
- raw ADC, GPIO3 millivolts, and reconstructed battery volts.

CSV rows are flushed immediately. The default `logs/` directory is ignored by
Git so experimental data is not committed accidentally.

## 5. Failsafe and recovery

### Receiver startup

1. For five seconds after boot, UART1 is disabled, GPIO4 is high-impedance, and
   no SBUS frames are sent.
2. The receiver then requires five consecutive healthy packets from one
   transmitter with failsafe clear and CH8 locked at 1000.
3. It enables SBUS and locks to that transmitter MAC until receiver reboot.

### Fault response

| Fault | Result |
|---|---|
| No valid PC channel command for 500 ms | Transmitter preserves CH3, forces CH8 to 1000, and marks its wireless packet as failsafe |
| No ESP-NOW control packet at receiver for 500 ms | Receiver stops UART1 and makes GPIO4 high-impedance |
| Transmitter reports host failsafe | Receiver stops UART1 and makes GPIO4 high-impedance |

The receiver does not send an SBUS frame with the SBUS failsafe flag set; it
stops SBUS output. Recovery cannot automatically arm the robot. The PC must
first resume valid packets with CH8=1000. After five healthy locked packets,
the receiver restores SBUS; CH8 can then be changed deliberately.

Before first powered operation, verify the GPIO4 and divider wiring, use common
ground, restrain the mechanism, keep CH8 locked, and test USB and radio loss.

## 6. Build and flash

Each MCU is an independent PlatformIO project. COM10 and COM11 below are
examples; replace them with the ports reported on your PC. Protocol 3 changes
both packed ESP-NOW structures, so **flash both transmitter and receiver before
running the updated GUI or logger**. Mixed old/new firmware is intentionally
rejected.

Transmitter:

```powershell
cd wireless\transmitter
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run --target upload --upload-port COM10
cd ..\..
```

PlatformIO environment: `esp32doit-devkit-v1`.

Receiver:

```powershell
cd wireless\receiver
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run --target upload --upload-port COM11
cd ..\..
```

PlatformIO environment: `esp32-c3-devkitm-1`. In Arduino IDE, **ESP32C3 Dev
Module** is the corresponding generic selection. Enable **USB CDC On Boot** only
when native-USB receiver debug output is required.

List ports with:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" device list
```

## 7. Optional GUI

```powershell
py -m pip install -r wireless\requirements.txt
py -m wireless.gui
```

The GUI identifies the transmitter before enabling controls, monitors link and
battery telemetry, locks CH8 on detected faults, and requires confirmation for
arming and benchmark start. It also displays the latest receiver-confirmed
channels. Click **START RECORDING**, choose a CSV destination, and click
**STOP RECORDING** when finished. Recording remains active across an automatic
reconnection so the disconnect gap and reconnection can remain part of one
session. It is an operator client of the same serial protocol described above,
not a required part of the embedded data path.

## 8. Repository and development notes

```text
wired/                         Legacy direct USB-to-SBUS prototype
wireless/gui.py                GUI, benchmark, and recording entry point
wireless/log_telemetry.py      Standalone safe-state CSV recorder
wireless/pc_tools/             Shared serial protocol and CSV logger
wireless/transmitter/          ESP32 transmitter firmware/PlatformIO project
wireless/receiver/             ESP32-C3 receiver firmware/PlatformIO project
tests/test_pc_tools.py          PC protocol/logger unit tests
logs/                           Local CSV output; ignored by Git
docs/serial-protocol.md        Direct Python automation protocol
docs/flapping-wing-sbus-controller.tex
                               Full engineering documentation
output/pdf/                    Compiled documentation PDF
```

`RECEIVER_VALUE_TEST` in `wireless/receiver/receiver.ino` is disabled by
default. Enable it only for receiver-side serial diagnostics.

Important current limitations include unauthenticated/unencrypted ESP-NOW,
fixed channel 1, 4 Hz confirmation that can skip intermediate 10 Hz commands,
no battery calibration stored in firmware, and no chemistry-aware battery
percentage. Any change to a packed wireless structure must be made in both
copies of `esp_now_link.h` and flashed to both nodes.

Primary future improvements are explicit forward-packet versioning,
authenticated peers, configurable radio/channel selection, calibrated battery
thresholds, and hardware-in-the-loop protocol/failsafe tests.

## External references

- [PySerial API](https://pyserial.readthedocs.io/en/stable/pyserial.html)
- [PySerial port listing](https://pyserial.readthedocs.io/en/latest/tools.html)
- [Espressif Arduino ADC API](https://docs.espressif.com/projects/arduino-esp32/en/latest/api/adc.html)
- [ESP32-C3 ADC calibration and noise](https://docs.espressif.com/projects/esp-idf/en/latest/esp32c3/api-reference/peripherals/adc/adc_calibration.html)
