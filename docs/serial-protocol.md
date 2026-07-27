# PC Serial Protocol

This document is the authoritative automation interface between a PC program
and the PC-side ESP32 transmitter. It is independent of the supplied GUI. The
LaTeX engineering document provides system context, while this file defines the
exact host messages, fields, timing, and recovery rules.

## 1. Scope and physical path

The PC opens only the transmitter's USB serial port during normal operation.

```text
PC program
   |
   | USB serial, bidirectional ASCII
   v
ESP32 transmitter
   |
   | ESP-NOW, binary packets
   v
ESP32-C3 receiver
   |
   | inverted SBUS on GPIO4
   v
Flight controller
```

Receiver telemetry follows the reverse path:

```text
ESP32-C3 ReceiverStatusPacket
   |
   | binary ESP-NOW
   v
ESP32 transmitter
   |
   | Serial.printf converts fields to ASCII text
   v
Windows COM port
   |
   | PySerial read()
   v
PC receive buffer and parser
```

The receiver's USB port is not involved. It is used only for flashing and
receiver debug output.

### Python/PySerial quick start

`gui.py` is not required. Install PySerial (the package name is `pyserial`,
while the Python import name is `serial`) and list the available ports:

```powershell
py -m pip install pyserial
py -m serial.tools.list_ports -v
```

A Python automation program should then:

1. open the transmitter COM port at 115200 baud, 8N1;
2. send `IDENTIFY\n` until it receives the expected protocol-2 identity;
3. write a complete line such as
   `<1500,1500,1000,1500,1500,1000>\n` at least every 100 ms;
4. read from that same COM port continuously and split incoming bytes at `\n`;
5. parse `RECEIVER_STATUS ...` lines for link, failsafe, packet count, and
   battery data; and
6. send CH8=1000 several times before closing the port.

In short, input uses `ser.write(channel_line)` and returned data uses
`ser.read(...)` or `ser.readline()`. Both directions share the transmitter COM
port. The complete safe reference client is in Section 7.

## 2. Serial transport

| Setting | Value |
|---|---|
| Baud rate | 115200 |
| Data bits | 8 |
| Parity | None |
| Stop bits | 1 |
| Flow control | None |
| Encoding | ASCII; UTF-8 decoding is also safe for these messages |
| Message delimiter | Line feed, `\n`, byte `0x0A` |

`8N1` is shorthand for the asynchronous serial frame format: **8** data bits,
**N**o parity bit, and **1** stop bit. A start bit is also present on the wire
but is not written in the shorthand. `115200 baud` means 115200 signaling
intervals per second; for this binary UART that is nominally 115200 bit/s. Each
8N1 character occupies 10 bits including the start and stop bits, so the
theoretical maximum is about 11520 characters/s before USB and software
overhead. This PC-to-transmitter format is different from SBUS, which uses
100000 baud, 8 data bits, even parity, and 2 stop bits (`8E2`).

The same settings can be written explicitly in PySerial:

```python
import serial

ser = serial.Serial(
    port="COM10",
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.05,
    write_timeout=0.25,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
)
```

Use a `with serial.Serial(...) as ser:` block in real code so the COM port is
closed even if an exception occurs.

Serial is a byte stream, not a message transport. One read can contain part of
a line, one line, or several lines. Accumulate bytes and split only at `\n`.
Remove an optional carriage return `\r` before parsing.

Opening the port can reset some ESP32 DevKit boards through the USB-to-UART
control signals. Do not depend on a fixed startup delay. Send `IDENTIFY`
periodically until the expected identity is received or a timeout expires.

## 3. PC-to-transmitter commands

Every command is one ASCII line ending in `\n`.

### 3.1 Identity request

```text
IDENTIFY\n
```

Expected response:

```text
DEVICE:FLAPPING_WING_TRANSMITTER;PROTOCOL:2;RADIO:1
```

Accept the device only when:

- device name is `FLAPPING_WING_TRANSMITTER`;
- protocol is `2`;
- `RADIO` is `1`.

### 3.2 Health request

```text
PING\n
```

Expected response:

```text
PONG;PROTOCOL:2;HOST_FAILSAFE:0;RADIO:1;UPTIME_MS:12345
```

`PING` checks the transmitter, but it does not count as a control heartbeat.
Only a valid six-channel packet refreshes the 500 ms host-control timeout.

### 3.3 Channel command

```text
<CH1,CH2,CH3,CH5,CH6,CH8>\n
```

Safe example:

```text
<1500,1500,1000,1500,1500,1000>\n
```

| Position | Channel | Function | Valid range | Safe value |
|---:|---|---|---:|---:|
| 1 | CH1 | Yaw | 1000-2000 | 1500 |
| 2 | CH2 | Pitch | 1000-2000 | 1500 |
| 3 | CH3 | Throttle | 1000-2000 | 1000 |
| 4 | CH5 | Trim 1 / servo center | 1000-2000 | 1500 |
| 5 | CH6 | Trim 2 / servo center | 1000-2000 | 1500 |
| 6 | CH8 | Throttle lock / arm | 1000-2000 | 1000 |

The parser requires exactly six unsigned decimal values. Empty, missing, extra,
or non-numeric fields cause the complete command to be ignored. Valid numeric
values are clamped to 1000-2000. The transmitter does not return a formal error
or acknowledgement for a rejected command.

Send the full six-channel packet every 100 ms. The transmitter forwards the
latest state immediately and also rebroadcasts it over ESP-NOW every 50 ms.

## 4. Transmitter-to-PC lines

The transmitter emits structured protocol lines and unstructured diagnostic
lines on the same serial stream. Automation must recognize the structured
prefixes and safely ignore everything else.

### 4.1 Device identity

```text
DEVICE:FLAPPING_WING_TRANSMITTER;PROTOCOL:2;RADIO:1
```

Regular expression:

```python
r"^DEVICE:([^;]+);PROTOCOL:(\d+);RADIO:([01])$"
```

### 4.2 Transmitter health

```text
PONG;PROTOCOL:2;HOST_FAILSAFE:0;RADIO:1;UPTIME_MS:12345
```

Regular expression:

```python
r"^PONG;PROTOCOL:(\d+);HOST_FAILSAFE:([01]);RADIO:([01]);UPTIME_MS:(\d+)$"
```

Field meanings:

- `HOST_FAILSAFE=1`: valid PC channel packets timed out or have not yet cleared
  the startup failsafe.
- `RADIO=1`: ESP-NOW initialized. This does not prove that a receiver is online.
- `UPTIME_MS`: transmitter milliseconds since boot. A decrease indicates a
  transmitter restart.

### 4.3 Receiver status

```text
RECEIVER_STATUS mac=AA:BB:CC:DD:EE:FF packets=1234 link=1 failsafe=0 battery_raw=1800 battery_pin_mv=1100
```

Regular expression:

```python
r"^RECEIVER_STATUS mac=([0-9A-Fa-f:]{17}) packets=(\d+) "
r"link=([01]) failsafe=([01]) battery_raw=(\d+) battery_pin_mv=(\d+)$"
```

| Field | Meaning |
|---|---|
| `mac` | ESP32-C3 receiver Wi-Fi MAC that sent this status |
| `packets` | Valid forward packets received since receiver boot |
| `link` | `1` only while receiver SBUS output is actually active |
| `failsafe` | `0` only while the receiver considers control ready |
| `battery_raw` | Averaged 12-bit GPIO3 ADC count, 0-4095 |
| `battery_pin_mv` | Averaged calibrated GPIO3 voltage in millivolts |

For the 300/100 kOhm divider:

```python
battery_voltage = battery_pin_mv / 1000.0 * 4.0
```

Status is normally generated every 250 ms. More than one recent `mac` means
multiple receivers are responding to the broadcast control stream. Treat that
as an unsafe condition unless multi-receiver operation was deliberately
designed and tested.

This protocol version does not return channel values or acknowledge individual
commands. `link=1` and `failsafe=0` establish receiver health, but the PC cannot
prove which particular channel command is currently on the SBUS wire.

### 4.4 Diagnostic lines

Examples include:

```text
Transmitter MAC: ...
ESP-NOW transmitter ready
yaw(CH1):1500
GUI connection lost - CH8 locked and failsafe engaged
```

These lines are useful to a human but are not a stable machine protocol. An
automation client should ignore unknown lines and use `PONG` and
`RECEIVER_STATUS` for state decisions.

## 5. How Python retrieves a received line

This is the operation that removes waiting bytes from the operating system's
COM-port receive buffer and returns them to Python:

```python
chunk = ser.read(min(ser.in_waiting, 4096))
```

The complete buffering pattern is:

```python
rx_buffer = bytearray()

waiting = ser.in_waiting
if waiting:
    rx_buffer.extend(ser.read(min(waiting, 4096)))

while b"\n" in rx_buffer:
    raw_line, _, remainder = rx_buffer.partition(b"\n")
    rx_buffer = bytearray(remainder)
    line = raw_line.decode("ascii", errors="replace").rstrip("\r")
    if line:
        handle_line(line)
```

`in_waiting` reports the byte count; `read()` retrieves the bytes;
`partition(b"\n")` extracts one complete message; `decode()` converts the
bytes to a Python string.

## 6. Safe automation state machine

### 6.1 Connect

1. Open the expected COM port at 115200 baud.
2. Clear stale input bytes.
3. Send `IDENTIFY` every 500 ms until the correct versioned identity arrives.
4. Reject unrelated devices and `RADIO=0`.

### 6.2 Establish safe control

1. Begin sending the complete safe packet every 100 ms:
   `<1500,1500,1000,1500,1500,1000>`.
2. Optionally send `PING` every second and require a response within three
   seconds.
3. Track receiver status by MAC.
4. Require exactly one recent receiver with `link=1` and `failsafe=0`.
5. Confirm CH3 is 1000 before allowing an arm action.

The transmitter starts in host failsafe. A valid packet with CH8=1000 is
required to clear it. Starting with CH8=2000 cannot automatically arm the
robot.

### 6.3 Operate

- Continue sending all six values every 100 ms, even when unchanged.
- Keep parsing serial input; transmission and reception are simultaneous.
- Treat a transmitter restart, missing `PONG`, receiver timeout, receiver
  `failsafe=1`, receiver `link=0`, or multiple MAC addresses as a fault.
- On a fault, set CH8 to 1000 and stop automated motion.

### 6.4 Stop and disconnect

1. Set CH8 to 1000 in the local command state.
2. Send the complete locked packet at least three times, separated by about
   50 ms.
3. Close the COM port.

The repeated locked packets reduce the chance that a single wireless loss
delays the lock. This protocol has no returned CH8 field, so the PC cannot
independently confirm that the receiver accepted the locked value.

If the application crashes or the cable is removed, the 500 ms host timeout
still activates the firmware failsafe.

## 7. Maintained Python clients and CSV logging

The maintained parser and logger are reusable modules under
`host/pc_tools/`. Prefer importing them instead of copying protocol regular
expressions into new scripts:

```python
from host.pc_tools import LiveCsvLogger, parse_receiver_status

status = parse_receiver_status(line)
if status is not None:
    logger.log_receiver_status(status, commanded_channels)
```

Run the GUI from the repository root and use its **START RECORDING** button:

```powershell
py -m host.gui
```

Enter a filename in the GUI and select **START RECORDING**. The GUI saves the
file under `logs/`, adds `.csv` when omitted, and refuses to overwrite an
existing file. Each recording contains timestamps, commanded channels,
receiver link/failsafe state, raw ADC data, GPIO3 millivolts, and reconstructed
battery voltage. Commanded values are PC-side records, not receiver
acknowledgements.

For recording without the GUI:

```powershell
py -m host.log_telemetry --list-ports
py -m host.log_telemetry --port COM10 --duration 60
```

Omit `--duration` to record until Ctrl+C. The standalone recorder deliberately
sends only the safe locked channel state. GUI and standalone recorder cannot
run simultaneously because only one process can own the transmitter COM port.

## 8. Protocol limitations relevant to automation

- PC ASCII channel commands have no checksum or explicit malformed-command
  response.
- Receiver telemetry is 4 Hz and contains no returned channel values or
  per-command acknowledgement.
- The forward ESP-NOW destination is broadcast by default and is unencrypted.
- Structured PC lines are versioned through the identity and status filtering,
  but the forward binary control packet has no explicit version field.
- Human-readable diagnostic lines share the serial stream with protocol lines.
- Battery voltage needs hardware calibration; battery percentage is undefined.

A future automation-focused protocol should add an explicit forward packet
version, authenticated peers, and a documented error response for malformed
host commands.
