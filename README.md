# Flapping-Wing SBUS Controller

Wireless control and battery telemetry for a flapping-wing robot. A
USB-connected ESP32 transmitter receives commands from Python, sends them over
ESP-NOW, and receives telemetry from an ESP32-C3 SuperMini receiver. The
receiver generates inverted SBUS for the flight controller.

```text
Control:
Python / optional GUI -> USB serial -> ESP32 transmitter -> ESP-NOW
                      -> ESP32-C3 receiver -> GPIO4 SBUS -> flight controller

Telemetry:
GPIO3 ADC -> ESP32-C3 receiver -> ESP-NOW -> ESP32 transmitter
          -> USB serial -> Python / optional GUI
```

## Documentation

- [PC serial protocol](docs/serial-protocol.md) is the authoritative interface
  for Python automation.
- [Engineering document](docs/flapping-wing-sbus-controller.tex) covers the
  complete signal paths, wiring, failsafe behavior, firmware, and references.

The firmware is authoritative if implementation and documentation disagree.
Protocol changes must update both documents.

## Quick start

Run from the repository root:

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe -m host.gui
```

List ports or run the headless safe-state logger:

```powershell
.\.venv\Scripts\python.exe -m host.log_telemetry --list-ports
.\.venv\Scripts\python.exe -m host.log_telemetry --port COM10 --duration 60
```

Open `flapping-wing-sbus-controller.code-workspace` in VS Code to expose the
repository and both independent PlatformIO projects. Alternatively, open
`firmware/transmitter/` or `firmware/receiver/` directly.

## Control contract

The PC sends six dimensionless values from 1000 to 2000 in this order:

| Position | Channel | Function | Safe value |
|---:|---|---|---:|
| 1 | CH1 | Yaw | 1500 |
| 2 | CH2 | Pitch | 1500 |
| 3 | CH3 | Throttle | 1000 |
| 4 | CH5 | Trim 1 / servo center | 1500 |
| 5 | CH6 | Trim 2 / servo center | 1500 |
| 6 | CH8 | Throttle lock / arm | 1000 |

Safe command:

```text
<1500,1500,1000,1500,1500,1000>\n
```

These are host control units, not PWM microseconds. The receiver maps them to
raw SBUS counts.

## Wiring

| Connection | Purpose |
|---|---|
| PC to transmitter USB | Control, telemetry, and transmitter power |
| Receiver GPIO4 to flight-controller SBUS input | Inverted SBUS |
| Receiver GPIO3 to battery-divider midpoint | Battery measurement |
| Receiver GND to flight-controller GND and battery negative | Common reference |

Battery divider:

```text
Battery + ---- 300 kOhm ----+-- ESP32-C3 GPIO3
                            |
                          100 kOhm
                            |
Battery - ------------------+-- ESP32-C3 GND
```

Never connect raw battery voltage directly to GPIO3. Verify divider orientation
and voltage with a multimeter before connecting the receiver.

## Flashing

The transmitter and receiver are separate PlatformIO projects. PlatformIO
normally auto-detects the port when only one compatible MCU is connected.

Transmitter:

```powershell
cd firmware\transmitter
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run --target upload
```

Receiver:

```powershell
cd firmware\receiver
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run --target upload
```

If both boards are connected, append `--upload-port COM10` or the appropriate
port. Changes to `firmware/link/esp_now_link.h` require both MCUs to be
reflashed.

## One-board wired ESP32-C3 diagnostic

Use the isolated diagnostic to test the flight-controller SBUS wire and GPIO3
voltage measurement without ESP-NOW:

| ESP32-C3 connection | Diagnostic purpose |
|---|---|
| USB to PC | Firmware control, status, and receiver power |
| GPIO4 to flight-controller SBUS input | Inverted SBUS test signal |
| GND to flight-controller GND and battery negative | Common SBUS/ADC reference |
| GPIO3 to 300 kOhm/100 kOhm divider midpoint | Battery measurement |
| Flight-controller 5 V | **Leave disconnected while USB powers the C3** |

Flash the diagnostic onto the ESP32-C3:

```powershell
cd diagnostics\wired_c3
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run --target upload --upload-port COM11
cd ..\..
```

Run its dedicated GUI from the repository root:

```powershell
.\.venv\Scripts\python.exe -m diagnostics.wired_c3.gui
```

The firmware sends no SBUS for the first five seconds, then requires five
consecutive commands with CH8 locked. A 500 ms host-command timeout stops UART1
and returns GPIO4 to high impedance. The GUI displays GPIO3 raw ADC, pin
voltage, and battery voltage reconstructed with the 4:1 divider ratio.

This test replaces the wireless receiver firmware temporarily. Restore normal
operation afterward by flashing `firmware/receiver/` back onto the C3.

## Operation and recording

The GUI verifies transmitter identity before enabling controls, monitors
receiver telemetry, locks CH8 during faults, runs the optional ramped benchmark,
displays battery voltage, and records receiver-confirmed CSV data.

Enter a CSV filename in the GUI before selecting **START RECORDING**. The
`.csv` extension is added when omitted, existing files are never overwritten,
and recordings are saved under `logs/`. The standalone logger sends only the
safe locked command while recording. The GUI and logger cannot use the same COM
port simultaneously. Recorded CSV files are ignored by Git.

## Safety summary

- Keep CH8 at 1000 until the mechanism and link are verified.
- The receiver sends no SBUS during its initial five-second qualification
  period.
- Loss of PC commands or ESP-NOW packets disables receiver SBUS output.
- Recovery requires healthy packets with CH8 locked before SBUS resumes.
- Restrain the mechanism when testing new wiring or firmware.

See the engineering document for the complete startup, fault, and recovery
state definitions.

## Repository structure

```text
docs/                         Protocol and engineering documentation
diagnostics/wired_c3/         One-board USB-to-SBUS and GPIO3 hardware test
firmware/link/                ESP-NOW link packet definitions
firmware/transmitter/         ESP32 transmitter PlatformIO project
firmware/receiver/            ESP32-C3 receiver PlatformIO project
host/gui.py                   Optional graphical PC client
host/log_telemetry.py         Headless safe-state CSV recorder
host/pc_tools/                Shared Python protocol and serial modules
logs/                         Local receiver-confirmed CSV recordings
tests/                        PC protocol, logging, and structure tests
requirements.txt              Python runtime dependency list
pyproject.toml                Python project metadata and entry points
```

The legacy wired prototype is retained only for historical reference.

## Local checks

```powershell
.\.venv\Scripts\python.exe -m unittest discover -s tests -v

cd firmware\transmitter
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run

cd ..\receiver
& "$env:USERPROFILE\.platformio\penv\Scripts\platformio.exe" run
```

## Build the engineering PDF

Install MiKTeX or another LaTeX distribution providing `pdflatex`, then run
from the repository root:

```powershell
New-Item -ItemType Directory -Force output\pdf | Out-Null
pdflatex -interaction=nonstopmode -halt-on-error `
  -output-directory=output\pdf docs\flapping-wing-sbus-controller.tex
pdflatex -interaction=nonstopmode -halt-on-error `
  -output-directory=output\pdf docs\flapping-wing-sbus-controller.tex
```

The generated file is
`output/pdf/flapping-wing-sbus-controller.pdf`. The entire `output/` directory
is ignored by Git. Two passes resolve the table of contents, citations, and
cross-references.

The repository's VS Code settings use the same two-pass `pdflatex` recipe and
clean auxiliary files with file globs. Building and cleaning therefore do not
require `latexmk` or Perl.

No command in this repository pushes changes or uploads firmware unless the
corresponding command is run explicitly.
