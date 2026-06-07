# SVEA AD3 SBUS RC Emulator

Analog Discovery 3 based SBUS RC emulator for the SVEA PX4 7-channel profile.

This project drives PX4-compatible SBUS from an AD3 digital output and provides:

- a Pygame 7-channel RC UI,
- a DWF runtime/device diagnostic,
- staged DIO loopback tests,
- SBUS loopback decoding before connecting to PX4.

## PX4/SVEA Profile

The generator is matched to the local SVEA PX4 Autopilot source:

- SBUS UART: `100000` baud, `8E2`.
- Physical output: inverted SBUS by default.
- Frame: 25 bytes, start byte `0x0f`, plain SBUS end byte `0x00`.
- PX4 channel scale: raw SBUS `200..1800` maps to `1000..2000 us`.
- SVEA board defaults: `RC_INPUT_PROTO=2` SBUS, `RC_CHAN_CNT=7`.

Channel layout:

| Channel | Control  | Meaning                              |
| ------- | -------- | ------------------------------------ |
| CH1     | Steering | 1000..2000 us                        |
| CH2     | Throttle | 1000..2000 us                        |
| CH3     | VR dial  | misc servo absolute dial             |
| CH4     | SWA      | gear button                          |
| CH5     | SWB      | left MAVROS, mid RC-only, right kill |
| CH6     | SWC      | misc0, none, misc1 selector          |
| CH7     | SWD      | arm button                           |

## Install

Install Digilent WaveForms with the SDK/runtime so this exists:

```bash
ls -l /Library/Frameworks/dwf.framework/dwf
```

Then create a virtualenv and install this project:

```bash
python3 -m venv .venv
source .venv/bin/activate
python3 -m pip install -e .
```

## Verify AD3 Access

```bash
svea-ad3-dwf-diag
```

Expected:

```text
FDwfEnum(all): rc=1 count=1
0: Analog Discovery 3 SN:... opened=0
```

WaveForms must be closed while Python owns the AD3.

## Staged Loopback Test

Wire:

```text
AD3 DIO1 -> AD3 DIO2
```

Run:

```bash
svea-ad3-dio-test --out-pin 1 --in-pin 2 --capture-rate 400000
```

This tests:

1. AD3 open and digital capability reporting.
2. Direct low/high output.
3. Direct low/high loopback toggling.
4. Custom digital output square wave.
5. Full PX4-compatible SBUS frame decode.

Expected final line:

```text
PASS: staged DIO and SBUS loopback tests passed
```

The AD3 reports a 2048-bit custom output buffer on DIO and a 16384-sample digital input buffer in this setup. The SBUS output fits because it uses `100 kHz * 20 ms = 2000` custom bits. The loopback capture uses `400 kHz` so two 20 ms frames fit in the input buffer.

## Run The UI

After loopback passes, connect PX4:

```text
AD3 DIO1 -> PX4 SBUS RX signal
AD3 GND  -> PX4 / receiver-port GND
```

Do not connect AD3 3V3/V+ to the PX4 receiver power rail for this emulator.

Run:

```bash
svea-ad3-sbus-ui --pin 1
```

Preview without opening the AD3:

```bash
svea-ad3-sbus-ui --no-ad3
```

## PX4 Checks

On the PX4 shell, check that RC is detected and mapped:

```bash
listener input_rc 5
listener manual_control_switches 5
```

For SVEA mode testing:

- SWB left / CH5 `1000 us`: MAVROS manual control accepted.
- SWB mid / CH5 `1500 us`: RC-only.
- SWB right / CH5 `2000 us`: kill asserted.

## Commands

```bash
svea-ad3-sbus --help
svea-ad3-sbus-ui --help
svea-ad3-sbus-verify --help
svea-ad3-dio-test --help
svea-ad3-dwf-diag
```
