#!/usr/bin/env python3
"""
Generate SBUS frames on a Digilent Analog Discovery 3 digital output.

Default output:
  DIO0: SBUS signal

Wire:
  AD3 GND  -> flight-controller RC GND
  AD3 DIO0 -> flight-controller RC signal

Do not connect AD3 digital output to an RC 5 V supply pin.
"""

from __future__ import annotations

import argparse
import sys
import time
from ctypes import byref, c_double, c_int, c_ubyte, cdll, create_string_buffer
from pathlib import Path


SDK_PY = Path("/Applications/WaveForms.app/Contents/Resources/SDK/samples/py")
if SDK_PY.is_dir():
    sys.path.insert(0, str(SDK_PY))

from dwfconstants import (  # type: ignore  # noqa: E402
    DwfDigitalOutIdleZet,
    DwfDigitalOutTypeCustom,
)


SBUS_FRAME_BYTES = 25
SBUS_BAUD = 100_000
SBUS_BITS_PER_BYTE = 12  # start + 8 data + even parity + 2 stop
SBUS_CHANNELS = 16
SBUS_PX4_FRAME_RATE_HZ = 50.0
SBUS_PX4_RANGE_MIN = 200.0
SBUS_PX4_RANGE_MAX = 1800.0
SBUS_PX4_TARGET_MIN = 1000.0
SBUS_PX4_TARGET_MAX = 2000.0
SBUS_PX4_SCALE_FACTOR = (SBUS_PX4_TARGET_MAX - SBUS_PX4_TARGET_MIN) / (SBUS_PX4_RANGE_MAX - SBUS_PX4_RANGE_MIN)
SBUS_PX4_SCALE_OFFSET = int(SBUS_PX4_TARGET_MIN - (SBUS_PX4_SCALE_FACTOR * SBUS_PX4_RANGE_MIN + 0.5))


def dwf_candidates() -> list[str]:
    if sys.platform.startswith("darwin"):
        return [
            "/Library/Frameworks/dwf.framework/dwf",
            "/Applications/WaveForms.app/Contents/Frameworks/dwf.framework/dwf",
            "/Applications/WaveForms.app/Contents/Frameworks/dwf.framework/Versions/A/dwf",
        ]

    if sys.platform.startswith("win"):
        return ["dwf"]

    return ["libdwf.so"]


def load_dwf():
    if sys.platform.startswith("win"):
        return cdll.dwf

    for candidate in dwf_candidates():
        if candidate == "libdwf.so" or Path(candidate).is_file():
            return cdll.LoadLibrary(candidate)

    raise OSError("DWF framework not found; install the WaveForms SDK/runtime")


def us_to_sbus_raw(value_us: int) -> int:
    """Convert PX4-style RC microseconds to SBUS raw channel units.

    Mirrors PX4 src/lib/rc/sbus.cpp:
      raw 200..1800 -> target 1000..2000 us
    """
    value_us = max(1000, min(2000, value_us))
    return int(round((value_us - SBUS_PX4_SCALE_OFFSET) / SBUS_PX4_SCALE_FACTOR))


def sbus_raw_to_us(value: int) -> int:
    """Mirror PX4 sbus_decode raw-to-us conversion."""
    return int(round(value * SBUS_PX4_SCALE_FACTOR) + SBUS_PX4_SCALE_OFFSET)


def build_sbus_frame(channels_us: list[int], failsafe: bool = False, frame_lost: bool = False) -> bytes:
    if len(channels_us) != SBUS_CHANNELS:
        raise ValueError(f"expected {SBUS_CHANNELS} channels, got {len(channels_us)}")

    channels = [max(0, min(0x07FF, us_to_sbus_raw(v))) for v in channels_us]
    frame = bytearray(SBUS_FRAME_BYTES)
    frame[0] = 0x0F

    bit_index = 0
    for value in channels:
        for bit in range(11):
            if value & (1 << bit):
                byte_index = 1 + bit_index // 8
                frame[byte_index] |= 1 << (bit_index % 8)
            bit_index += 1

    flags = 0
    if frame_lost:
        flags |= 1 << 2
    if failsafe:
        flags |= 1 << 3
    frame[23] = flags
    frame[24] = 0x00
    return bytes(frame)


def decode_sbus_frame_like_px4(frame: bytes, max_channels: int = SBUS_CHANNELS) -> list[int]:
    if len(frame) != SBUS_FRAME_BYTES:
        raise ValueError(f"expected {SBUS_FRAME_BYTES} bytes, got {len(frame)}")
    if frame[0] != 0x0F:
        raise ValueError("invalid PX4 SBUS start byte")
    if frame[24] not in (0x00, 0x04, 0x14, 0x24, 0x34):
        raise ValueError("invalid PX4 SBUS end byte")

    channel_count = min(max_channels, SBUS_CHANNELS)
    values: list[int] = []
    bit_index = 0
    for _ in range(channel_count):
        raw = 0
        for bit in range(11):
            byte_index = 1 + bit_index // 8
            if frame[byte_index] & (1 << (bit_index % 8)):
                raw |= 1 << bit
            bit_index += 1
        values.append(sbus_raw_to_us(raw))
    return values


def validate_frame_matches_px4(channels_us: list[int], frame: bytes, max_error_us: int = 1) -> None:
    decoded = decode_sbus_frame_like_px4(frame, SBUS_CHANNELS)
    for index, (expected, actual) in enumerate(zip(channels_us, decoded), start=1):
        if abs(expected - actual) > max_error_us:
            raise RuntimeError(
                f"PX4 SBUS decode mismatch on CH{index}: expected {expected} us, decoded {actual} us"
            )


def even_parity_bit(byte: int) -> int:
    return byte.bit_count() & 1


def uart_8e2_bits(data: bytes) -> list[int]:
    bits: list[int] = []
    for byte in data:
        bits.append(0)  # start
        for bit in range(8):
            bits.append((byte >> bit) & 1)
        bits.append(even_parity_bit(byte))
        bits.extend([1, 1])  # stop bits
    return bits


def sample_bits(bits: list[int], samples_per_bit: int, frame_samples: int, inverted: bool) -> list[int]:
    if samples_per_bit < 1:
        raise ValueError("samples_per_bit must be >= 1")

    idle = 1
    out: list[int] = []
    for bit in bits:
        value = bit ^ (1 if inverted else 0)
        out.extend([value] * samples_per_bit)

    idle_value = idle ^ (1 if inverted else 0)
    if len(out) > frame_samples:
        raise ValueError("frame does not fit in selected frame period/sample rate")

    out.extend([idle_value] * (frame_samples - len(out)))
    return out


def sbus_output_timing(rate: float, frame_rate: float) -> tuple[int, int]:
    samples_per_bit_float = rate / SBUS_BAUD
    samples_per_bit = int(round(samples_per_bit_float))
    if samples_per_bit < 1 or abs(samples_per_bit_float - samples_per_bit) > 1e-9:
        raise RuntimeError(f"--rate must be an integer multiple of PX4 SBUS baud {SBUS_BAUD}")

    frame_samples_float = rate / frame_rate
    frame_samples = int(round(frame_samples_float))
    if frame_samples < SBUS_FRAME_BYTES * SBUS_BITS_PER_BYTE * samples_per_bit:
        raise RuntimeError("SBUS frame does not fit in selected --frame-rate")
    if abs(frame_samples_float - frame_samples) > 1e-9:
        raise RuntimeError("--rate / --frame-rate must be an integer number of DWF samples")

    return samples_per_bit, frame_samples


def make_bit_buffer(bits: list[int]):
    raw = (c_ubyte * ((len(bits) + 7) // 8))()
    for i, bit in enumerate(bits):
        if bit:
            raw[i >> 3] |= 1 << (i & 7)
    return raw


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pin", type=int, default=0, help="AD3 DIO pin to drive")
    parser.add_argument("--rate", type=float, default=100_000.0, help="pattern sample rate in Hz")
    parser.add_argument("--frame-rate", type=float, default=SBUS_PX4_FRAME_RATE_HZ, help="SBUS frame repeat rate in Hz")
    parser.add_argument("--uninverted", action="store_true", help="emit normal idle-high UART instead of physical inverted SBUS")
    parser.add_argument("--seconds", type=float, default=0.0, help="stop after this many seconds; 0 means run until Ctrl-C")
    parser.add_argument("--pulse-arm", action="store_true", help="hold CH7 high for the first 0.6 s, then low")
    parser.add_argument("--steer", type=int, default=1500, help="CH1 steering us")
    parser.add_argument("--throttle", type=int, default=1500, help="CH2 throttle us")
    parser.add_argument("--vr", type=int, default=1500, help="CH3 misc dial us")
    parser.add_argument("--gear-button", type=int, default=1000, help="CH4 gear button us")
    parser.add_argument("--mode", choices=["mavros", "rc", "kill"], default="mavros", help="CH5 switch position")
    parser.add_argument("--misc-select", choices=["misc0", "none", "misc1"], default="none", help="CH6 selector")
    parser.add_argument("--arm", type=int, default=1000, help="CH7 arm switch/button us")
    return parser.parse_args()


def channel_defaults(args: argparse.Namespace) -> list[int]:
    mode_ch5 = {"mavros": 1000, "rc": 1500, "kill": 2000}[args.mode]
    misc_ch6 = {"misc0": 1000, "none": 1500, "misc1": 2000}[args.misc_select]
    channels = [1500] * SBUS_CHANNELS
    channels[0] = args.steer
    channels[1] = args.throttle
    channels[2] = args.vr
    channels[3] = args.gear_button
    channels[4] = mode_ch5
    channels[5] = misc_ch6
    channels[6] = args.arm
    return channels


def get_last_error(dwf) -> str:
    err = create_string_buffer(512)
    dwf.FDwfGetLastErrorMsg(err)
    return err.value.decode(errors="replace")


def require_dwf(dwf, ok: int, call: str) -> None:
    if ok == 0:
        raise RuntimeError(f"{call} failed: {get_last_error(dwf)}")


def configure_pattern(dwf, hdwf, pin: int, rate: float, bits: list[int]):
    hz_sys = c_double()
    require_dwf(dwf, dwf.FDwfDigitalOutInternalClockInfo(hdwf, byref(hz_sys)), "FDwfDigitalOutInternalClockInfo")
    divider = max(1, int(round(hz_sys.value / rate)))

    max_bits = c_int()
    require_dwf(dwf, dwf.FDwfDigitalOutDataInfo(hdwf, c_int(pin), byref(max_bits)), "FDwfDigitalOutDataInfo")
    if len(bits) > max_bits.value:
        raise RuntimeError(
            f"DIO{pin} custom pattern is {len(bits)} bits, but AD3 reports max {max_bits.value}; "
            "lower --rate or raise --frame-rate"
        )

    data = make_bit_buffer(bits)

    require_dwf(dwf, dwf.FDwfDigitalOutEnableSet(hdwf, c_int(pin), c_int(1)), "FDwfDigitalOutEnableSet")
    require_dwf(dwf, dwf.FDwfDigitalOutIdleSet(hdwf, c_int(pin), DwfDigitalOutIdleZet), "FDwfDigitalOutIdleSet")
    require_dwf(dwf, dwf.FDwfDigitalOutTypeSet(hdwf, c_int(pin), DwfDigitalOutTypeCustom), "FDwfDigitalOutTypeSet")
    require_dwf(dwf, dwf.FDwfDigitalOutDividerSet(hdwf, c_int(pin), c_int(divider)), "FDwfDigitalOutDividerSet")
    require_dwf(dwf, dwf.FDwfDigitalOutDataSet(hdwf, c_int(pin), byref(data), c_int(len(bits))), "FDwfDigitalOutDataSet")
    require_dwf(dwf, dwf.FDwfDigitalOutRepeatSet(hdwf, c_int(0)), "FDwfDigitalOutRepeatSet")
    require_dwf(dwf, dwf.FDwfDigitalOutRunSet(hdwf, c_double(len(bits) / rate)), "FDwfDigitalOutRunSet")
    require_dwf(dwf, dwf.FDwfDigitalOutConfigure(hdwf, c_int(1)), "FDwfDigitalOutConfigure")


def main() -> int:
    args = parse_args()
    inverted = not args.uninverted
    samples_per_bit, frame_samples = sbus_output_timing(args.rate, args.frame_rate)

    channels = channel_defaults(args)
    frame = build_sbus_frame(channels)
    validate_frame_matches_px4(channels, frame)
    bits = sample_bits(uart_8e2_bits(frame), samples_per_bit, frame_samples, inverted)

    dwf = load_dwf()
    version = create_string_buffer(16)
    dwf.FDwfGetVersion(version)
    print(f"DWF version: {version.value.decode(errors='replace')}")

    hdwf = c_int()
    dwf.FDwfDeviceOpen(c_int(-1), byref(hdwf))
    if hdwf.value == 0:
        err = create_string_buffer(512)
        dwf.FDwfGetLastErrorMsg(err)
        print(f"failed to open AD3: {err.value.decode(errors='replace')}", file=sys.stderr)
        return 1

    try:
        dwf.FDwfDeviceAutoConfigureSet(hdwf, c_int(0))
        print(
            f"DIO{args.pin}: SBUS {args.frame_rate:g} Hz, sample_rate={args.rate:g} Hz, "
            f"samples_per_bit={samples_per_bit}, inverted={inverted}, mode={args.mode}"
        )
        configure_pattern(dwf, hdwf, args.pin, args.rate, bits)

        started = time.monotonic()
        if args.pulse_arm:
            time.sleep(0.6)
            channels[6] = 1000
            frame = build_sbus_frame(channels)
            validate_frame_matches_px4(channels, frame)
            bits = sample_bits(uart_8e2_bits(frame), samples_per_bit, frame_samples, inverted)
            configure_pattern(dwf, hdwf, args.pin, args.rate, bits)
            print("CH7 arm pulse complete; CH7 returned low")

        while args.seconds <= 0 or time.monotonic() - started < args.seconds:
            time.sleep(0.2)

    except KeyboardInterrupt:
        pass
    finally:
        dwf.FDwfDigitalOutReset(hdwf)
        dwf.FDwfDeviceCloseAll()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
