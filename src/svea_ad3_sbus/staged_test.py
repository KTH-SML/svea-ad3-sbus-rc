#!/usr/bin/env python3
"""Staged AD3 DIO loopback test.

Wire:
  AD3 DIO1 -> AD3 DIO2

Stages:
  1. Open AD3 and report digital capabilities.
  2. Direct-drive output pin low/high and read both pins.
  3. Direct-drive repeated low/high toggles.
  4. Custom digital-out square wave capture.
  5. Full SBUS loopback decode.
"""

from __future__ import annotations

import argparse
import time
from ctypes import byref, c_int, c_uint, create_string_buffer

from .sbus import (
    SBUS_BAUD,
    build_sbus_frame,
    channel_defaults,
    configure_pattern,
    validate_frame_matches_px4,
    load_dwf,
    require_dwf,
    sbus_output_timing,
    sample_bits,
    uart_8e2_bits,
)
from .verify import capture_samples, configure_capture, decode_uart_8e2, pin_stats


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-pin", type=int, default=1, help="DIO pin to drive")
    parser.add_argument("--in-pin", type=int, default=2, help="DIO pin to observe")
    parser.add_argument("--rate", type=float, default=100_000.0, help="custom/SBUS output sample rate in Hz")
    parser.add_argument("--capture-rate", type=float, default=500_000.0, help="digital input capture sample rate in Hz")
    parser.add_argument("--frame-rate", type=float, default=50.0, help="SBUS frame rate in Hz")
    parser.add_argument("--uninverted", action="store_true", help="test normal idle-high UART instead of inverted SBUS")
    parser.add_argument("--steer", type=int, default=1500, help="CH1 steering us")
    parser.add_argument("--throttle", type=int, default=1500, help="CH2 throttle us")
    parser.add_argument("--vr", type=int, default=1500, help="CH3 misc dial us")
    parser.add_argument("--gear-button", type=int, default=1000, help="CH4 gear button us")
    parser.add_argument("--mode", choices=["mavros", "rc", "kill"], default="mavros", help="CH5 switch position")
    parser.add_argument("--misc-select", choices=["misc0", "none", "misc1"], default="none", help="CH6 selector")
    parser.add_argument("--arm", type=int, default=1000, help="CH7 arm switch/button us")
    return parser.parse_args()


def read_pins(dwf, hdwf) -> int:
    value = c_uint()
    require_dwf(dwf, dwf.FDwfDigitalIOStatus(hdwf), "FDwfDigitalIOStatus")
    require_dwf(dwf, dwf.FDwfDigitalIOInputStatus(hdwf, byref(value)), "FDwfDigitalIOInputStatus")
    return value.value


def bit(value: int, pin: int) -> int:
    return (value >> pin) & 1


def set_direct_output(dwf, hdwf, out_pin: int, level: int) -> int:
    mask = 1 << out_pin
    output = mask if level else 0
    require_dwf(dwf, dwf.FDwfDigitalIOOutputEnableSet(hdwf, c_uint(mask)), "FDwfDigitalIOOutputEnableSet")
    require_dwf(dwf, dwf.FDwfDigitalIOOutputSet(hdwf, c_uint(output)), "FDwfDigitalIOOutputSet")
    require_dwf(dwf, dwf.FDwfDigitalIOConfigure(hdwf), "FDwfDigitalIOConfigure")
    time.sleep(0.02)
    return read_pins(dwf, hdwf)


def stage_direct_levels(dwf, hdwf, out_pin: int, in_pin: int) -> None:
    print("\nStage 2: direct low/high")
    low = set_direct_output(dwf, hdwf, out_pin, 0)
    print(f"  drive DIO{out_pin}=0 -> read DIO{out_pin}={bit(low, out_pin)} DIO{in_pin}={bit(low, in_pin)} raw=0x{low:04x}")
    if bit(low, out_pin) != 0 or bit(low, in_pin) != 0:
        raise RuntimeError("direct low test failed")

    high = set_direct_output(dwf, hdwf, out_pin, 1)
    print(f"  drive DIO{out_pin}=1 -> read DIO{out_pin}={bit(high, out_pin)} DIO{in_pin}={bit(high, in_pin)} raw=0x{high:04x}")
    if bit(high, out_pin) != 1:
        raise RuntimeError(f"DIO{out_pin} did not read high when directly driven high")
    if bit(high, in_pin) != 1:
        raise RuntimeError(f"DIO{in_pin} did not read high; loopback wire is wrong/missing or pin numbering is wrong")


def stage_direct_toggle(dwf, hdwf, out_pin: int, in_pin: int) -> None:
    print("\nStage 3: direct toggle")
    observed = []
    for level in [0, 1] * 8:
        value = set_direct_output(dwf, hdwf, out_pin, level)
        observed.append(bit(value, in_pin))
    print("  expected:", " ".join(str(v) for v in ([0, 1] * 8)))
    print("  observed:", " ".join(str(v) for v in observed))
    if observed != [0, 1] * 8:
        raise RuntimeError("direct toggle loopback failed")


def stage_custom_square(dwf, hdwf, out_pin: int, in_pin: int, rate: float, capture_rate: float) -> None:
    print("\nStage 4: custom square wave")
    require_dwf(dwf, dwf.FDwfDigitalIOReset(hdwf), "FDwfDigitalIOReset")
    require_dwf(dwf, dwf.FDwfDigitalIOConfigure(hdwf), "FDwfDigitalIOConfigure")

    samples = 2000
    bits = []
    for i in range(samples):
        bits.append((i // 25) & 1)

    capture_samples_count = int(0.01 * capture_rate)
    configure_capture(dwf, hdwf, capture_rate, capture_samples_count)
    configure_pattern(dwf, hdwf, out_pin, rate, bits)
    time.sleep(0.02)
    captured = capture_samples(dwf, hdwf, capture_samples_count)
    high_out, trans_out = pin_stats(captured, out_pin)
    high_in, trans_in = pin_stats(captured, in_pin)
    print(f"  DIO{out_pin}: high={high_out}/{len(captured)} transitions={trans_out}")
    print(f"  DIO{in_pin}: high={high_in}/{len(captured)} transitions={trans_in}")
    if trans_out == 0:
        raise RuntimeError("custom digital output did not toggle on output pin")
    if trans_in == 0:
        raise RuntimeError("custom digital output toggled, but loopback input did not")


def stage_sbus(dwf, hdwf, args: argparse.Namespace) -> None:
    print("\nStage 5: SBUS loopback decode")
    inverted = not args.uninverted
    output_samples_per_bit, frame_samples = sbus_output_timing(args.rate, args.frame_rate)
    capture_samples_per_bit = int(round(args.capture_rate / SBUS_BAUD))
    if capture_samples_per_bit < 4:
        raise RuntimeError("--capture-rate must be at least 400 kHz for SBUS decode")

    channels = channel_defaults(args)
    expected = build_sbus_frame(channels)
    validate_frame_matches_px4(channels, expected)
    bits = sample_bits(uart_8e2_bits(expected), output_samples_per_bit, frame_samples, inverted)

    capture_count = int((2.0 / args.frame_rate) * args.capture_rate)
    configure_capture(dwf, hdwf, args.capture_rate, capture_count)
    configure_pattern(dwf, hdwf, args.out_pin, args.rate, bits)
    time.sleep(0.02)
    captured = capture_samples(dwf, hdwf, capture_count)
    decoded = decode_uart_8e2(captured, args.in_pin, capture_samples_per_bit, inverted)
    if decoded is None:
        high_out, trans_out = pin_stats(captured, args.out_pin)
        high_in, trans_in = pin_stats(captured, args.in_pin)
        print(f"  DIO{args.out_pin}: high={high_out}/{len(captured)} transitions={trans_out}")
        print(f"  DIO{args.in_pin}: high={high_in}/{len(captured)} transitions={trans_in}")
        raise RuntimeError("SBUS frame did not decode")
    if decoded != expected:
        print(f"  expected: {expected.hex(' ')}")
        print(f"  decoded:  {decoded.hex(' ')}")
        raise RuntimeError("SBUS frame bytes mismatch")
    print("  decoded one valid SBUS frame")
    print("  channels us:", " ".join(f"CH{i + 1}={value}" for i, value in enumerate(channels[:7])))


def main() -> int:
    args = parse_args()
    if args.out_pin == args.in_pin:
        raise RuntimeError("--out-pin and --in-pin must differ")

    dwf = load_dwf()
    version = create_string_buffer(16)
    dwf.FDwfGetVersion(version)
    print(f"Stage 1: open AD3")
    print(f"  DWF version: {version.value.decode(errors='replace')}")

    hdwf = c_int()
    require_dwf(dwf, dwf.FDwfDeviceOpen(c_int(-1), byref(hdwf)), "FDwfDeviceOpen")
    if hdwf.value == 0:
        err = create_string_buffer(512)
        dwf.FDwfGetLastErrorMsg(err)
        raise RuntimeError(f"failed to open AD3: {err.value.decode(errors='replace')}")

    try:
        require_dwf(dwf, dwf.FDwfDeviceAutoConfigureSet(hdwf, c_int(0)), "FDwfDeviceAutoConfigureSet")
        out_mask = c_uint()
        in_mask = c_uint()
        custom_bits = c_uint()
        require_dwf(dwf, dwf.FDwfDigitalIOOutputEnableInfo(hdwf, byref(out_mask)), "FDwfDigitalIOOutputEnableInfo")
        require_dwf(dwf, dwf.FDwfDigitalIOInputInfo(hdwf, byref(in_mask)), "FDwfDigitalIOInputInfo")
        require_dwf(dwf, dwf.FDwfDigitalOutDataInfo(hdwf, c_int(args.out_pin), byref(custom_bits)), "FDwfDigitalOutDataInfo")
        print(f"  output-enable mask: 0x{out_mask.value:04x}")
        print(f"  input mask:         0x{in_mask.value:04x}")
        print(f"  DIO{args.out_pin} custom max bits: {custom_bits.value}")
        print("  PX4 SBUS: 100000 baud, 8E2, RX inversion enabled, frame[0]=0x0f, frame[24]=0x00")
        print("  PX4 scale: raw 200..1800 -> 1000..2000 us")
        print("  SVEA board: RC_INPUT_PROTO=2(SBUS), RC_CHAN_CNT=7")

        stage_direct_levels(dwf, hdwf, args.out_pin, args.in_pin)
        stage_direct_toggle(dwf, hdwf, args.out_pin, args.in_pin)
        stage_custom_square(dwf, hdwf, args.out_pin, args.in_pin, args.rate, args.capture_rate)
        stage_sbus(dwf, hdwf, args)
        print("\nPASS: staged DIO and SBUS loopback tests passed")
        return 0
    finally:
        dwf.FDwfDigitalOutReset(hdwf)
        dwf.FDwfDigitalIOReset(hdwf)
        dwf.FDwfDigitalInReset(hdwf)
        dwf.FDwfDeviceCloseAll()


if __name__ == "__main__":
    raise SystemExit(main())
