#!/usr/bin/env python3
"""Loopback-verify AD3 SBUS output by decoding captured DIO samples.

Wire:
  AD3 DIO0 -> AD3 DIO1

This does not verify the PX4 input circuit, but it does verify that DWF is
driving a decodable inverted SBUS UART frame on the selected output pin.
"""

from __future__ import annotations

import argparse
import sys
import time
from ctypes import byref, c_double, c_int, c_ubyte, c_uint16, create_string_buffer
from pathlib import Path

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

SDK_PY = Path("/Applications/WaveForms.app/Contents/Resources/SDK/samples/py")
if SDK_PY.is_dir():
    sys.path.insert(0, str(SDK_PY))

from dwfconstants import DwfStateDone, acqmodeRecord, trigsrcNone  # type: ignore  # noqa: E402


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-pin", type=int, default=0, help="DIO pin to drive")
    parser.add_argument("--in-pin", type=int, default=1, help="DIO pin to capture")
    parser.add_argument("--rate", type=float, default=100_000.0, help="output sample rate in Hz")
    parser.add_argument("--capture-rate", type=float, default=500_000.0, help="capture sample rate in Hz")
    parser.add_argument("--frame-rate", type=float, default=50.0, help="SBUS frame repeat rate in Hz")
    parser.add_argument("--uninverted", action="store_true", help="verify normal idle-high UART instead of inverted SBUS")
    parser.add_argument("--steer", type=int, default=1500, help="CH1 steering us")
    parser.add_argument("--throttle", type=int, default=1500, help="CH2 throttle us")
    parser.add_argument("--vr", type=int, default=1500, help="CH3 misc dial us")
    parser.add_argument("--gear-button", type=int, default=1000, help="CH4 gear button us")
    parser.add_argument("--mode", choices=["mavros", "rc", "kill"], default="mavros", help="CH5 switch position")
    parser.add_argument("--misc-select", choices=["misc0", "none", "misc1"], default="none", help="CH6 selector")
    parser.add_argument("--arm", type=int, default=1000, help="CH7 arm switch/button us")
    return parser.parse_args()


def configure_capture(dwf, hdwf, rate: float, samples: int) -> None:
    hz_di = c_double()
    require_dwf(dwf, dwf.FDwfDigitalInInternalClockInfo(hdwf, byref(hz_di)), "FDwfDigitalInInternalClockInfo")
    divider = max(1, int(round(hz_di.value / rate)))

    max_samples = c_int()
    require_dwf(dwf, dwf.FDwfDigitalInBufferSizeInfo(hdwf, byref(max_samples)), "FDwfDigitalInBufferSizeInfo")
    if samples > max_samples.value:
        raise RuntimeError(f"capture needs {samples} samples, but AD3 reports max {max_samples.value}")

    require_dwf(dwf, dwf.FDwfDigitalInAcquisitionModeSet(hdwf, acqmodeRecord), "FDwfDigitalInAcquisitionModeSet")
    require_dwf(dwf, dwf.FDwfDigitalInDividerSet(hdwf, c_int(divider)), "FDwfDigitalInDividerSet")
    require_dwf(dwf, dwf.FDwfDigitalInSampleFormatSet(hdwf, c_int(16)), "FDwfDigitalInSampleFormatSet")
    require_dwf(dwf, dwf.FDwfDigitalInBufferSizeSet(hdwf, c_int(samples)), "FDwfDigitalInBufferSizeSet")
    require_dwf(dwf, dwf.FDwfDigitalInTriggerSourceSet(hdwf, trigsrcNone), "FDwfDigitalInTriggerSourceSet")
    require_dwf(dwf, dwf.FDwfDigitalInTriggerPositionSet(hdwf, c_int(samples)), "FDwfDigitalInTriggerPositionSet")
    require_dwf(dwf, dwf.FDwfDigitalInInputOrderSet(hdwf, c_int(0)), "FDwfDigitalInInputOrderSet")
    require_dwf(dwf, dwf.FDwfDigitalInConfigure(hdwf, c_int(1), c_int(1)), "FDwfDigitalInConfigure")


def capture_samples(dwf, hdwf, samples: int, timeout_s: float = 2.0) -> list[int]:
    status = c_ubyte()
    available = c_int()
    lost = c_int()
    corrupted = c_int()
    captured: list[int] = []
    deadline = time.monotonic() + timeout_s

    while len(captured) < samples and time.monotonic() < deadline:
        if dwf.FDwfDigitalInStatus(hdwf, c_int(1), byref(status)) == 0:
            raise RuntimeError("FDwfDigitalInStatus failed")

        dwf.FDwfDigitalInStatusRecord(hdwf, byref(available), byref(lost), byref(corrupted))
        if lost.value:
            raise RuntimeError("capture lost samples")
        if corrupted.value:
            raise RuntimeError("capture corrupted samples")

        if available.value > 0:
            count = min(available.value, samples - len(captured))
            buf = (c_uint16 * count)()
            dwf.FDwfDigitalInStatusData(hdwf, byref(buf), c_int(2 * count))
            captured.extend(int(v) for v in buf)

        if status.value == DwfStateDone.value and available.value == 0:
            break

        time.sleep(0.002)

    if len(captured) < samples:
        raise RuntimeError(f"capture timed out: got {len(captured)} of {samples} samples")
    return captured


def pin_stats(samples: list[int], pin: int) -> tuple[int, int]:
    values = [(sample >> pin) & 1 for sample in samples]
    high = sum(values)
    transitions = sum(1 for a, b in zip(values, values[1:]) if a != b)
    return high, transitions


def decode_uart_8e2(samples: list[int], in_pin: int, samples_per_bit: int, inverted: bool) -> bytes | None:
    pin_samples = [((sample >> in_pin) & 1) ^ (1 if inverted else 0) for sample in samples]
    need_bits = 25 * 12

    for start in range(0, max(0, len(pin_samples) - need_bits * samples_per_bit)):
        for phase in range(samples_per_bit):
            bits = []
            base = start + phase
            for bit_index in range(need_bits):
                center = base + bit_index * samples_per_bit + samples_per_bit // 2
                if center >= len(pin_samples):
                    break
                bits.append(pin_samples[center])
            if len(bits) != need_bits:
                continue

            data = bytearray()
            ok = True
            for byte_index in range(25):
                chunk = bits[byte_index * 12 : (byte_index + 1) * 12]
                if chunk[0] != 0 or chunk[10] != 1 or chunk[11] != 1:
                    ok = False
                    break
                value = sum(chunk[1 + bit] << bit for bit in range(8))
                parity = value.bit_count() & 1
                if chunk[9] != parity:
                    ok = False
                    break
                data.append(value)

            if ok and data[0] == 0x0F and data[24] == 0x00:
                return bytes(data)

    return None


def main() -> int:
    args = parse_args()
    inverted = not args.uninverted
    output_samples_per_bit, frame_samples = sbus_output_timing(args.rate, args.frame_rate)
    capture_samples_per_bit = int(round(args.capture_rate / SBUS_BAUD))
    capture_count = int((2.0 / args.frame_rate) * args.capture_rate)

    if args.out_pin == args.in_pin:
        raise RuntimeError("--out-pin and --in-pin must be different")
    if capture_samples_per_bit < 4:
        raise RuntimeError("--capture-rate must be at least 400 kHz for reliable decode")

    channels = channel_defaults(args)
    expected = build_sbus_frame(channels)
    validate_frame_matches_px4(channels, expected)
    bits = sample_bits(uart_8e2_bits(expected), output_samples_per_bit, frame_samples, inverted)

    dwf = load_dwf()
    version = create_string_buffer(16)
    dwf.FDwfGetVersion(version)
    print(f"DWF version: {version.value.decode(errors='replace')}")

    hdwf = c_int()
    dwf.FDwfDeviceOpen(c_int(-1), byref(hdwf))
    if hdwf.value == 0:
        err = create_string_buffer(512)
        dwf.FDwfGetLastErrorMsg(err)
        raise RuntimeError(f"failed to open AD3: {err.value.decode(errors='replace')}")

    try:
        dwf.FDwfDeviceAutoConfigureSet(hdwf, c_int(0))
        configure_pattern(dwf, hdwf, args.out_pin, args.rate, bits)
        time.sleep(0.05)
        configure_capture(dwf, hdwf, args.capture_rate, capture_count)
        samples = capture_samples(dwf, hdwf, capture_count)
    finally:
        dwf.FDwfDigitalOutReset(hdwf)
        dwf.FDwfDigitalInReset(hdwf)
        dwf.FDwfDeviceCloseAll()

    decoded = decode_uart_8e2(samples, args.in_pin, capture_samples_per_bit, inverted)
    if decoded is None:
        print("FAIL: captured data did not decode as one complete SBUS 8E2 frame")
        print("Observed digital input activity:")
        for pin in range(16):
            high, transitions = pin_stats(samples, pin)
            if transitions or pin in (args.out_pin, args.in_pin):
                print(f"  DIO{pin}: high={high}/{len(samples)} transitions={transitions}")
        alt = decode_uart_8e2(samples, args.in_pin, capture_samples_per_bit, not inverted)
        if alt is not None:
            print(f"  DIO{args.in_pin} decodes if inversion is flipped; try --uninverted or check expected polarity")
        return 1
    if decoded != expected:
        print("FAIL: decoded SBUS frame differs from generated frame")
        print(f"expected: {expected.hex(' ')}")
        print(f"decoded:  {decoded.hex(' ')}")
        return 1

    print(f"PASS: DIO{args.out_pin} -> DIO{args.in_pin} decoded one valid SBUS frame")
    print(f"frame: {decoded.hex(' ')}")
    print("channels us:", " ".join(f"CH{i + 1}={value}" for i, value in enumerate(channels[:7])))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
