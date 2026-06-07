#!/usr/bin/env python3
"""Diagnose Digilent WaveForms SDK visibility for AD3."""

from __future__ import annotations

import subprocess
from ctypes import byref, c_int, create_string_buffer
from pathlib import Path

from .sbus import dwf_candidates, load_dwf


def run(cmd: list[str]) -> str:
    try:
        return subprocess.check_output(cmd, text=True, stderr=subprocess.STDOUT)
    except (OSError, subprocess.CalledProcessError) as exc:
        return str(exc)


def main() -> int:
    print("DWF candidates:")
    for candidate in dwf_candidates():
        exists = candidate == "libdwf.so" or Path(candidate).is_file()
        print(f"  {'OK ' if exists else 'MISS'} {candidate}")

    print("\nUSB Digilent devices:")
    usb = run(["ioreg", "-p", "IOUSB", "-l", "-w0"])
    lines = [line.strip() for line in usb.splitlines() if "Digilent" in line or "1443" in line or "7003" in line]
    if lines:
        for line in lines:
            print(f"  {line}")
    else:
        print("  none found by ioreg")

    print("\nDWF enumeration:")
    try:
        dwf = load_dwf()
    except OSError as exc:
        print(f"  load failed: {exc}")
        return 1

    version = create_string_buffer(16)
    dwf.FDwfGetVersion(version)
    print(f"  version: {version.value.decode(errors='replace')}")

    count = c_int()
    rc = dwf.FDwfEnum(c_int(0), byref(count))
    print(f"  FDwfEnum(all): rc={rc} count={count.value}")

    for idx in range(count.value):
        name = create_string_buffer(64)
        serial = create_string_buffer(64)
        opened = c_int()
        dwf.FDwfEnumDeviceName(c_int(idx), name)
        dwf.FDwfEnumSN(c_int(idx), serial)
        dwf.FDwfEnumDeviceIsOpened(c_int(idx), byref(opened))
        print(
            f"  {idx}: {name.value.decode(errors='replace')} "
            f"{serial.value.decode(errors='replace')} opened={opened.value}"
        )

    if count.value == 0:
        print("\nConclusion:")
        print("  macOS may see the USB device, but the DWF runtime used by Python does not.")
        print("  Quit WaveForms, then rerun this. If count remains 0, install the WaveForms SDK/runtime")
        print("  so /Library/Frameworks/dwf.framework/dwf exists; the app-bundled framework is not enough here.")

    return 0 if count.value > 0 else 2


if __name__ == "__main__":
    raise SystemExit(main())
