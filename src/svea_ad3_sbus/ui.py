#!/usr/bin/env python3
"""
Pygame UI for streaming a documented 7-channel SVEA RC profile over SBUS
through an Analog Discovery 3.

Wire:
  AD3 GND  -> flight-controller RC GND
  AD3 DIO0 -> flight-controller RC signal

Do not connect AD3 digital output to an RC 5 V supply pin.
"""

from __future__ import annotations

import argparse
import math
import time
from ctypes import byref, c_int, create_string_buffer
from dataclasses import dataclass
from pathlib import Path


from .sbus import (
    SBUS_BAUD,
    build_sbus_frame,
    configure_pattern,
    load_dwf,
    sbus_output_timing,
    sample_bits,
    uart_8e2_bits,
    validate_frame_matches_px4,
)


WIDTH = 1280
HEIGHT = 820
BG = (18, 20, 24)
PANEL = (32, 36, 43)
PANEL2 = (41, 46, 55)
LINE = (77, 86, 101)
TEXT = (237, 241, 245)
MUTED = (150, 160, 174)
ACCENT = (74, 171, 255)
GREEN = (34, 197, 94)
AMBER = (245, 158, 11)
RED = (239, 68, 68)
WHITE = (255, 255, 255)


class TextFont:
    """Small wrapper that avoids pygame.font on Python builds where it is broken."""

    def __init__(self, pygame_module, size: int) -> None:
        import pygame._freetype as ft

        ft.init()
        font_path = Path(pygame_module.__file__).resolve().parent / "freesansbold.ttf"
        if not font_path.is_file():
            raise RuntimeError(f"pygame bundled font not found: {font_path}")

        self._font = ft.Font(str(font_path), size)
        self._font.antialiased = True

    def render(self, text: str, antialias, color):
        surface, _ = self._font.render(str(text), color)
        return surface


def clamp(value: int, low=1000, high=2000) -> int:
    return max(low, min(high, value))


@dataclass
class RcState:
    # SVEA documented 7-channel profile:
    # CH1 steer, CH2 throttle, CH3 VR, CH4 SWA gear button,
    # CH5 SWB mode/kill, CH6 SWC misc selector, CH7 SWD arm button.
    steer: int = 1500
    throttle: int = 1500
    vr: int = 1500
    swa_gear: int = 1000
    swb_mode: str = "mavros"
    swc_misc: str = "none"
    swd_arm: int = 1000

    def channels(self) -> list[int]:
        ch = [1500] * 16
        ch[0] = self.steer
        ch[1] = self.throttle
        ch[2] = self.vr
        ch[3] = self.swa_gear
        ch[4] = {"mavros": 1000, "rc": 1500, "kill": 2000}[self.swb_mode]
        ch[5] = {"misc0": 1000, "none": 1500, "misc1": 2000}[self.swc_misc]
        ch[6] = self.swd_arm
        return ch


class SbusOutput:
    def __init__(self, pin: int, rate: float, frame_rate: float, inverted: bool) -> None:
        self.pin = pin
        self.rate = rate
        self.frame_rate = frame_rate
        self.inverted = inverted
        self.samples_per_bit, self.frame_samples = sbus_output_timing(rate, frame_rate)
        self.last_channels: list[int] | None = None
        self.dwf = load_dwf()
        self.hdwf = c_int()

        version = create_string_buffer(16)
        self.dwf.FDwfGetVersion(version)
        print(f"DWF version: {version.value.decode(errors='replace')}")

        self.dwf.FDwfDeviceOpen(c_int(-1), byref(self.hdwf))
        if self.hdwf.value == 0:
            err = create_string_buffer(512)
            self.dwf.FDwfGetLastErrorMsg(err)
            raise RuntimeError(f"failed to open AD3: {err.value.decode(errors='replace')}")

        self.dwf.FDwfDeviceAutoConfigureSet(self.hdwf, c_int(0))

    def send(self, channels: list[int]) -> None:
        if channels == self.last_channels:
            return

        frame = build_sbus_frame(channels)
        validate_frame_matches_px4(channels, frame)
        bits = sample_bits(uart_8e2_bits(frame), self.samples_per_bit, self.frame_samples, self.inverted)
        configure_pattern(self.dwf, self.hdwf, self.pin, self.rate, bits)
        self.last_channels = list(channels)

    def close(self) -> None:
        self.dwf.FDwfDigitalOutReset(self.hdwf)
        self.dwf.FDwfDeviceCloseAll()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pin", type=int, default=0)
    parser.add_argument("--rate", type=float, default=100_000.0)
    parser.add_argument("--frame-rate", type=float, default=50.0)
    parser.add_argument("--uninverted", action="store_true")
    parser.add_argument("--no-ad3", action="store_true", help="UI preview only; do not open or drive AD3")
    parser.add_argument("--seconds", type=float, default=0.0, help="stop after this many seconds; 0 means run until closed")
    return parser.parse_args()


def label(screen, font, text: str, x: int, y: int, color=TEXT) -> None:
    screen.blit(font.render(text, True, color), (x, y))


def centered(screen, font, text: str, center, color=TEXT) -> None:
    surf = font.render(text, True, color)
    screen.blit(surf, surf.get_rect(center=center))


def panel(screen, rect, title: str, font, title_color=TEXT) -> None:
    import pygame

    pygame.draw.rect(screen, PANEL, rect, border_radius=8)
    pygame.draw.rect(screen, LINE, rect, 1, border_radius=8)
    label(screen, font, title, rect.x + 16, rect.y + 12, title_color)


def draw_three_way(screen, font, title, rect, options, active_key) -> None:
    import pygame

    panel(screen, rect, title, font)
    seg_w = (rect.w - 32) // 3
    y = rect.y + 50
    for i, (key, short, detail, color) in enumerate(options):
        r = pygame.Rect(rect.x + 16 + i * seg_w, y, seg_w - 8, 70)
        active = key == active_key
        pygame.draw.rect(screen, color if active else PANEL2, r, border_radius=7)
        pygame.draw.rect(screen, LINE, r, 1, border_radius=7)
        centered(screen, font, short, (r.centerx, r.y + 22), WHITE if active else TEXT)
        centered(screen, font, detail, (r.centerx, r.y + 49), WHITE if active else MUTED)


def three_way_hit(rect, pos):
    import pygame

    if not rect.collidepoint(pos):
        return None
    seg_w = (rect.w - 32) // 3
    y = rect.y + 50
    for i in range(3):
        r = pygame.Rect(rect.x + 16 + i * seg_w, y, seg_w - 8, 70)
        if r.collidepoint(pos):
            return i
    return None


def draw_button(screen, font, title, rect, active, detail, color) -> None:
    import pygame

    pygame.draw.rect(screen, color if active else PANEL2, rect, border_radius=8)
    pygame.draw.rect(screen, LINE, rect, 1, border_radius=8)
    centered(screen, font, title, (rect.centerx, rect.y + 28), WHITE if active else TEXT)
    centered(screen, font, detail, (rect.centerx, rect.y + 56), WHITE if active else MUTED)


def draw_wheel(screen, font, state: RcState, center, radius: int) -> None:
    import pygame

    pygame.draw.circle(screen, (25, 28, 34), center, radius + 18)
    pygame.draw.circle(screen, PANEL2, center, radius)
    pygame.draw.circle(screen, LINE, center, radius, 2)
    pygame.draw.circle(screen, (20, 23, 28), center, 36)
    angle = (state.steer - 1500) / 500 * math.radians(115)
    spoke_len = radius - 22
    for base in (0, 2 * math.pi / 3, 4 * math.pi / 3):
        a = base + angle
        end = (center[0] + int(math.cos(a) * spoke_len), center[1] + int(math.sin(a) * spoke_len))
        pygame.draw.line(screen, ACCENT, center, end, 9)
    centered(screen, font, "CH1 STEER", (center[0], center[1] + radius + 36), TEXT)
    centered(screen, font, f"{state.steer} us", (center[0], center[1] + radius + 62), MUTED)


def steer_from_pos(pos, center) -> int:
    dx = pos[0] - center[0]
    normalized = max(-1.0, min(1.0, dx / 125.0))
    return clamp(1500 + int(normalized * 500))


def pointer_angle(pos, center) -> float:
    return math.atan2(pos[1] - center[1], pos[0] - center[0])


def angle_delta(now_angle: float, start_angle: float) -> float:
    return (now_angle - start_angle + math.pi) % (2 * math.pi) - math.pi


def steer_from_drag(pos, center, start_angle: float, start_value: int) -> int:
    delta = angle_delta(pointer_angle(pos, center), start_angle)
    return clamp(start_value + int(delta / math.radians(115) * 500))


def draw_trigger(screen, font, state: RcState, rect) -> None:
    import pygame

    panel(screen, rect, "Trigger", font)
    rail = pygame.Rect(rect.x + 60, rect.y + 56, 54, rect.h - 110)
    pygame.draw.rect(screen, (26, 30, 36), rail, border_radius=18)
    pygame.draw.rect(screen, LINE, rail, 1, border_radius=18)
    frac = (state.throttle - 1000) / 1000
    y = rail.bottom - int(frac * rail.h)
    knob = pygame.Rect(rail.x - 18, y - 18, rail.w + 36, 36)
    pygame.draw.rect(screen, ACCENT, knob, border_radius=12)
    centered(screen, font, "forward", (rail.centerx, rail.y - 16), MUTED)
    centered(screen, font, "reverse", (rail.centerx, rail.bottom + 20), MUTED)
    centered(screen, font, "CH2 THROTTLE", (rect.centerx, rect.bottom - 36), TEXT)
    centered(screen, font, f"{state.throttle} us", (rect.centerx, rect.bottom - 14), MUTED)


def throttle_from_pos(pos, rect) -> int:
    rail_top = rect.y + 56
    rail_h = rect.h - 110
    frac = 1.0 - ((pos[1] - rail_top) / rail_h)
    return clamp(1000 + int(frac * 1000))


def draw_knob(screen, font, state: RcState, center, radius) -> None:
    import pygame

    pygame.draw.circle(screen, PANEL2, center, radius)
    pygame.draw.circle(screen, LINE, center, radius, 2)
    angle = math.radians(225) + ((state.vr - 1000) / 1000) * math.radians(270)
    end = (center[0] + int(math.cos(angle) * (radius - 13)), center[1] + int(math.sin(angle) * (radius - 13)))
    pygame.draw.line(screen, ACCENT, center, end, 6)
    pygame.draw.circle(screen, (22, 24, 29), center, 12)
    centered(screen, font, "CH3 VR", (center[0], center[1] + radius + 28), TEXT)
    centered(screen, font, "misc servo dial", (center[0], center[1] + radius + 52), MUTED)
    centered(screen, font, f"{state.vr} us", (center[0], center[1] + radius + 76), MUTED)


def vr_from_pos(pos, center) -> int:
    angle = math.atan2(pos[1] - center[1], pos[0] - center[0])
    start = math.radians(225)
    span = math.radians(270)
    value = (angle - start) % (2 * math.pi)
    if value > span:
        value = 0 if value > math.pi else span
    return clamp(1000 + int((value / span) * 1000))


def vr_from_drag(pos, center, start_angle: float, start_value: int) -> int:
    delta = angle_delta(pointer_angle(pos, center), start_angle)
    return clamp(start_value + int(delta / math.radians(270) * 1000))


def draw_channels(screen, font, state: RcState, x: int, y: int) -> None:
    names = [
        ("CH1", "steer", state.steer),
        ("CH2", "throttle", state.throttle),
        ("CH3", "VR misc dial", state.vr),
        ("CH4", "SWA gear pulse", state.swa_gear),
        ("CH5", "SWB mode/kill", {"mavros": 1000, "rc": 1500, "kill": 2000}[state.swb_mode]),
        ("CH6", "SWC misc select", {"misc0": 1000, "none": 1500, "misc1": 2000}[state.swc_misc]),
        ("CH7", "SWD arm button", state.swd_arm),
    ]
    for i, (ch, name, value) in enumerate(names):
        yy = y + i * 28
        label(screen, font, ch, x, yy, ACCENT)
        label(screen, font, name, x + 58, yy, TEXT)
        label(screen, font, f"{value:4d} us", x + 230, yy, MUTED)


def main() -> int:
    args = parse_args()
    try:
        import pygame
    except ImportError as exc:
        raise RuntimeError("pygame is required: python3 -m pip install pygame") from exc

    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("SVEA 7ch RC over AD3 SBUS")
    small_font = TextFont(pygame, 13)
    font = TextFont(pygame, 15)
    title_font = TextFont(pygame, 24)
    clock = pygame.time.Clock()

    state = RcState()
    output = None if args.no_ad3 else SbusOutput(args.pin, args.rate, args.frame_rate, not args.uninverted)

    wheel_center = (230, 285)
    trigger_rect = pygame.Rect(430, 155, 165, 335)
    knob_center = (705, 285)
    swb_rect = pygame.Rect(830, 105, 405, 142)
    swc_rect = pygame.Rect(830, 280, 405, 142)
    gear_rect = pygame.Rect(680, 485, 190, 76)
    arm_rect = pygame.Rect(890, 485, 190, 76)
    channel_rect = pygame.Rect(54, 555, 535, 250)
    help_rect = pygame.Rect(625, 585, 600, 165)

    dragging = None
    drag_start_angle = 0.0
    drag_start_value = 1500
    gear_pressed = False
    arm_pressed = False
    started = time.monotonic()
    running = True

    try:
        while running:
            now = time.monotonic()
            if args.seconds > 0 and now - started >= args.seconds:
                running = False

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                    if math.dist(event.pos, wheel_center) <= 155:
                        dragging = "wheel"
                        drag_start_angle = pointer_angle(event.pos, wheel_center)
                        drag_start_value = state.steer
                    elif trigger_rect.collidepoint(event.pos):
                        dragging = "trigger"
                        state.throttle = throttle_from_pos(event.pos, trigger_rect)
                    elif math.dist(event.pos, knob_center) <= 80:
                        dragging = "knob"
                        drag_start_angle = pointer_angle(event.pos, knob_center)
                        drag_start_value = state.vr
                    elif gear_rect.collidepoint(event.pos):
                        gear_pressed = True
                    elif arm_rect.collidepoint(event.pos):
                        arm_pressed = True
                    else:
                        hit = three_way_hit(swb_rect, event.pos)
                        if hit is not None:
                            state.swb_mode = ["mavros", "rc", "kill"][hit]
                        hit = three_way_hit(swc_rect, event.pos)
                        if hit is not None:
                            state.swc_misc = ["misc0", "none", "misc1"][hit]
                elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                    dragging = None
                    gear_pressed = False
                    arm_pressed = False
                elif event.type == pygame.MOUSEMOTION:
                    if dragging == "wheel":
                        state.steer = steer_from_drag(event.pos, wheel_center, drag_start_angle, drag_start_value)
                    elif dragging == "trigger":
                        state.throttle = throttle_from_pos(event.pos, trigger_rect)
                    elif dragging == "knob":
                        state.vr = vr_from_drag(event.pos, knob_center, drag_start_angle, drag_start_value)
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        state.steer = 1500
                        state.throttle = 1500
                        state.vr = 1500
                    elif event.key == pygame.K_1:
                        state.swb_mode = "mavros"
                    elif event.key == pygame.K_2:
                        state.swb_mode = "rc"
                    elif event.key == pygame.K_3:
                        state.swb_mode = "kill"
                    elif event.key == pygame.K_q:
                        state.swc_misc = "misc0"
                    elif event.key == pygame.K_w:
                        state.swc_misc = "none"
                    elif event.key == pygame.K_e:
                        state.swc_misc = "misc1"

            keys = pygame.key.get_pressed()
            if keys[pygame.K_LEFT]:
                state.steer = clamp(state.steer - 8)
            if keys[pygame.K_RIGHT]:
                state.steer = clamp(state.steer + 8)
            if keys[pygame.K_UP]:
                state.throttle = clamp(state.throttle + 8)
            if keys[pygame.K_DOWN]:
                state.throttle = clamp(state.throttle - 8)
            state.swa_gear = 2000 if gear_pressed or keys[pygame.K_g] else 1000
            state.swd_arm = 2000 if arm_pressed or keys[pygame.K_a] else 1000

            if output is not None:
                output.send(state.channels())

            screen.fill(BG)
            label(screen, title_font, "SVEA RC Controller Emulator", 54, 34)
            status = "preview only" if output is None else f"DIO{args.pin} streaming SBUS"
            label(screen, small_font, f"{status} | inverted={not args.uninverted} | AD3 output is 3.3 V logic", 54, 68, MUTED)

            draw_wheel(screen, font, state, wheel_center, 122)
            draw_trigger(screen, font, state, trigger_rect)
            draw_knob(screen, font, state, knob_center, 66)

            draw_three_way(
                screen,
                font,
                "SWB / CH5 mode and kill",
                swb_rect,
                [
                    ("mavros", "LEFT", "MAVROS", GREEN),
                    ("rc", "MID", "RC ONLY", AMBER),
                    ("kill", "RIGHT", "KILL", RED),
                ],
                state.swb_mode,
            )
            draw_three_way(
                screen,
                font,
                "SWC / CH6 misc-servo selector",
                swc_rect,
                [
                    ("misc0", "LOW", "MISC0", ACCENT),
                    ("none", "MID", "NONE", AMBER),
                    ("misc1", "HIGH", "MISC1", ACCENT),
                ],
                state.swc_misc,
            )
            draw_button(screen, font, "SWA / CH4", gear_rect, state.swa_gear > 1500, "gear momentary", AMBER)
            draw_button(screen, font, "SWD / CH7", arm_rect, state.swd_arm > 1500, "arm momentary", GREEN)

            panel(screen, channel_rect, "Live SBUS channels", font)
            draw_channels(screen, small_font, state, channel_rect.x + 18, channel_rect.y + 48)

            panel(screen, help_rect, "Reference", font)
            label(screen, small_font, "Docs: controlling-the-car.md, pmb3/actuators.md, mikroe rc.board_defaults", help_rect.x + 18, help_rect.y + 44, MUTED)
            label(screen, small_font, "Keys: arrows steer/throttle | 1/2/3 SWB | Q/W/E SWC | G gear | A arm | space center", help_rect.x + 18, help_rect.y + 72, MUTED)
            label(screen, small_font, "PX4: listener input_rc 5; listener manual_control_switches 5", help_rect.x + 18, help_rect.y + 100, MUTED)

            pygame.display.flip()
            clock.tick(60)

    finally:
        if output is not None:
            output.close()
        pygame.quit()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
