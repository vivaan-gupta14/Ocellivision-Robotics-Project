#!/usr/bin/env python
"""Stage 1: PS4 controller connection + raw input detection.

No IK, no joint targets, no robot output yet, and no handbrake/e-stop state
machine yet either -- this is purely proving the controller is being read
correctly (connection, buttons, sticks, triggers, d-pad) before anything else
gets built on top of it. The handbrake will come back once we can see it
actually toggling correctly here; it's safety-critical and shouldn't be wired
into anything that drives real motors until it demonstrably works.

Runs as a pygame window (black background, white text) rather than printing to
the terminal -- pygame's joystick/hotplug event handling is more reliable with
an actual display/event loop driving it, which a bare terminal script doesn't
give it.

Behavior:
    - Polls for a connected controller; if none is found, the window prompts to
      connect one over Bluetooth and keeps polling.
    - Once connected, continuously shows every currently-active button/axis
      alongside its planned robot-control meaning, or "Waiting for input..."
      when idle.
    - A RAW DEBUG panel is always shown too, listing every currently-pressed
      button index and every axis away from its rest value, unfiltered by the
      guessed mapping below -- this is what you use to find/fix any wrong
      index (see the comment above BUTTON_MAP).

Requires: uv sync   (pygame is declared in pyproject.toml)
Run:      uv run controller_input.py
"""

import sys
import time

import pygame

# --------------------------------------------------------------------------------------
# Raw button/axis index guesses -- Linux + Bluetooth + SDL2 DualShock 4, FIRST GUESS ONLY.
# The RAW DEBUG panel always shows the true index for whatever you press, so if any of
# these are wrong for your kernel driver (hid-sony vs hid-playstation), it'll be obvious
# immediately -- just fix the number here.
# --------------------------------------------------------------------------------------

BUTTON_MAP = {
    # index: (label, what it does once the real pipeline exists)
    0: ("Cross (X)", "Remote center of motion (RCM) config [not yet implemented]"),
    1: ("Circle", "Right jaw: open"),
    2: ("Triangle", "Return to home position [not yet implemented]"),
    3: ("Square", "Right jaw: close"),
    4: ("L1", "Roll left"),
    5: ("R1", "Roll right"),
    8: ("Share", "(unmapped)"),
    9: ("Options", "Display current keybinds"),
    10: ("L3 (stick click)", "(unmapped)"),
    11: ("R3 (stick click)", "(unmapped)"),
    12: ("PS button", "(unmapped)"),
    13: ("Touchpad", "(unmapped)"),
}

# axis index: (positive-direction label, negative-direction label)
# SDL/pygame stick axes: pushing the stick UP reports a NEGATIVE Y value.
STICK_AXIS_MAP = {
    0: ("Left stick right -> Yaw right", "Left stick left -> Yaw left"),
    1: ("Left stick down -> Pitch down", "Left stick up -> Pitch up"),
    3: ("Right stick right -> +X", "Right stick left -> -X"),
    4: ("Right stick down -> -Y", "Right stick up -> +Y"),
}
TRIGGER_AXIS_MAP = {
    2: "L2 -> -Z",
    5: "R2 -> +Z",
}

# Opposite-direction control pairs that can physically be held at once. Resolved by
# "first press wins": whichever side's press edge lands first keeps exclusive priority
# until it's released, even while the other side is also held. D-pad left/right (left
# jaw) is deliberately left out of this -- physically hard to press both at once, so not
# worth the complexity yet.
SOCD_BUTTON_PAIRS = {
    "roll": (4, 5),       # L1, R1
    "jaw_right": (1, 3),  # Circle, Square
}
SOCD_TRIGGER_PAIR = ("z", 2, 5)  # name, L2 axis, R2 axis
SOCD_BUTTON_INDICES = {i for pair in SOCD_BUTTON_PAIRS.values() for i in pair}

STICK_DEADZONE = 0.2
AXIS_REST_DEADZONE = 0.15  # deviation from each axis's sampled rest value, used for debug too
AXIS_REST_SAMPLES = 10
POLL_HZ = 30
RECONNECT_POLL_S = 0.5

WINDOW_SIZE = (960, 760)
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GRAY = (160, 160, 160)


def sample_axis_rest(joystick):
    """Sample every axis a few times at connect and average, so both the mapped display
    and the raw debug panel judge 'active' by deviation from actual rest rather than an
    assumed idle value -- idle position varies by driver (some report -1.0 at rest for
    triggers, others 0.0)."""
    num_axes = joystick.get_numaxes()
    rest = {i: 0.0 for i in range(num_axes)}
    for _ in range(AXIS_REST_SAMPLES):
        pygame.event.pump()
        for i in range(num_axes):
            rest[i] += joystick.get_axis(i)
        time.sleep(0.01)
    return {i: v / AXIS_REST_SAMPLES for i, v in rest.items()}


def resolve_first_press(a_down, b_down, held):
    """SOCD resolution for one conflicting pair: whichever side (a/b) was pressed first
    keeps exclusive priority until it releases, even if the other side is also held.
    `held` is the persisted winner from the previous frame ('a', 'b', or None)."""
    if held == "a" and not a_down:
        held = None
    if held == "b" and not b_down:
        held = None
    if held is None:
        if a_down and not b_down:
            held = "a"
        elif b_down and not a_down:
            held = "b"
        elif a_down and b_down:
            held = "a"  # both edges landed on the same frame -- arbitrary stable default
    return held


def collect_mapped_active_inputs(joystick, axis_rest, socd_state):
    active = []
    num_buttons = joystick.get_numbuttons()
    num_axes = joystick.get_numaxes()
    num_hats = joystick.get_numhats()

    for idx, (label, meaning) in BUTTON_MAP.items():
        if idx in SOCD_BUTTON_INDICES:
            continue
        if idx < num_buttons and joystick.get_button(idx):
            active.append(f"[btn {idx:2d}] {label:20s} -> {meaning}")

    for idx, (pos_label, neg_label) in STICK_AXIS_MAP.items():
        if idx >= num_axes:
            continue
        v = joystick.get_axis(idx)
        if v > STICK_DEADZONE:
            active.append(f"[axis {idx}] {pos_label}  ({v:+.2f})")
        elif v < -STICK_DEADZONE:
            active.append(f"[axis {idx}] {neg_label}  ({v:+.2f})")

    for name, (idx_a, idx_b) in SOCD_BUTTON_PAIRS.items():
        a_down = idx_a < num_buttons and bool(joystick.get_button(idx_a))
        b_down = idx_b < num_buttons and bool(joystick.get_button(idx_b))
        socd_state[name] = resolve_first_press(a_down, b_down, socd_state.get(name))
        winner = socd_state[name]
        if winner == "a":
            label, meaning = BUTTON_MAP[idx_a]
            suffix = "  (other held too, suppressed)" if b_down else ""
            active.append(f"[btn {idx_a:2d}] {label:20s} -> {meaning}{suffix}")
        elif winner == "b":
            label, meaning = BUTTON_MAP[idx_b]
            suffix = "  (other held too, suppressed)" if a_down else ""
            active.append(f"[btn {idx_b:2d}] {label:20s} -> {meaning}{suffix}")

    z_name, l2_idx, r2_idx = SOCD_TRIGGER_PAIR
    if l2_idx < num_axes and r2_idx < num_axes:
        l2_v = joystick.get_axis(l2_idx)
        r2_v = joystick.get_axis(r2_idx)
        l2_down = abs(l2_v - axis_rest.get(l2_idx, 0.0)) > AXIS_REST_DEADZONE
        r2_down = abs(r2_v - axis_rest.get(r2_idx, 0.0)) > AXIS_REST_DEADZONE
        socd_state[z_name] = resolve_first_press(l2_down, r2_down, socd_state.get(z_name))
        winner = socd_state[z_name]
        if winner == "a":
            suffix = "  (other held too, suppressed)" if r2_down else ""
            active.append(
                f"[axis {l2_idx}] {TRIGGER_AXIS_MAP[l2_idx]}  ({l2_v:+.2f}, rest {axis_rest[l2_idx]:+.2f}){suffix}"
            )
        elif winner == "b":
            suffix = "  (other held too, suppressed)" if l2_down else ""
            active.append(
                f"[axis {r2_idx}] {TRIGGER_AXIS_MAP[r2_idx]}  ({r2_v:+.2f}, rest {axis_rest[r2_idx]:+.2f}){suffix}"
            )

    if num_hats > 0:
        hat = joystick.get_hat(0)
        hat_map = {
            (0, 1): "D-pad up -> Prismatic joint: forward",
            (0, -1): "D-pad down -> Prismatic joint: backward",
            (-1, 0): "D-pad left -> Left jaw: open",
            (1, 0): "D-pad right -> Left jaw: close",
        }
        if hat in hat_map:
            active.append(f"[hat 0]  {hat_map[hat]}  {hat}")

    return active


def collect_raw_debug(joystick, axis_rest):
    """Every currently-pressed button index and every axis away from rest, unfiltered by
    BUTTON_MAP/STICK_AXIS_MAP -- this is what's used to discover/fix the real hardware
    mapping."""
    pressed_buttons = [i for i in range(joystick.get_numbuttons()) if joystick.get_button(i)]
    active_axes = []
    for i in range(joystick.get_numaxes()):
        v = joystick.get_axis(i)
        rest = axis_rest.get(i, 0.0)
        if abs(v - rest) > AXIS_REST_DEADZONE:
            active_axes.append(f"axis {i}: {v:+.2f} (rest {rest:+.2f})")
    hat = joystick.get_hat(0) if joystick.get_numhats() > 0 else None
    return pressed_buttons, active_axes, hat


def draw_text_block(screen, font, lines, x, y, color=WHITE, line_height=26):
    for line in lines:
        surf = font.render(line, True, color)
        screen.blit(surf, (x, y))
        y += line_height
    return y


def draw_waiting_screen(screen, font, big_font):
    screen.fill(BLACK)
    y = draw_text_block(
        screen, big_font, ["PS4 CONTROLLER TELEOP -- STAGE 1"], 30, 30, WHITE, 40
    )
    y += 20
    draw_text_block(
        screen,
        font,
        [
            "No controller detected.",
            "",
            "-> Connect your PS4 controller via Bluetooth (hold the PS",
            "   button until the light bar goes solid), then wait...",
        ],
        30,
        y,
        GRAY,
    )
    pygame.display.flip()


def draw_main_screen(screen, font, big_font, name, mapped_active, raw_pressed, raw_axes, raw_hat):
    screen.fill(BLACK)
    y = draw_text_block(screen, big_font, ["PS4 CONTROLLER TELEOP -- STAGE 1"], 30, 30, WHITE, 40)
    y += 10
    y = draw_text_block(screen, font, [f"Connected: {name}"], 30, y, GRAY)
    y += 20

    if mapped_active:
        y = draw_text_block(screen, font, ["ACTIVE INPUT:"], 30, y, WHITE)
        y = draw_text_block(screen, font, [f"  {line}" for line in mapped_active], 30, y, WHITE)
    else:
        y = draw_text_block(screen, font, ["Waiting for input..."], 30, y, GRAY)

    y += 30
    y = draw_text_block(screen, font, ["-" * 70], 30, y, GRAY)
    y = draw_text_block(screen, font, ["RAW DEBUG (use this to fix indices above):"], 30, y, GRAY)
    y = draw_text_block(screen, font, [f"  buttons pressed: {raw_pressed if raw_pressed else '(none)'}"], 30, y, GRAY)
    if raw_axes:
        y = draw_text_block(screen, font, ["  active axes:"], 30, y, GRAY)
        y = draw_text_block(screen, font, [f"    {a}" for a in raw_axes], 30, y, GRAY)
    else:
        y = draw_text_block(screen, font, ["  active axes: (none)"], 30, y, GRAY)
    if raw_hat is not None:
        y = draw_text_block(screen, font, [f"  hat 0: {raw_hat}"], 30, y, GRAY)

    y += 30
    draw_text_block(screen, font, ["Close the window (or Ctrl+C in the terminal) to quit."], 30, y, GRAY)

    pygame.display.flip()


def main():
    pygame.init()
    pygame.joystick.init()
    screen = pygame.display.set_mode(WINDOW_SIZE)
    pygame.display.set_caption("PS4 Controller Teleop -- Stage 1")
    font = pygame.font.SysFont("couriernew,monospace", 20)
    big_font = pygame.font.SysFont("couriernew,monospace", 28, bold=True)
    clock = pygame.time.Clock()

    joystick = None
    axis_rest = {}
    socd_state = {"roll": None, "jaw_right": None, "z": None}

    running = True
    try:
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
            if not running:
                break

            if joystick is None and pygame.joystick.get_count() > 0:
                joystick = pygame.joystick.Joystick(0)
                joystick.init()
                axis_rest = sample_axis_rest(joystick)
                socd_state = {"roll": None, "jaw_right": None, "z": None}
                print(f"Controller connected: {joystick.get_name()}")
            elif joystick is not None and pygame.joystick.get_count() == 0:
                print("Controller disconnected.")
                joystick = None

            if joystick is None:
                draw_waiting_screen(screen, font, big_font)
                clock.tick(POLL_HZ)
                continue

            mapped_active = collect_mapped_active_inputs(joystick, axis_rest, socd_state)
            raw_pressed, raw_axes, raw_hat = collect_raw_debug(joystick, axis_rest)

            draw_main_screen(screen, font, big_font, joystick.get_name(), mapped_active, raw_pressed, raw_axes, raw_hat)
            clock.tick(POLL_HZ)
    except KeyboardInterrupt:
        print("Exiting.")
    finally:
        pygame.quit()


if __name__ == "__main__":
    sys.exit(main())
