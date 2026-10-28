#!/usr/bin/env python3
"""Cosmic Byte Ares Cartesian joystick bridge for the five-bar Arduino."""
import argparse
import math
import sys
import time

import pygame
import serial

# Adjust these for the gamepad/Arduino on this PC.
DEFAULT_PORT = "COM9"
DEFAULT_BAUD = 115200
AXIS_X, AXIS_Y = 0, 1
INVERT_X, INVERT_Y = False, True  # pygame Y is commonly positive downward
BUTTON_Y, BUTTON_A = 3, 0         # Confirm with --diagnostic if needed
DEADZONE = 0.12
RESPONSE_CURVE = 1.6
MIN_SPEED, MAX_SPEED = 2.0, 63.0  # mm/s
SPEED_CHANGE = 5.0
UPDATE_HZ = 60


def shape_axis(v):
    v = max(-1.0, min(1.0, float(v)))
    magnitude = abs(v)
    if magnitude <= DEADZONE:
        return 0.0
    scaled = (magnitude - DEADZONE) / (1.0 - DEADZONE)
    return math.copysign(scaled ** RESPONSE_CURVE, v)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD)
    parser.add_argument("--diagnostic", action="store_true", help="print raw axes and button events; no motion commands")
    args = parser.parse_args()
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() < 1:
        print("ERROR: No controller detected. Connect the Ares and check its mode/USB or Bluetooth connection.", file=sys.stderr)
        pygame.quit()
        return 2
    pad = pygame.joystick.Joystick(0)
    pad.init()
    print(f"Controller: {pad.get_name()} | left stick moves XY; Y/A adjust speed; Ctrl+C exits")
    ser = None
    speed = 20.0
    last_buttons = set()
    last_send = 0.0
    last_diag = 0.0
    period = 1.0 / UPDATE_HZ
    try:
        if not args.diagnostic:
            ser = serial.Serial(args.port, args.baud, timeout=0, write_timeout=0)
            # Opening an Uno may reset it; allow firmware startup, then clear stale boot text.
            time.sleep(2.0)
            ser.reset_input_buffer()
            ser.write(b"ENABLE\n")
            print(f"Serial: {args.port} at {args.baud} baud. Speed {speed:.0f} mm/s (Y/A adjust).")
        else:
            print("Diagnostic mode: move sticks and press buttons; no serial connection or motion commands.")
        while True:
            started = time.monotonic()
            events = pygame.event.get()
            if any(e.type == pygame.JOYDEVICEREMOVED and e.instance_id == pad.get_instance_id() for e in events):
                raise RuntimeError("Controller disconnected")
            if not pad.get_init():
                raise RuntimeError("Controller disconnected")
            buttons = {i for i in range(pad.get_numbuttons()) if pad.get_button(i)}
            for b in buttons - last_buttons:
                print(f"Button down: {b}")
                if not args.diagnostic:
                    if b == BUTTON_Y:
                        speed = min(MAX_SPEED, speed + SPEED_CHANGE)
                        print(f"Maximum speed: {speed:.0f} mm/s")
                    elif b == BUTTON_A:
                        speed = max(MIN_SPEED, speed - SPEED_CHANGE)
                        print(f"Maximum speed: {speed:.0f} mm/s")
            last_buttons = buttons
            if args.diagnostic and started - last_diag >= 0.1:
                axes = [round(pad.get_axis(i), 3) for i in range(pad.get_numaxes())]
                print(f"axes={axes} buttons={sorted(buttons)}")
                last_diag = started
            elif started - last_send >= period:
                if AXIS_X >= pad.get_numaxes() or AXIS_Y >= pad.get_numaxes():
                    raise RuntimeError("Configured joystick axis index is not available")
                jx, jy = shape_axis(pad.get_axis(AXIS_X)), shape_axis(pad.get_axis(AXIS_Y))
                if INVERT_X: jx = -jx
                if INVERT_Y: jy = -jy
                # Arduino MAX_SPEED is 60 mm/s. Scale normalized joystick
                # commands so the Y/A-selected host speed actually takes effect.
                scale = speed / MAX_SPEED
                ser.write(f"JOY,{jx * scale:.4f},{jy * scale:.4f}\n".encode("ascii"))
                last_send = started
            # Keep the loop responsive without polling at uncontrolled CPU speed.
            remaining = period - (time.monotonic() - started)
            if remaining > 0:
                time.sleep(min(remaining, 0.005))
    except KeyboardInterrupt:
        print("\nStopping controller...")
    except (serial.SerialException, OSError, RuntimeError) as exc:
        print(f"Controller/serial error: {exc}", file=sys.stderr)
    finally:
        if ser is not None and ser.is_open:
            try:
                ser.write(b"STOP\n")
                ser.flush()
                ser.close()
            except serial.SerialException:
                pass
        pygame.quit()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
