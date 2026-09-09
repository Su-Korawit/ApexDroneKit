"""
console.py - Fly the drone by typing commands, one at a time.

    python3 console.py

The drone connects once and stays connected. You type a command, watch what
happens, then type another. No editing files, no reconnecting, no waiting for a
Bluetooth scan between every idea.

This is the fastest way to get to know a drone. Once you know what you want it
to do, write the same commands into a program of your own - or add them as a
mission at the bottom of this file.

    [4.02V on ground] >> t
    Taking off
    [3.88V in air 108cm] >> f 50
    Forward 50 cm (1.35 s)
    [3.85V in air 106cm] >> cw 90
    Rotate cw 90 deg (guided by the gyro)
      turned 91 deg (off by +1), heading now 267
    [3.84V in air 105cm] >> l

Ctrl+C lands the drone and leaves. `x` is the emergency stop.
"""

from apexdrone import Drone

LINK = "ble"    # "ble" to fly, or "sim" to practise without a drone
DRONE = "APEX_USART_751F02"      # your drone's name from scan_drones.py, e.g. "APEX_USART_C73303"
                # leave empty only if just one drone is switched on


# ---------------------------------------------------------------------------
# Missions - a saved sequence of commands, run by typing its name
#
# This is where typing commands turns into writing programs. Get a sequence
# working one command at a time, then write it down here and run the whole
# thing with one word. Add as many as you like.
# ---------------------------------------------------------------------------

def square(d):
    """Take off, fly a 50 cm square, land."""
    d.takeoff()
    for side in range(4):
        print(f"  side {side + 1} of 4")
        d.forward(dist=50)
        d.rotate_cw(angle=90)
    d.land()


def staircase(d):
    """Take off, climb in three steps while moving forward, land."""
    d.takeoff()
    for step in range(3):
        print(f"  step {step + 1} of 3")
        d.forward(dist=30)
        d.up(dist=20)
    d.hover(seconds=2)
    d.land()


def spin(d):
    """Take off, turn all the way round in four quarters, land."""
    d.takeoff()
    for _ in range(4):
        d.rotate_cw(angle=90)
        d.hover(seconds=1)
    d.land()


MISSIONS = {
    "m1": square,
    "m2": staircase,
    "m3": spin,
}


# ---------------------------------------------------------------------------
# Commands - each entry is: help text, what it does, whether it takes a number
# Adding a command is one line. The help screen builds itself from this table.
# ---------------------------------------------------------------------------

def build_commands():
    return {
        "t":    ("take off", lambda d, v: d.takeoff(), None),
        "l":    ("land", lambda d, v: d.land(), None),
        "h":    ("hover for N seconds", lambda d, v: d.hover(seconds=v), 3),

        "f":    ("forward N cm", lambda d, v: d.forward(dist=v), 50),
        "b":    ("back N cm", lambda d, v: d.back(dist=v), 50),
        "lt":   ("slide left N cm", lambda d, v: d.left(dist=v), 50),
        "rt":   ("slide right N cm", lambda d, v: d.right(dist=v), 50),
        "u":    ("up N cm", lambda d, v: d.up(dist=v), 30),
        "dn":   ("down N cm", lambda d, v: d.down(dist=v), 30),

        "cw":   ("rotate right N degrees", lambda d, v: d.rotate_cw(angle=v), 90),
        "ccw":  ("rotate left N degrees", lambda d, v: d.rotate_ccw(angle=v), 90),

        "flip": ("flip - needs 3 m of space", lambda d, v: d.flip(), None),
        "home": ("return to the start", lambda d, v: d.go_home(), None),
        "cal":  ("calibrate - put it on a flat surface", lambda d, v: d.calibrate(), None),
        "speed": ("set speed gear 1-3", lambda d, v: d.set_speed(level=int(v)), 2),

        "st":   ("show all sensor readings", lambda d, v: show_status(d), None),
        "x":    ("EMERGENCY STOP - motors off, it will fall",
                 lambda d, v: d.emergency_stop(), None),
    }


def show_status(d):
    info = d.status()
    if info is None:
        print("  No sensor data on this connection.")
        return
    for key in ("battery_volt", "battery_percent", "altitude_cm", "yaw_deg",
                "armed", "fly_status", "optical_flow_x", "optical_flow_y"):
        print(f"  {key:18} {info[key]}")


def show_help(commands):
    print("\n  Commands")
    print("  " + "-" * 52)
    for name, (text, _, default) in commands.items():
        example = f"{name} {default}" if default is not None else name
        print(f"    {example:<10} {text}")
    print("\n  Missions")
    print("  " + "-" * 52)
    for name, func in MISSIONS.items():
        print(f"    {name:<10} {func.__doc__}")
    print("\n  Also")
    print("  " + "-" * 52)
    print("    ?          this help")
    print("    q          land and quit")
    print("    Ctrl+C     land and quit")
    print("\n  A number after a command replaces the default: 'f 80' flies 80 cm.\n")


def prompt(d):
    """The prompt shows the battery and what the drone is doing right now."""
    info = d.status()
    if info is None:
        return ">> "
    height = f" {info['altitude_cm']}cm" if info["altitude_cm"] else ""
    return f"[{info['battery_volt']}V {info['fly_status']}{height}] >> "


def main():
    commands = build_commands()

    print("=" * 60)
    print("Drone console")
    print("=" * 60)
    print("Type ? for the list of commands, q to land and quit.")
    print("Ctrl+C also lands and quits. x is the emergency stop.\n")

    with Drone(LINK, device_name=DRONE or None) as d:
        show_help(commands)

        while True:
            try:
                line = input(prompt(d)).strip()
            except EOFError:
                break

            if not line:
                continue

            word, *rest = line.split()
            word = word.lower()

            if word in ("q", "quit", "exit"):
                break

            if word in ("?", "help"):
                show_help(commands)
                continue

            try:
                if word in MISSIONS:
                    print(f"Mission {word}: {MISSIONS[word].__doc__}")
                    MISSIONS[word](d)
                    continue

                if word not in commands:
                    print(f"  Don't know '{word}'. Type ? for the list.")
                    continue

                _, action, default = commands[word]
                value = default
                if rest:
                    try:
                        value = float(rest[0])
                    except ValueError:
                        print(f"  '{rest[0]}' is not a number.")
                        continue
                action(d, value)

            except ValueError as error:
                # A value outside the range the drone accepts. Say so and carry
                # on, rather than ending the session over a typo.
                print(f"  {error}")
            except KeyboardInterrupt:
                print("\n  Command interrupted. Type q to land and quit, "
                      "or x for an emergency stop.")
            except Exception as error:                      # noqa: BLE001
                print(f"  Something went wrong: {error}")

    print("\nDisconnected.")


if __name__ == "__main__":
    main()
