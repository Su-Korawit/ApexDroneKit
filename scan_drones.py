"""
scan_drones.py - Find out what your drone is called.

    python3 scan_drones.py

It keeps scanning until it finds a drone, prints the name, and stops. Nothing is
saved and nothing is changed - it only looks.

Copy the name it prints into your own program:

    with Drone("ble", device_name="APEX_USART_C73303") as d:
        ...

Why bother naming it? If two drones are switched on in the same room, a program
that just takes "the first drone it finds" can end up flying somebody else's.
Naming yours makes that impossible.

The drone must not be paired in your computer's Bluetooth settings. It is a Low
Energy device that programs connect to directly; pairing it stops that working.
"""

import asyncio

from bleak import BleakScanner

from apexdrone import clean_name

SCAN_SECONDS = 6.0     # length of one scan
MAX_ROUNDS = 10        # give up after this many scans

# Words that usually appear in the name of a drone in this family.
HINTS = ("APEX", "UART", "USART", "TSPEED", "GD", "149")


def looks_like_a_drone(name: str) -> bool:
    return any(hint in clean_name(name).upper() for hint in HINTS)


async def main():
    print("=" * 62)
    print("Drone finder")
    print("=" * 62)
    print(f"""
Before you press Enter:
  1. Switch the drone on and put it within a metre of the computer.
  2. Close any phone app connected to it, and switch the remote off.
     The drone accepts only one connection at a time.
  3. Do not pair the drone in your Bluetooth settings.

Scanning stops as soon as a drone is found.""")
    input("Press Enter to start: ")

    for round_number in range(1, MAX_ROUNDS + 1):
        print(f"\nScanning... (attempt {round_number} of {MAX_ROUNDS})")
        devices = await BleakScanner.discover(timeout=SCAN_SECONDS)
        named = sorted((d for d in devices if d.name), key=lambda d: d.name.upper())
        found = [d for d in named if looks_like_a_drone(d.name)]

        if found:
            # Advertised names often carry invisible characters, so print the
            # cleaned version - that is what has to be copied into a program.
            names = [clean_name(d.name) for d in found]
            print("\n" + "=" * 62)
            print(f"Found {len(found)} drone{'s' if len(found) > 1 else ''}:")
            print("=" * 62)
            for name, device in zip(names, found):
                print(f"  {name:28} {device.address}")

            print("\nCopy the name of yours into your program:\n")
            print(f'    with Drone("ble", device_name="{names[0]}") as d:')
            print("        d.takeoff()")
            print("        d.land()")

            if len(found) > 1:
                print("\nMore than one drone is switched on. Make sure you pick the")
                print("right name, or you will be flying somebody else's drone.")
            return

        print(f"  No drone yet. Devices seen: "
              f"{[clean_name(d.name) for d in named] if named else 'none at all'}")

    print("\n" + "=" * 62)
    print("Gave up after " + str(MAX_ROUNDS) + " attempts. Things to check:")
    print("=" * 62)
    print("""
  1. Is the drone switched on? Are its lights blinking?
  2. Is a phone or the remote already connected to it? Turn them off.
  3. macOS: System Settings > Privacy & Security > Bluetooth
     must have a tick next to Terminal (or VS Code, or whichever you use).
     Quit and reopen it after switching that on.
  4. Some drones only advertise before they bind to their remote.
  5. If nothing at all was listed above, Bluetooth permission is the most
     likely cause - a working scan sees plenty of devices, drone or not.
  6. If it still never appears, your drone may be Wi-Fi only.
     Use Drone("wifi") instead - everything works except sensor readings.""")


if __name__ == "__main__":
    asyncio.run(main())
