# APEX GD-149 Drone Kit

A Python library for flying the APEX GD-149 with code.

```python
from apexdrone import Drone

with Drone("ble", device_name="APEX_USART_C73303") as d:
    d.takeoff()
    d.forward(dist=50)      # centimetres
    d.rotate_cw(angle=90)   # degrees
    d.land()
```

That is the whole idea. Everything underneath - the radio protocol, the
checksums, the 100 ms heartbeat the drone needs - is handled for you.

---

## What is in this folder

| File | What it is |
|---|---|
| `apexdrone.py` | The library. Import `Drone` from here. Do not edit it. |
| `console.py` | Fly by typing commands, one at a time. Start here. |
| `scan_drones.py` | Run it to find out what your drone is called. |
| `emergency_stop.py` | Run if the motors will not stop. |
| `README.md` | This manual. |
| `gui_step1.py` … `gui_step4.py` | The GUI workshop, in four steps. See below. |
| `drone_worker.py` | Used by the GUI steps. Keeps the window from freezing. |

Write your own programs as new files in this same folder.

---

## Safety - read before flying

1. Fly in an open space. Keep 3 metres clear of people, animals and anything
   breakable.
2. `takeoff()` climbs to about **110 cm**, not the 50 cm the printed manual
   claims. You need at least 2 metres of ceiling indoors.
3. **Ctrl+C is not an emergency stop.** It asks the drone to land, which works
   in normal flight. If the motors will not stop, run `emergency_stop.py` in a
   new terminal window, or unplug the battery. Unplugging always works.
4. `emergency_stop()` cuts the motors instantly. **The drone will fall.** Use it
   only in a real emergency.
5. Start with small distances: `d.forward(dist=20)`. Work up from there.
6. One battery gives about 9-10 minutes of flying and takes 40-60 minutes to
   charge. Spare batteries are worth having.
7. Remove the propellers whenever you are testing on a bench. With the props
   off, `land()` does **not** stop the motors - the drone never actually left
   the ground, so its flight controller thinks it is still taking off. Use
   `emergency_stop()` instead in that situation.

---

## Setting up

### 1. Practice mode - works right now, no drone needed

Create a file called `hello.py` next to `apexdrone.py`:

```python
from apexdrone import Drone

with Drone("sim") as d:
    d.takeoff()
    d.hover(3)
    d.land()
```

Then run it:

```bash
python3 hello.py
```

`"sim"` is practice mode. Your program runs for real and prints every action,
but nothing is sent to a drone. Nothing can break, nothing can fly away. Get
comfortable here first.

### 2. Install the Bluetooth library

Only needed when you want to fly for real over Bluetooth.

```bash
cd path/to/this/folder
python3 -m venv venv
source venv/bin/activate        # Windows: venv\Scripts\activate
pip install bleak
```

**Every time you open a new terminal, activate it again:**

```bash
source venv/bin/activate
```

Forget this and you will see `ModuleNotFoundError: No module named 'bleak'`.

### 3. Allow Bluetooth (macOS only)

Open **System Settings > Privacy & Security > Bluetooth** and switch on the
entry for Terminal (or VS Code, or whichever program you run Python from).
Quit and reopen it afterwards, or the permission will not apply.

If the entry is not listed yet, run `scan_drones.py` once and macOS will ask.

**Never pair the drone in your Bluetooth settings.** It is a Low Energy device
that programs connect to directly. Pairing it makes the operating system hold
on to it and your scripts will fail. If you already paired it, click the `i`
next to its name and choose Forget This Device, then restart the drone.

### 4. Find your drone

```bash
python3 scan_drones.py
```

It scans until it finds a drone, prints the name, and stops. It does not change
anything - it only looks. Write the name down:

```
Found 1 drone:
  APEX_USART_C73303            487D6C84-E369-C9EF-B6B3-B749F34028A1

Copy the name of yours into your program:

    with Drone("ble", device_name="APEX_USART_C73303") as d:
```

### 5. Fly

Change `"sim"` to `"ble"` and give your drone's name:

```python
with Drone("ble", device_name="APEX_USART_C73303") as d:
    d.takeoff()
    d.hover(seconds=3)
    d.land()
```

Naming the drone matters more than it sounds. If two are switched on in the same
room, a program that connects to "the first drone it finds" can end up flying
somebody else's. With a name, that cannot happen: if your drone is not there, the
program says so and stops rather than picking a different one.

If only one drone is ever switched on, you can leave the name out and write
`Drone("ble")`. The moment a second one appears, the program will refuse to guess
and ask you to name one.

---

## Connection modes

| Mode | Setup | Internet | Sensor data | Notes |
|---|---|---|---|---|
| `"sim"` | none | yes | no | Practice mode. Start here. |
| `"ble"` | `pip install bleak` | **works** | **yes** | Recommended for real flying. |
| `"wifi"` | join the drone's hotspot | no | no | Hotspot name starts with `7_`. |

Bluetooth is the better choice: your internet keeps working and the drone
reports its battery, height and heading back to you.

Always name your drone when more than one might be switched on:

```python
with Drone("ble", device_name="APEX_USART_C73303") as d:
    ...
```

---

## Command reference

### Flying

Every command speaks in real-world units: centimetres and degrees.

| Command | What it does |
|---|---|
| `d.takeoff()` | Take off. Climbs to about 110 cm on its own. |
| `d.land()` | Land |
| `d.hover(seconds=3)` | Stay in place for 3 seconds |
| `d.forward(dist=50)` | Fly forward 50 cm |
| `d.back(dist=50)` | Fly backward 50 cm |
| `d.left(dist=50)` / `d.right(dist=50)` | Slide sideways, without turning |
| `d.up(dist=50)` / `d.down(dist=50)` | Climb or descend 50 cm |
| `d.rotate_cw(angle=90)` | Turn right 90 degrees |
| `d.rotate_ccw(angle=90)` | Turn left 90 degrees |

Distances accept 10 to 180 cm, angles 1 to 360 degrees. Ask for something outside
that and the program stops with a message telling you the limits, rather than
quietly doing nothing:

```
ValueError: dist must be between 10 and 180 cm, got 500. Try dist=180.
```

A complete flight is then just a list of instructions:

```python
with Drone("ble") as d:
    d.takeoff()
    for side in range(4):
        d.forward(dist=50)
        d.rotate_cw(angle=90)
    d.land()
```

### Which commands are accurate, and which are not

Not every command is equally trustworthy, and it is worth knowing which is which.

| Command | How it works | How accurate |
|---|---|---|
| `rotate_cw`, `rotate_ccw` | reads the gyro and stops at the right heading | good, a few degrees |
| `up`, `down` | reads the height sensor and stops at the right height | good, a few centimetres |
| `forward`, `back`, `left`, `right` | runs a timer, no sensor involved | rough, can be out by half |

The turning and climbing commands measure the result and correct themselves.
They even allow for the fact that the drone cannot stop dead - it keeps moving
for a moment after the command ends - and they learn how much that is as you fly.

The horizontal moves have nothing to measure with. This drone has no sensor that
reports how far it has travelled, so `forward(dist=50)` can give you 30 cm one
time and 70 cm the next. That is the drone, not your program.

**This difference is the most useful thing in the kit.** Fly a square and watch
the corners come out square while the sides do not. That is closed-loop control
against open-loop control, in one flight, with your own drone.

### Raw control

When you want to fly by feel instead of by numbers:

```python
d.move("forward", seconds=1.5, power=60)
```

`direction` is one of forward, back, left, right, up, down, cw, ccw.
`power` runs from 0 to 100. This is the command that all the others are built on.

### Other commands

| Command | What it does |
|---|---|
| `d.flip()` | Flip. Needs 3 m of space and over 50% battery. |
| `d.go_home()` | Ask the drone to return to its starting point |
| `d.headless()` | Toggle headless mode |
| `d.calibrate()` | Calibrate sensors. Put it on a flat surface first. |
| `d.set_speed(level=2)` | Speed gear: 1 slow, 2 medium, 3 fast |
| `d.emergency_stop()` | Cut the motors. **The drone will fall.** |

### Reading the sensors (Bluetooth only)

```python
info = d.status()
print(info["battery_volt"])     # 4.02
print(info["battery_percent"])  # 80
print(info["altitude_cm"])      # 105
print(info["yaw_deg"])          # 327, in whole degrees, 0-359
print(info["fly_status"])       # "on ground" or "in air"
```

`d.status()` returns `None` if no data has arrived yet, or on the Wi-Fi link.
Always check for `None` before using it.

`d.battery()` is a shortcut for the percentage.

### Recording a flight

```python
with Drone("ble") as d:
    d.start_recording("flight1.csv")   # sensor readings every 100 ms
    d.takeoff()
    d.hover(5)
    d.land()
    d.stop_recording()

d.save_log("flight1_log.csv")          # what your program did, with timestamps
```

Both files land next to `apexdrone.py`, whichever folder you ran from.

### Seeing the actual bytes

```python
with Drone("ble", debug=True) as d:
    d.takeoff()
```

```
frame sent: 24 4d 3c 0a 0f 80 80 00 80 80 40 40 00 00 7a 7f
Taking off
frame sent: 24 4d 3c 0a 0f 80 80 00 80 80 40 40 08 00 7a 77
frame sent: 24 4d 3c 0a 0f 80 80 00 80 80 40 40 00 00 7a 7f
```

Only the changed bytes are interesting: `takeoff()` sets one bit for half a
second and then releases it by itself. There is no magic in the command.

---

## What this drone can and cannot do

These numbers come from measuring a real GD-149, not from the sales sheet.

**Reliable**

| | How good |
|---|---|
| Holding height | stays within about 2 cm while hovering |
| Climbing to a height | `up` and `down` read the height sensor, accurate to a few centimetres |
| Turning by angle | closed loop against the gyro, accurate to a few degrees |
| Height reading | correct in centimetres, once armed |

**Not reliable**

| | Why |
|---|---|
| Flying an exact distance | The same command can travel 17 cm or 50 cm. The drone tries to hold position as soon as you release the stick, and how far it got depends on when that kicks in. `forward` and its friends use a fixed factory figure, so treat their distances as rough. |
| Battery percentage in flight | The voltage sags by up to 0.36 V when the motors work, so the percentage swings wildly. Only trust it when the drone is idle. |
| The drone's optical flow numbers | Despite what the vendor document says, they are not a distance in centimetres. Do not use them to measure travel. |

The practical conclusion: build flight paths out of **turns and height changes**,
which repeat reliably, and treat horizontal distance as a rough estimate.

---

## Troubleshooting

| Problem | What to check |
|---|---|
| `ModuleNotFoundError: apexdrone` | Run your script from this folder, or put your script here |
| `ModuleNotFoundError: bleak` | You forgot `source venv/bin/activate` |
| "Found more than one drone" | Run `scan_drones.py` and save yours, or pass `device_name=` |
| "No drone named ... found" | Drone off, too far, already connected to a phone or remote, or the name is misspelled |
| Nothing at all in the scan | Bluetooth permission not granted, or the terminal not restarted after granting it |
| Connects but no sensor data | Some units do not send it. Control still works; `d.status()` returns `None`. |
| Drifts the same way every time | Put it on a flat surface and call `d.calibrate()` before flying |
| Takes off then drops | Battery low, or a propeller fitted in the wrong position - they are not interchangeable |
| Motors will not stop | Run `emergency_stop.py`, or unplug the battery |

---

## The GUI workshop

Four files that build up to a drone control application with buttons. Each one
runs on its own, and each adds one idea.

| File | What is new | Needs a drone |
|---|---|---|
| `gui_step1.py` | window, widget, main loop | no |
| `gui_step2.py` | buttons, `command=`, `.pack()` and `.grid()` | no |
| `gui_step3.py` | buttons wired to the drone, in practice mode | no |
| `gui_step4.py` | the same app flying for real, plus live sensor readings | yes |

Step 3 is a complete, working application. It is set to practice mode, so the
whole class can build and test it at the same time with no drones at all. Step 4
changes two lines at the top - the link and the drone name - and flies it.

That order is deliberate. It means everyone finishes with a working app even if
the flying part runs out of time, and it separates "is my program right?" from
"is my Bluetooth working?", which are much easier to solve one at a time.

### The one thing that catches everybody

Tkinter runs everything in a single thread. While your function is running, the
window is frozen - it will not redraw and no other button responds:

```python
def on_takeoff():
    drone.takeoff()      # the window is dead for three seconds
```

Students read that freeze as a crash. Worse, the emergency stop button cannot be
clicked while the drone is flying, which is a safety problem rather than a
cosmetic one.

`drone_worker.py` solves it: commands are queued and run on a second thread, and
messages come back through another queue that the window empties on a timer with
`root.after()`. A background thread must never touch a widget directly, so the
queue is not optional - it is what makes it safe.

The emergency stop deliberately skips the queue, so it works even while another
command is still running.

Read `drone_worker.py` once before teaching Step 3. It is short, and the pattern
it uses turns up in every program that has both a window and slow work to do.

---

## How it works underneath

Useful once you are curious, not needed to fly.

Your command never reaches the motors. `power=40` does not mean "run the motors
at 40%" - it means "lean forward about this much". The flight controller decides
what each of the four motors does, and it decides again hundreds of times per
second. That is why the drone can hover steadily while your program does nothing.

What the library actually sends is a small frame, over and over:

```
your code  ->  13 or 14 bytes, repeated every 100 ms
           ->  Bluetooth or Wi-Fi
           ->  a radio module on the drone that just passes bytes through
           ->  the flight controller
           ->  a control loop comparing the wanted angle to the gyro reading
           ->  four motor speeds
```

The frame carries four stick positions (throttle, yaw, pitch, roll), two trim
values, a couple of bitfields for the buttons, and a checksum. Buttons like
takeoff are momentary: the bit is held for about 500 ms and then released.

The 100 ms repetition is a safety feature. If the frames stop arriving, the
drone assumes the link is lost and lands itself instead of flying away. That is
also why this library runs a background thread that keeps sending, even while
your program is doing nothing.

On Bluetooth the drone answers with a 12-byte packet holding the heading,
height, battery voltage and status flags. The library decodes it for you in
`parse_telemetry`, using units measured from a real drone - the vendor
documentation gets several of them wrong.
