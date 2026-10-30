#!/usr/bin/env pybricks-micropython
# Lab Task 3 -- controller: P   test: A (step)
# Start with the book 600 mm away; the robot drives to the 200 mm setpoint.

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, UltrasonicSensor
from pybricks.parameters import Port, Button
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch, DataLog

# utime.localtime may not exist in every Pybricks EV3 build,
# so fall back to a placeholder instead of crashing.
try:
    from utime import localtime
except ImportError:
    localtime = None


def now():
    """Current date and time as text (no commas, so the CSV stays intact)."""
    if localtime is None:
        return "no-clock"
    try:
        t = localtime()
        return "{:04d}-{:02d}-{:02d} {:02d}:{:02d}:{:02d}".format(
            t[0], t[1], t[2], t[3], t[4], t[5])
    except Exception:
        return "no-clock"


# ======================= SETTINGS =======================
MODE = "P"
TEST = "A"
RUN_TIME_S = 20       # s per test (centre button stops early)

KP, KI, KD = (2.0, 0.0, 0.0)  # (KP, KI, KD) -- starting values, tune them

SET = 200             # mm, wanted distance
H = 0.05              # s, loop period (0.05, NOT 50)
UMAX = 300            # mm/s, drive clamp
ALPHA = 0.3           # derivative filter
NO_ECHO = 2500        # mm, sensor reads 2550 when it sees nothing
# ========================================================

LOOP_MS = int(H * 1000)

ev3 = EV3Brick()
eyes = UltrasonicSensor(Port.S4)
robot = DriveBase(Motor(Port.B), Motor(Port.C),
                  wheel_diameter=55.5, axle_track=104)
log = DataLog("t_ms", "d_mm", "e_mm", "u_mms", "date",
              name="task3_{}_{}".format(MODE, TEST))


def clamp(u, limit=UMAX):
    return max(-limit, min(limit, u))


# wait for the start, with the book in place
ev3.screen.clear()
ev3.screen.print(MODE + " test " + TEST)
ev3.screen.print("Book at 600 mm")
ev3.screen.print("Centre = start/stop")
while Button.CENTER not in ev3.buttons.pressed():
    wait(20)
while Button.CENTER in ev3.buttons.pressed():
    wait(20)
ev3.speaker.beep()

# first valid reading, so the derivative does not "kick"
d = eyes.distance()
while d > NO_ECHO:
    wait(LOOP_MS)
    d = eyes.distance()
e_prev = d - SET

integral = 0.0
deriv = 0.0
clock = StopWatch()

while clock.time() < RUN_TIME_S * 1000:
    if Button.CENTER in ev3.buttons.pressed():
        break

    d = eyes.distance()                       # 1 read
    if d > NO_ECHO:                           # saw nothing: hold still
        robot.drive(0, 0)
        wait(LOOP_MS)
        continue

    e = d - SET                               # +ve = too far -> go forward
    deriv = ALPHA * (e - e_prev) / H + (1 - ALPHA) * deriv
    u = KP * e + KI * integral + KD * deriv   # 2 compute

    if abs(u) < UMAX:                         # anti-windup
        integral += e * H

    u = clamp(u)
    robot.drive(u, 0)                         # 3 write

    log.log(clock.time(), d, e, u, now())     # 5 values = 5 columns
    print(d, u)
    e_prev = e
    wait(LOOP_MS)                             # 4 wait

robot.stop()
ev3.speaker.beep()
