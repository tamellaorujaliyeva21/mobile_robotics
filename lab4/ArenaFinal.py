#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Port
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
from urandom import randint

# ------------------------------------------------------------------
# Settings (tune these on the real arena)
# ------------------------------------------------------------------
RUN_MINUTES = 5             # how long to run; set to 0 to run until you press a button
HEAT_TIME_MS = RUN_MINUTES * 60 * 1000

# Color sensor readings (0-100). Measure them with sensor_test.py and edit!
WHITE_VALUE = 60            # reflection over the white floor
BLACK_VALUE = 8             # reflection over the black border
THRESHOLD = (WHITE_VALUE + BLACK_VALUE) // 2   # below this = border (34)
CRUISE_SPEED = 150          # mm/s  normal speed
SLOW_SPEED = 90             # mm/s  speed when something is ahead
SLOW_DIST = 450             # mm    start slowing down
AVOID_DIST = 250            # mm    turn away (robot too close)
BACKUP_DIST = 60            # mm    reverse after touching the border
WANDER_EVERY_MS = 4000      # small random steering so we don't trace the same path

WHEEL_DIAMETER = 56         # mm  (standard EV3 wheel)
AXLE_TRACK = 114            # mm  (distance between the wheels)

# ------------------------------------------------------------------
# Setup
# ------------------------------------------------------------------
ev3 = EV3Brick()
left = Motor(Port.B)
right = Motor(Port.C)
color = ColorSensor(Port.S3)
sonar = UltrasonicSensor(Port.S4)

robot = DriveBase(left, right, WHEEL_DIAMETER, AXLE_TRACK)

ev3.speaker.beep()
wait(3000)   # time to place the robot in the corner

# Random start delay (0-1.5 s) so four identical programs don't move in sync
wait(randint(0, 1500))

timer = StopWatch()
last_wander = 0
robot.drive(CRUISE_SPEED, 0)


def turn_away(min_angle, max_angle):
    """Turn a random amount, in a random direction (breaks head-on deadlocks)."""
    angle = randint(min_angle, max_angle)
    if randint(0, 1) == 0:
        angle = -angle
    robot.turn(angle)


# ------------------------------------------------------------------
# Main loop
# ------------------------------------------------------------------
while RUN_MINUTES == 0 or timer.time() < HEAT_TIME_MS:
    # Any EV3 button press stops the program (useful when RUN_MINUTES = 0)
    if ev3.buttons.pressed():
        break

    # 1) Border check (highest priority: never leave the arena)
    if color.reflection() < THRESHOLD:
        robot.stop()
        robot.straight(-BACKUP_DIST)
        turn_away(110, 170)          # turn back toward the middle
        robot.drive(CRUISE_SPEED, 0)
        continue

    # 2) Other robot check
    dist = sonar.distance()

    if dist < AVOID_DIST:
        # Too close: steer away without stopping the heat
        robot.stop()
        turn_away(70, 140)
        robot.drive(CRUISE_SPEED, 0)
    elif dist < SLOW_DIST:
        # Something ahead: slow down and curve slightly to give it room
        robot.drive(SLOW_SPEED, 25 if randint(0, 1) else -25)
    else:
        # Clear path: cruise, with an occasional random wiggle
        now = timer.time()
        if now - last_wander > WANDER_EVERY_MS:
            last_wander = now
            robot.drive(CRUISE_SPEED, randint(-30, 30))
        # otherwise keep the current drive command

    wait(20)

# ------------------------------------------------------------------
# Time is up
# ------------------------------------------------------------------
robot.stop()
ev3.speaker.beep()
