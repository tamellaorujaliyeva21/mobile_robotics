#!/usr/bin/env pybricks-micropython
"""
Task 5 - EXPLORE  (runs on the EV3)

Idea: follow ONE EDGE of the tape. Doing that, the robot traces the outline of the
whole drawing: it goes up every spur, wraps around every dead end and comes back.
  Lap 1: tape on the robot's LEFT  -> traces the OUTER outline of the track
  Lap 2: tape on the robot's RIGHT -> traces the INNER outline (inside the loop, D1)
Between the laps the robot crosses the tape. Both laps together drive along both
sides of every line, so every node and every edge gets visited.
The robot logs where it thinks it is (encoders + gyro). task5_plan.py finds the
nodes and edge lengths from that log on the laptop.

How to run (start at S):
 1. Put the robot on the S pad facing EAST (along the short line that leaves the
    pad towards M), with the sensor on the SOUTH edge of that line, just past the
    pad, so the tape is on the robot's LEFT. Put a small paper mark next to the
    line, level with the sensor (not on the tape).
 2. Keep it still and press CENTER (gyro resets, lap 1 starts).
 3. When the sensor comes back past the mark: press any button -> it crosses to the
    north side of the line and starts lap 2.
 4. When the sensor passes the mark again: press any button -> saves explore.csv
"""
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor, GyroSensor
from pybricks.parameters import Port, Button, Direction
from pybricks.tools import wait, StopWatch
from array import array
import math

R = 2.8            # cm, wheel radius (from task 1)
TARGET = 27        # reflection on the edge = (black + white) / 2
SPEED = 120        # deg/s, base wheel speed (slow = better map)
KP = 3             # steering per unit of reflection error (too high = swings across the whole tape)
SLOW_ERR = 14      # error where forward speed drops to 0 (pivot in corners)
REVERSE = 0.4      # max backwards speed (fraction of SPEED) when far off the edge
AVG = 0.1          # how fast the averaged error follows (smaller = smoother)
STUCK_MS = 2000    # no progress for this long = stuck
PUSH_MS = 400      # then push forward for this long
PUSH = 0.4         # push speed (fraction of SPEED)
CROSS_ANGLE = 50   # deg, angle used to cross the tape
CROSS_SIDE = 3.5   # cm, how far sideways to move when crossing (tape width + margin)
LOOP_MS = 10
DRIFT_MS = 10000   # how long to measure gyro drift before starting
PRINT_EVERY = 10    # print every Nth log row live (1 = every row, 0 = off). Printing is slow:
                   # if the robot drives worse while printing, use 5 or 10

ev3 = EV3Brick()
left = Motor(Port.B)
right = Motor(Port.C)
sensor = ColorSensor(Port.S3)
gyro = GyroSensor(Port.S1, Direction.COUNTERCLOCKWISE)   # left turn = +

clock = StopWatch()
# log in compact integer arrays (4 bytes per number instead of a whole Python object)
# distances are stored in mm, written to the CSV in cm
T, A, X, Y, H = array('i'), array('i'), array('i'), array('i'), array('i')
RF, LP, MD = array('b'), array('b'), array('b')
WL, WR = array('i'), array('i')      # raw wheel angles (deg), to cross-check the gyro
x = y = arc = 0.0
prev_l = prev_r = 0
lap = 1
drift = 0.0        # deg/s, measured at the start while standing still


def step(mode):
    """Read sensors, update position, log one row. mode 0 = following, 1 = crossing."""
    global x, y, arc, prev_l, prev_r
    refl = sensor.reflection()
    h = int(round(gyro.angle() - drift * clock.time() / 1000))   # drift removed
    l, r = left.angle(), right.angle()
    ds = math.radians((l - prev_l) + (r - prev_r)) / 2 * R   # distance of robot centre
    prev_l, prev_r = l, r
    x += ds * math.cos(math.radians(h))
    y += ds * math.sin(math.radians(h))
    arc += ds
    T.append(clock.time())
    A.append(int(arc * 10))
    X.append(int(x * 10))
    Y.append(int(y * 10))
    H.append(h)
    RF.append(refl)
    LP.append(lap)
    MD.append(mode)
    WL.append(l)
    WR.append(r)
    if PRINT_EVERY and len(T) % PRINT_EVERY == 0:      # same format as the CSV
        print("%d,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d" % (
            T[-1], A[-1] / 10, X[-1] / 10, Y[-1] / 10, h, refl, lap, mode, l, r))
    return refl


err_avg = 0.0
hist = []          # (time, distance, heading) for the stuck detector
push_until = 0


def follow(tape_left):
    global err_avg, push_until
    refl = step(0)
    err = refl - TARGET                          # + = sensor on white
    # speed follows the AVERAGE error: quick left-right wiggles cancel out (keeps
    # moving), only a corner that stays dark/white for a while slows it down
    err_avg += AVG * (err - err_avg)
    turn = KP * err if tape_left else -KP * err  # + = turn left (towards tape if on white)
    # far off the edge -> go slowly BACKWARDS while turning: lets the sensor
    # get round sharp (hairpin) corners instead of jumping over the tape
    v = SPEED * max(-REVERSE, 1 - abs(err_avg) / SLOW_ERR)
    # stuck detector: hardly moved AND hardly turned for STUCK_MS -> short push forward
    now = clock.time()
    hist.append((now, arc, gyro.angle()))
    while hist and now - hist[0][0] > STUCK_MS:
        old_t, old_arc, old_h = hist.pop(0)
        if abs(arc - old_arc) < 1.0 and abs(gyro.angle() - old_h) < 20 and now > push_until:
            push_until = now + PUSH_MS
            ev3.speaker.beep(800, 50)
    if now < push_until:
        v = PUSH * SPEED
        turn = turn / 2
    left.run(v - turn)
    right.run(v + turn)
    wait(LOOP_MS)


def turn_by(deg):
    target = gyro.angle() + deg
    while abs(target - gyro.angle()) > 2:
        e = target - gyro.angle()
        w = max(60, min(200, 4 * abs(e)))
        if e < 0:
            w = -w
        left.run(-w)
        right.run(w)
        step(1)
        wait(LOOP_MS)
    left.hold()
    right.hold()


def cross(tape_left):
    """Move the robot to the other side of the tape, same heading."""
    a = CROSS_ANGLE if tape_left else -CROSS_ANGLE      # turn towards the tape
    turn_by(a)
    start = arc
    dist = CROSS_SIDE / math.sin(math.radians(CROSS_ANGLE))
    while arc - start < dist:
        left.run(150)
        right.run(150)
        step(1)
        wait(LOOP_MS)
    left.hold()
    right.hold()
    turn_by(-a)
    del hist[:]        # the pause while crossing is not 'stuck'


def button_released():
    while ev3.buttons.pressed():
        wait(10)


ev3.screen.print("TASK 5 EXPLORE")
ev3.screen.print("CENTER = start")
while Button.CENTER not in ev3.buttons.pressed():
    wait(10)
button_released()
# measure the gyro drift: the robot stands still, so any change is fake rotation
ev3.screen.clear()
ev3.screen.print("GYRO CHECK 10 s")
ev3.screen.print("DON'T TOUCH")
wait(1000)
a0 = gyro.angle()
wait(DRIFT_MS)
drift = (gyro.angle() - a0) / (DRIFT_MS / 1000)
print("gyro drift: %.2f deg/s" % drift)
ev3.screen.clear()
ev3.screen.print("drift %.2f deg/s" % drift)
if abs(drift) > 1:
    ev3.screen.print("BAD GYRO: replug")
    ev3.screen.print("it while still")
gyro.reset_angle(0)
left.reset_angle(0)
right.reset_angle(0)
ev3.speaker.beep()
clock.reset()
if PRINT_EVERY:
    print("time_ms,arc_cm,x_cm,y_cm,heading_deg,reflection,lap,mode,left_deg,right_deg")

tape_left = True
error = None
try:
    while True:
        follow(tape_left)
        if ev3.buttons.pressed():
            left.hold()
            right.hold()
            button_released()
            if lap == 1:
                cross(tape_left)
                lap = 2
                tape_left = False
                ev3.speaker.beep()
            else:
                break
except Exception as e:          # whatever goes wrong: stop and still save what we have
    error = e

left.stop()
right.stop()
ev3.screen.clear()
if error is not None:
    print("STOPPED BY ERROR:", repr(error))
    ev3.screen.print("ERROR:")
    ev3.screen.print(repr(error)[:20])
ev3.screen.print("SAVING...")
ev3.screen.print("do not stop!")
n = len(T)
with open("explore.csv", "w") as f:
    f.write("time_ms,arc_cm,x_cm,y_cm,heading_deg,reflection,lap,mode,left_deg,right_deg\n")
    for k0 in range(0, n, 200):          # write in chunks: much faster
        f.write("".join("%d,%.1f,%.1f,%.1f,%d,%d,%d,%d,%d,%d\n" % (
            T[k], A[k] / 10, X[k] / 10, Y[k] / 10, H[k], RF[k], LP[k], MD[k], WL[k], WR[k])
            for k in range(k0, min(k0 + 200, n))))
ev3.screen.clear()
if error is not None:
    ev3.screen.print("ERROR - see output")
ev3.screen.print("saved explore.csv")
ev3.screen.print("{} rows".format(n))
wait(5000)
