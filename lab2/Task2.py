#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor, ColorSensor
from pybricks.parameters import Port, Button
from pybricks.tools import wait, StopWatch

# --- Hardware setup ---
ev3 = EV3Brick()
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)
line_sensor = ColorSensor(Port.S3)

# --- Settings ---
FOLLOW_EDGE = "right"   # "right" or "left". Flip if it steers away from the line.
DEBUG = True            # print values to the VS Code terminal

BASE_SPEED = 200        # deg/s on straights (raise once it is stable)
MIN_SPEED = 90          # deg/s in sharp turns
KP = 120                # reaction to the current error (error is -1..+1)
KD = 5                  # damping, reduces wobbling
DEADBAND = 0.05         # ignore tiny errors (sensor noise)
REVERSE_ALLOWED = 40    # how far the inner wheel may reverse (limits spinning)
SMOOTHING = 0.5         # 0..1, higher = smoother but slower

STEER_DIRECTION = -1 if FOLLOW_EDGE == "right" else 1


# --- Helper functions ---
def wait_for_center():
    while Button.CENTER not in ev3.buttons.pressed():
        wait(20)
    while ev3.buttons.pressed():
        wait(20)


def record_samples(seconds=4):
    ev3.speaker.beep()
    samples = []
    timer = StopWatch()
    while timer.time() < seconds * 1000:
        samples.append(line_sensor.reflection())
        wait(10)
    ev3.speaker.beep()
    samples.sort()
    return samples


def percentile(sorted_samples, p):
    return sorted_samples[int(p * (len(sorted_samples) - 1))]


def stop_motors():
    left_motor.brake()
    right_motor.brake()


# --- Calibration ---
ev3.screen.clear()
ev3.screen.print("LINE: press CENTER,")
ev3.screen.print("drag sensor along")
ev3.screen.print("the line for 4 s")
wait_for_center()
line_samples = record_samples()

ev3.screen.clear()
ev3.screen.print("FLOOR: press CENTER,")
ev3.screen.print("move sensor over")
ev3.screen.print("tiles for 4 s")
wait_for_center()
floor_samples = record_samples()

# Use the LIGHTEST part of the line and the DARKEST part of the floor,
# so light spots on the line never look like floor.
line_level = percentile(line_samples, 0.85)
floor_level = percentile(floor_samples, 0.15)

ev3.screen.clear()
ev3.screen.print("Line: %d-%d" % (line_samples[0], line_samples[-1]))
ev3.screen.print("Floor: %d-%d" % (floor_samples[0], floor_samples[-1]))
ev3.screen.print("Use: L%d F%d" % (line_level, floor_level))
print("Line min/max:", line_samples[0], line_samples[-1], "used:", line_level)
print("Floor min/max:", floor_samples[0], floor_samples[-1], "used:", floor_level)

if floor_level - line_level < 10:
    ev3.screen.print("Contrast too low!")
    ev3.screen.print("Lower the sensor.")
    wait(5000)
    raise SystemExit

threshold = (line_level + floor_level) / 2
half_range = (floor_level - line_level) / 2
print("Threshold:", threshold)

ev3.screen.print("Sensor on line,")
ev3.screen.print("press CENTER")
wait_for_center()

# --- Find the line edge ---
direction = 1 if FOLLOW_EDGE == "right" else -1
left_motor.run(80 * direction)
right_motor.run(-80 * direction)
search_timer = StopWatch()
while search_timer.time() < 4000:
    if line_sensor.reflection() >= threshold:
        break
    wait(5)
stop_motors()
wait(200)

# --- Control state ---
derivative = 0
last_error = 0
d_time = 0
smoothed = line_sensor.reflection()
timer = StopWatch()
timer.reset()
loops = 0

# --- Main loop ---
while True:
    dt = timer.time() / 1000
    timer.reset()
    if dt <= 0:
        dt = 0.005

    # Smoothed reading -> error scaled to about -1 (line) ... +1 (floor)
    smoothed = SMOOTHING * smoothed + (1 - SMOOTHING) * line_sensor.reflection()
    error = (smoothed - threshold) / half_range
    error = max(-1, min(1, error))

    p_error = 0 if abs(error) < DEADBAND else error

    # Derivative over ~30 ms windows (much less noisy than per-loop)
    d_time += dt
    if d_time >= 0.03:
        derivative = 0.5 * derivative + 0.5 * (error - last_error) / d_time
        last_error = error
        d_time = 0

    # Slow down in sharp turns, go fast on straights
    speed = BASE_SPEED - (BASE_SPEED - MIN_SPEED) * abs(error)

    # Limit steering so the robot turns but doesn't spin in place
    limit = speed + REVERSE_ALLOWED
    steer = KP * p_error + KD * derivative
    steer = max(-limit, min(limit, steer)) * STEER_DIRECTION

    left_motor.run(speed + steer)
    right_motor.run(speed - steer)

    loops += 1
    if DEBUG and loops % 40 == 0:
        print(int(smoothed), round(error, 2), int(steer))
    if loops % 20 == 0 and Button.DOWN in ev3.buttons.pressed():
        break

    wait(5)

stop_motors()
ev3.speaker.beep()
