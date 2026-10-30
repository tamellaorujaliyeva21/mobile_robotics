#!/usr/bin/env pybricks-micropython

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import Motor
from pybricks.parameters import Port

# Initialize EV3
ev3 = EV3Brick()

# Initialize motors
left_motor = Motor(Port.B)
right_motor = Motor(Port.C)

# Robot measurements
WHEEL_RADIUS = 2.8       # cm
WHEEL_DISTANCE = 12.0    # cm


def drive_straight(distance):
    """
    Drive forward by the given distance in centimetres.
    """

    motor_degrees = (180 * distance) / (3.14159 * WHEEL_RADIUS)

    left_motor.run_angle(300, motor_degrees, wait=False)
    right_motor.run_angle(300, motor_degrees)


def turn(angle):
    """
    Turn the robot in place by the given angle in degrees.
    """

    motor_degrees = (WHEEL_DISTANCE * angle) / (2 * WHEEL_RADIUS)

    left_motor.run_angle(300, motor_degrees, wait=False)
    right_motor.run_angle(300, -motor_degrees)


# Drive around a 50 cm square

drive_straight(50)
turn(90)

drive_straight(50)
turn(90)

drive_straight(50)
turn(90)

drive_straight(50)
turn(90)
