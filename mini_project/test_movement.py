
from picarx import Picarx
import time

# Initialize robot
px = Picarx()

# Movement settings
SPEED = 20
MOVE_TIME = 1.0
TURN_ANGLE = 25


# Move forward
def move_forward():
    print("Moving forward")
    px.set_dir_servo_angle(0)
    px.forward(SPEED)
    time.sleep(MOVE_TIME)
    px.stop()


# Move backward
def move_backward():
    print("Moving backward")
    px.set_dir_servo_angle(0)
    px.backward(SPEED)
    time.sleep(MOVE_TIME)
    px.stop()


# Turn left
def turn_left():
    print("Turning left")
    px.set_dir_servo_angle(-TURN_ANGLE)
    px.forward(SPEED)
    time.sleep(MOVE_TIME)
    px.stop()
    px.set_dir_servo_angle(0)


# Turn right
def turn_right():
    print("Turning right")
    px.set_dir_servo_angle(TURN_ANGLE)
    px.forward(SPEED)
    time.sleep(MOVE_TIME)
    px.stop()
    px.set_dir_servo_angle(0)


# Stop motors and straighten wheels
def stop_robot():
    print("Stopping robot")
    px.stop()
    px.set_dir_servo_angle(0)


# Main movement sequence
try:
    print("Starting PiCar-X movement test")

    move_forward()
    time.sleep(1)

    move_backward()
    time.sleep(1)

    turn_left()
    time.sleep(1)

    turn_right()
    time.sleep(1)

    print("Movement test completed")

except KeyboardInterrupt:
    print("Interrupted by user")

finally:
    stop_robot()


