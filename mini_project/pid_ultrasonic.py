
from picarx import Picarx
import time
import statistics

px = Picarx()

# ========== SETTINGS ==========

TARGET_DISTANCE = 30.0
TOLERANCE = 4.0

# PID settings
KP = 0.7
KI = 0.0
KD = 0.08

MIN_SPEED = 10
MAX_SPEED = 18

# Safety settings
MIN_DISTANCE = 5.0
MAX_DISTANCE = 150.0
MAX_RUNTIME = 60

# Steering remains straight
STEERING_ANGLE = 0

# Motor direction switching
DIRECTION_PAUSE = 0.3

# ========== ULTRASONIC SENSOR ==========

def get_distance():
    readings = []

    for _ in range(5):
        distance = px.get_distance()

        if 2 <= distance <= MAX_DISTANCE:
            readings.append(distance)

        time.sleep(0.05)

    if len(readings) < 4:
        return None

    # Reject inconsistent sensor measurements
    if max(readings) - min(readings) > 10:
        return None

    return statistics.median(readings)


# ========== MOVEMENT FUNCTIONS ==========

def move_forward(speed):
    px.forward(speed)


def move_backward(speed):
    px.backward(speed)


def stop_robot():
    px.stop()


# ========== PID FOLLOWING CONTROLLER ==========

def main():

    integral = 0.0
    previous_error = None
    previous_time = time.monotonic()
    start_time = previous_time

    last_direction = "STOP"

    px.stop()

    # Straight wheels - never steer around box
    px.set_dir_servo_angle(STEERING_ANGLE)

    print("PiCar-X BOX FOLLOWING STARTED")
    print("Target distance:", TARGET_DISTANCE, "cm")

    try:
        while time.monotonic() - start_time < MAX_RUNTIME:

            distance = get_distance()

            now = time.monotonic()
            dt = max(now - previous_time, 0.001)
            previous_time = now

            # Stop on invalid readings
            if distance is None:
                stop_robot()
                integral = 0.0
                previous_error = None
                last_direction = "STOP"

                print("INVALID SENSOR - STOP")
                time.sleep(0.1)
                continue

            # Too close for reliable safe operation
            if distance < MIN_DISTANCE:
                stop_robot()
                print("EMERGENCY STOP:", distance)
                break

            # Positive = box is too far
            # Negative = box is too close
            error = distance - TARGET_DISTANCE

            # Stop near target
            if abs(error) <= TOLERANCE:
                stop_robot()

                integral = 0.0
                previous_error = None
                last_direction = "STOP"

                print(
                    f"Distance: {distance:.1f} cm | "
                    "TARGET REACHED - STOP"
                )

                time.sleep(0.1)
                continue

            # ========== PID ==========

            P = KP * error

            integral += error * dt
            integral = max(-50, min(50, integral))
            I = KI * integral

            if previous_error is None:
                derivative = 0.0
            else:
                derivative = (
                    error - previous_error
                ) / dt

            D = KD * derivative

            pid_output = P + I + D

            # Decide direction from distance error,
            # not from noisy derivative spikes.
            if error > TOLERANCE:
                direction = "FORWARD"
            else:
                direction = "BACKWARD"

            # Motor power
            speed = int(
                max(
                    MIN_SPEED,
                    min(MAX_SPEED, abs(pid_output))
                )
            )

            # Pause before reversing direction
            if (
                last_direction != "STOP"
                and direction != last_direction
            ):
                stop_robot()
                time.sleep(DIRECTION_PAUSE)

                # Recheck distance after the pause
                distance = get_distance()

                if (
                    distance is None
                    or distance < MIN_DISTANCE
                ):
                    stop_robot()
                    last_direction = "STOP"
                    previous_error = None
                    continue

                new_error = distance - TARGET_DISTANCE

                # Abort if direction is no longer needed
                if (
                    abs(new_error) <= TOLERANCE
                    or (new_error > 0) != (error > 0)
                ):
                    stop_robot()
                    last_direction = "STOP"
                    previous_error = None
                    continue

            # ========== ROBOT MOVEMENT ==========

            if direction == "FORWARD":
                move_forward(speed)

            else:
                move_backward(speed)

            print(
                f"Distance: {distance:.1f} cm | "
                f"Error: {error:+.1f} | "
                f"PID: {pid_output:+.1f} | "
                f"{direction} | "
                f"Speed: {speed}"
            )

            previous_error = error
            last_direction = direction

            time.sleep(0.1)

    finally:
        stop_robot()
        px.set_dir_servo_angle(STEERING_ANGLE)
        print("Robot stopped")


try:
    main()

except KeyboardInterrupt:
    print("\nStopped by user")

finally:
    px.stop()
