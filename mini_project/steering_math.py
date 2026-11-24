
from picarx import Picarx
import math
import time
import csv
from datetime import datetime

px = Picarx()

# ==================================
# ROBOT CONFIGURATION
# ==================================

# Measure from front axle to rear axle
WHEELBASE_M = 0.095  # Example only: replace with your measurement

# Whether the robot physically drives
DRIVE_ENABLED = True

MOTOR_POWER = 15

# This is an ASSUMED velocity, not a sensor reading.
# Calibrate it experimentally before using pose results.
ASSUMED_SPEED_MPS = 0.12

SAMPLE_INTERVAL = 0.2
STEP_DURATION = 1.5

# Negative commands turn left on the usual PiCar-X setup
STEERING_SEQUENCE = [
    ("STRAIGHT", 0),
    ("LEFT_15", -15),
    ("LEFT_25", -25),
    ("STRAIGHT", 0),
    ("RIGHT_15", 15),
    ("RIGHT_25", 25),
    ("STRAIGHT", 0)
]

CSV_FILE = "steering_math_log_drive.csv"

# ==================================
# MATHEMATICAL FUNCTIONS
# ==================================

def steering_geometry(command_deg):
    # Convert PiCar-X command convention into
    # mathematical convention: positive = left
    delta_deg = -command_deg
    delta_rad = math.radians(delta_deg)

    curvature = math.tan(delta_rad) / WHEELBASE_M

    if abs(curvature) < 1e-9:
        radius = None
    else:
        radius = 1.0 / curvature

    return delta_deg, curvature, radius


def calculate_icc(x, y, heading, radius):
    if radius is None:
        return None, None

    icc_x = x - radius * math.sin(heading)
    icc_y = y + radius * math.cos(heading)

    return icc_x, icc_y


def update_pose(x, y, heading, v, curvature, dt):
    # Kinematic bicycle model with constant
    # speed and curvature over one interval

    omega = v * curvature
    new_heading = heading + omega * dt

    if abs(omega) < 1e-9:
        new_x = x + v * dt * math.cos(heading)
        new_y = y + v * dt * math.sin(heading)
    else:
        radius = v / omega

        new_x = x + radius * (
            math.sin(new_heading) - math.sin(heading)
        )

        new_y = y - radius * (
            math.cos(new_heading) - math.cos(heading)
        )

    return new_x, new_y, new_heading, omega


# ==================================
# MAIN LOGGING PROGRAM
# ==================================

def main():
    if WHEELBASE_M <= 0:
        raise ValueError("Wheelbase must be positive")

    x = 0.0
    y = 0.0
    heading = 0.0

    total_distance = 0.0

    fields = [
        "timestamp",
        "elapsed_s",
        "test_name",
        "command_angle_deg",
        "model_steering_deg",
        "wheelbase_m",
        "motor_power",
        "assumed_speed_mps",
        "turn_radius_m",
        "curvature_1_m",
        "angular_velocity_rad_s",
        "angular_velocity_deg_s",
        "delta_heading_deg",
        "heading_deg",
        "x_m",
        "y_m",
        "icc_x_m",
        "icc_y_m",
        "estimated_distance_m"
    ]

    px.stop()
    px.set_dir_servo_angle(0)

    print("PiCar-X Steering Math Logger")
    print("Drive enabled:", DRIVE_ENABLED)
    print("Wheelbase:", WHEELBASE_M, "m")
    print("Logging to:", CSV_FILE)

    with open(CSV_FILE, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()

        program_start = time.monotonic()

        for test_name, command in STEERING_SEQUENCE:

            px.stop()
            px.set_dir_servo_angle(command)

            # Give the steering servo time to settle
            time.sleep(0.5)

            delta_deg, curvature, radius = (
                steering_geometry(command)
            )

            if DRIVE_ENABLED:
                px.forward(MOTOR_POWER)
                v = ASSUMED_SPEED_MPS
            else:
                v = 0.0

            print(
                f"\nTEST: {test_name} | "
                f"Command: {command:+} degrees"
            )

            step_start = time.monotonic()
            previous_time = step_start

            while time.monotonic() - step_start < STEP_DURATION:

                now = time.monotonic()
                dt = now - previous_time
                previous_time = now

                # Estimate current ICC before movement
                icc_x, icc_y = calculate_icc(
                    x, y, heading, radius
                )

                old_heading = heading

                x, y, heading, omega = update_pose(
                    x, y, heading, v, curvature, dt
                )

                delta_heading = heading - old_heading
                total_distance += abs(v * dt)

                row = {
                    "timestamp": datetime.now().isoformat(
                        timespec="milliseconds"
                    ),
                    "elapsed_s": round(
                        now - program_start, 3
                    ),
                    "test_name": test_name,
                    "command_angle_deg": command,
                    "model_steering_deg": delta_deg,
                    "wheelbase_m": WHEELBASE_M,
                    "motor_power": (
                        MOTOR_POWER if DRIVE_ENABLED else 0
                    ),
                    "assumed_speed_mps": v,
                    "turn_radius_m": (
                        round(radius, 4)
                        if radius is not None else ""
                    ),
                    "curvature_1_m": round(curvature, 5),
                    "angular_velocity_rad_s": round(
                        omega, 5
                    ),
                    "angular_velocity_deg_s": round(
                        math.degrees(omega), 3
                    ),
                    "delta_heading_deg": round(
                        math.degrees(delta_heading), 4
                    ),
                    "heading_deg": round(
                        math.degrees(heading), 3
                    ),
                    "x_m": round(x, 4),
                    "y_m": round(y, 4),
                    "icc_x_m": (
                        round(icc_x, 4)
                        if icc_x is not None else ""
                    ),
                    "icc_y_m": (
                        round(icc_y, 4)
                        if icc_y is not None else ""
                    ),
                    "estimated_distance_m": round(
                        total_distance, 4
                    )
                }

                writer.writerow(row)
                f.flush()

                radius_str = (
                    "infinity"
                    if radius is None
                    else f"{radius:.3f} m"
                )

                print(
                    f"Steer: {delta_deg:+.1f} deg | "
                    f"R: {radius_str} | "
                    f"Curvature: {curvature:+.3f} | "
                    f"Yaw rate: {math.degrees(omega):+.2f} deg/s | "
                    f"Heading: {math.degrees(heading):+.2f} deg | "
                    f"X: {x:.3f} m | Y: {y:.3f} m"
                )

                time.sleep(SAMPLE_INTERVAL)

            px.stop()
            time.sleep(0.3)

    print("\nLogging complete:", CSV_FILE)


try:
    main()

except KeyboardInterrupt:
    print("\nInterrupted by user")

finally:
    px.stop()
    px.set_dir_servo_angle(0)
    print("Motors stopped, steering centered")

