
from flask import Flask, Response
from picamera2 import Picamera2
from picarx import Picarx

import cv2
import time
import numpy as np
import csv
import os
import threading
import statistics

from datetime import datetime

from ball_detection import (
    WIDTH,
    HEIGHT,
    find_ball_candidates,
    select_ball,
    CONFIRM_FRAMES,
    MAX_TRACK_JUMP
)

# ==========================================
# 1. SETTINGS
# ==========================================

TARGET_DISTANCE = 40.0  # Desired testing distance in cm

CSV_FILE = os.path.expanduser(
    "~/ball_calibration.csv"
)

IMAGE_FOLDER = os.path.expanduser(
    "~/ball_calibration_images"
)

os.makedirs(IMAGE_FOLDER, exist_ok=True)


# ==========================================
# 2. INITIALIZE HARDWARE
# ==========================================

app = Flask(__name__)

px = Picarx()

camera = Picamera2()

camera.configure(
    camera.create_video_configuration(
        main={
            "size": (WIDTH, HEIGHT),
            "format": "RGB888"
        }
    )
)

camera.start()
time.sleep(2)


# ==========================================
# 3. SHARED DATA
# ==========================================

lock = threading.Lock()

reset_tracking = threading.Event()
tracking_reset_done = threading.Event()

latest_jpeg = None

latest_result = {
    "candidate": False,
    "confirmed": False,
    "x": None,
    "y": None,
    "radius": None,
    "coverage": None,
    "distance": None
}

running = True


# ==========================================
# 4. ULTRASONIC SENSOR
# ==========================================

def get_distance():

    readings = []

    for _ in range(3):

        try:

            distance = px.get_distance()

            if distance is not None and 2 <= distance <= 200:
                readings.append(float(distance))

        except Exception:
            pass

        time.sleep(0.03)

    if len(readings) >= 2:
        return statistics.median(readings)

    return None


# ==========================================
# BALL CANDIDATE CALIBRATION FILTER
# ==========================================

MIN_BALL_RADIUS = 25
MAX_BALL_RADIUS = 180

MIN_MEAN_SATURATION = 90

def filter_ball_candidates(frame, candidates):

    height, width = frame.shape[:2]

    # Convert image to HSV
    hsv = cv2.cvtColor(
        frame,
        cv2.COLOR_BGR2HSV
    )

    saturation = hsv[:, :, 1]

    valid_candidates = []

    for candidate in candidates:

        x, y, radius, coverage = candidate

        x = int(x)
        y = int(y)
        radius = int(radius)

        # FILTER 1: Object size
        if radius < MIN_BALL_RADIUS:
            continue

        if radius > MAX_BALL_RADIUS:
            continue

        # FILTER 2: Reject incomplete circles
        if (
            x - radius < 3 or
            y - radius < 3 or
            x + radius >= width - 3 or
            y + radius >= height - 3
        ):
            continue

        # FILTER 3: Examine the inner part
        # of the detected circular object.

        mask = np.zeros(
            (height, width),
            dtype=np.uint8
        )

        inner_radius = max(
            3,
            int(radius * 0.60)
        )

        cv2.circle(
            mask,
            (x, y),
            inner_radius,
            255,
            -1
        )

        mean_saturation = cv2.mean(
            saturation,
            mask=mask
        )[0]

        # Reject weakly saturated objects
        if mean_saturation < MIN_MEAN_SATURATION:
            continue

        valid_candidates.append(candidate)

    return valid_candidates

# ==========================================
# 5. CAMERA AND DETECTION THREAD
# ==========================================

def camera_worker():

    global latest_jpeg

    previous_center = None
    stable_frames = 0
    missing_frames = 0

    distance = None
    last_sensor_time = 0

    while running:

        # Reset tracking when a new experiment starts
        if reset_tracking.is_set():

            previous_center = None
            stable_frames = 0
            missing_frames = 0

            reset_tracking.clear()
            tracking_reset_done.set()

        # Capture camera image
        frame = camera.capture_array()

        # Read ultrasonic sensor periodically
        now = time.monotonic()

        if now - last_sensor_time >= 0.3:

            distance = get_distance()
            last_sensor_time = now

        # Find circular candidates
        # Find all circular candidates
        candidates = find_ball_candidates(frame)

        # Apply calibrated ball filters
        candidates = filter_ball_candidates(
            frame,
            candidates
        )

        # Select the best remaining candidate
        selected = select_ball(
            candidates,
            previous_center
        )

        result = {
            "candidate": False,
            "confirmed": False,
            "x": None,
            "y": None,
            "radius": None,
            "coverage": None,
            "distance": distance
        }

        # ==================================
        # PROCESS DETECTION
        # ==================================

        if selected is not None:

            x, y, radius, coverage = selected

            if previous_center is not None:

                movement = (
                    (x - previous_center[0]) ** 2 +
                    (y - previous_center[1]) ** 2
                ) ** 0.5

                if movement <= MAX_TRACK_JUMP:
                    stable_frames += 1
                else:
                    stable_frames = 1

            else:
                stable_frames = 1

            previous_center = (x, y)
            missing_frames = 0

            confirmed = stable_frames >= CONFIRM_FRAMES

            result.update({
                "candidate": True,
                "confirmed": confirmed,
                "x": x,
                "y": y,
                "radius": radius,
                "coverage": round(coverage, 3)
            })

            # Draw detection box
            color = (
                (0, 255, 0)
                if confirmed
                else (0, 165, 255)
            )

            cv2.rectangle(
                frame,
                (x - radius, y - radius),
                (x + radius, y + radius),
                color,
                3
            )

            cv2.circle(
                frame,
                (x, y),
                5,
                (0, 0, 255),
                -1
            )

            label = (
                "CANDIDATE CONFIRMED"
                if confirmed
                else "CHECKING..."
            )

            cv2.putText(
                frame,
                label,
                (10, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2
            )

        else:

            missing_frames += 1

            if missing_frames >= 5:

                previous_center = None
                stable_frames = 0

            cv2.putText(
                frame,
                "NO CANDIDATE",
                (10, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2
            )

        # ==================================
        # DISPLAY INFORMATION
        # ==================================

        if distance is not None:

            distance_text = (
                f"Ultrasonic: {distance:.1f} cm"
            )

        else:

            distance_text = "Ultrasonic: NO READING"

        cv2.putText(
            frame,
            distance_text,
            (10, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            f"CALIBRATION | TARGET: {TARGET_DISTANCE:.0f} cm",
            (10, 55),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2
        )

        # ==================================
        # ENCODE CAMERA IMAGE
        # ==================================

        success, buffer = cv2.imencode(
            ".jpg",
            frame
        )

        if success:

            with lock:

                latest_jpeg = buffer.tobytes()
                latest_result.update(result)

        time.sleep(0.03)


# ==========================================
# 6. WEB CAMERA WINDOW
# ==========================================

@app.route("/")
def index():

    return """
    <html>

    <head>

        <title>PiCar-X Ball Calibration</title>

        <style>

            body {
                background: #171717;
                color: white;
                text-align: center;
                font-family: Arial;
            }

            img {
                width: 95%;
                max-width: 900px;
                border: 3px solid #22c55e;
                border-radius: 10px;
            }

        </style>

    </head>

    <body>

        <h1>PiCar-X Ball Calibration</h1>

        <h3>Live camera and detection</h3>

        <img src="/video_feed">

        <p>Green = confirmed candidate</p>
        <p>Orange = checking candidate</p>
        <p>Red = no candidate</p>

    </body>

    </html>
    """


@app.route("/video_feed")
def video_feed():

    def generate():

        while running:

            with lock:
                data = latest_jpeg

            if data is not None:

                yield (
                    b"--frame\r\n"
                    b"Content-Type: image/jpeg\r\n\r\n"
                    + data +
                    b"\r\n"
                )

            time.sleep(0.04)

    return Response(
        generate(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# ==========================================
# 7. USER FEEDBACK
# ==========================================

def ask_yes_no(question):

    while True:

        answer = input(
            question + " (y/n): "
        ).strip().lower()

        if answer in ("y", "yes"):
            return True

        if answer in ("n", "no"):
            return False

        print("Please enter y or n.")


# ==========================================
# 8. CONTINUE TRIAL NUMBERING
# ==========================================

def get_last_trial_number():

    if not os.path.exists(CSV_FILE):
        return 0

    last_number = 0

    try:

        with open(CSV_FILE, "r", newline="") as file:

            reader = csv.DictReader(file)

            for row in reader:

                try:

                    number = int(row["trial"])

                    last_number = max(
                        last_number,
                        number
                    )

                except (ValueError, KeyError, TypeError):
                    continue

    except Exception as error:

        print("Could not read previous trials:", error)

    return last_number


# ==========================================
# 9. SAVE EXPERIMENT
# ==========================================

def save_trial(
    trial_number,
    position,
    manual_distance,
    result,
    ball_visible,
    detection_correct,
    image_data
):

    file_exists = (
        os.path.exists(CSV_FILE)
        and os.path.getsize(CSV_FILE) > 0
    )

    timestamp = datetime.now().isoformat(
        timespec="seconds"
    )

    image_path = os.path.join(
        IMAGE_FOLDER,
        f"trial_{trial_number:03d}.jpg"
    )

    # Save the exact screenshot captured in main()
    if image_data is not None:

        with open(image_path, "wb") as image_file:
            image_file.write(image_data)

    else:

        print("WARNING: No camera image captured!")
        image_path = ""

    # Prepare CSV data
    row = {

        "trial": trial_number,

        "timestamp": timestamp,

        "position": position,

        "manual_distance_cm": manual_distance,

        "ultrasonic_distance_cm": result["distance"],

        "candidate_found": result["candidate"],

        "confirmed": result["confirmed"],

        "center_x": result["x"],

        "center_y": result["y"],

        "radius_px": result["radius"],

        "edge_coverage": result["coverage"],

        "ball_visible": ball_visible,

        "detection_correct": detection_correct,

        "image_file": image_path
    }

    # Write CSV
    with open(
        CSV_FILE,
        "a",
        newline=""
    ) as file:

        writer = csv.DictWriter(
            file,
            fieldnames=list(row.keys())
        )

        if not file_exists:
            writer.writeheader()

        writer.writerow(row)

    print("\nTRIAL SAVED SUCCESSFULLY")

    print("Trial number:", trial_number)

    print("Image:", image_path)

    print("CSV:", CSV_FILE)


# ==========================================
# 10. MAIN CALIBRATION
# ==========================================

def main():

    global running

    # Start camera detection thread
    worker = threading.Thread(
        target=camera_worker,
        daemon=True
    )

    worker.start()

    # Start browser server
    server = threading.Thread(
        target=lambda: app.run(
            host="0.0.0.0",
            port=5000,
            debug=False,
            threaded=True,
            use_reloader=False
        ),
        daemon=True
    )

    server.start()

    print("\n================================")
    print(" PICAR-X BALL CALIBRATION")
    print("================================")

    print("\nOpen your browser at:")

    print("http://YOUR_PI_IP:5000")

    print("\nCSV file:")
    print(CSV_FILE)

    print("\nImage folder:")
    print(IMAGE_FOLDER)

    print("\nTarget testing distance:")
    print(TARGET_DISTANCE, "cm")

    # Continue numbering from previous experiments
    trial_number = get_last_trial_number()

    print("\nPrevious trials:", trial_number)

    try:

        while True:

            print("\n--------------------------------")

            position = input(
                "Enter position name (or q to finish): "
            ).strip()

            if position.lower() == "q":
                break

            if not position:

                position = (
                    f"Position_{trial_number + 1}"
                )

            # Ask for manually measured distance
            manual_input = input(
                "Measured camera-to-ball distance in cm: "
            ).strip()

            try:

                manual_distance = float(manual_input)

            except ValueError:

                manual_distance = None

                print("Invalid measurement. Saving as empty.")

            # Wait for user to position the ball
            input(
                "\nPlace the ball in position, watch the browser, "
                "then press ENTER to record..."
            )

            # ==================================
            # RESET TRACKING FOR NEW TRIAL
            # ==================================

            tracking_reset_done.clear()

            reset_tracking.set()

            # Wait until camera thread resets tracking
            if not tracking_reset_done.wait(timeout=2):

                print("WARNING: Tracking reset timed out.")

            # Allow fresh frames to be processed
            time.sleep(0.5)

            # Capture result and image together
            with lock:

                result = latest_result.copy()

                image_data = latest_jpeg

            # ==================================
            # DISPLAY DETECTION RESULT
            # ==================================

            if result["candidate"]:

                print("\nALGORITHM: CANDIDATE FOUND")

                print(
                    "Confirmed:",
                    result["confirmed"]
                )

                print(
                    "Centre:",
                    result["x"],
                    result["y"]
                )

                print(
                    "Radius:",
                    result["radius"],
                    "pixels"
                )

                print(
                    "Edge coverage:",
                    result["coverage"]
                )

            else:

                print("\nALGORITHM: NO CANDIDATE FOUND")

            # ==================================
            # DISPLAY ULTRASONIC RESULT
            # ==================================

            if result["distance"] is not None:

                print(
                    "Ultrasonic distance:",
                    round(result["distance"], 1),
                    "cm"
                )

            else:

                print("Ultrasonic distance: NO READING")

            # ==================================
            # ASK USER FOR GROUND TRUTH
            # ==================================

            ball_visible = ask_yes_no(
                "\nIs the ball actually visible in the camera?"
            )

            detection_correct = ask_yes_no(
                "Is the detection box correctly identifying the ball?"
            )

            # ==================================
            # SAVE TRIAL
            # ==================================

            trial_number += 1

            save_trial(
                trial_number,
                position,
                manual_distance,
                result,
                ball_visible,
                detection_correct,
                image_data
            )

            print(
                f"\nCompleted {trial_number} total trials."
            )

    except KeyboardInterrupt:

        print("\nCalibration interrupted by user.")

    finally:

        running = False

        reset_tracking.set()

        time.sleep(0.3)

        try:
            camera.stop()
        except Exception:
            pass

        try:
            px.stop()
        except Exception:
            pass

        print("\n================================")
        print("CALIBRATION FINISHED")
        print("================================")

        print("Total trials:", trial_number)

        print("CSV:", CSV_FILE)

        print("Images:", IMAGE_FOLDER)

        print("Robot stopped.")


# ==========================================
# 11. RUN PROGRAM
# ==========================================

if __name__ == "__main__":

    try:

        main()

    except Exception as error:

        print("\nPROGRAM ERROR:", error)

        try:
            camera.stop()
        except Exception:
            pass

        try:
            px.stop()
        except Exception:
            pass
