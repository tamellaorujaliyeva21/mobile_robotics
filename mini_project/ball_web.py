
from flask import Flask, Response
from picamera2 import Picamera2
from picarx import Picarx

import cv2
import time
import statistics

from collections import deque

from ball_detection import (
    WIDTH,
    HEIGHT,
    find_ball_candidates,
    select_ball,
    CONFIRM_FRAMES,
    MAX_TRACK_JUMP
)

# =====================================
# SETTINGS
# =====================================

TARGET_DISTANCE = 40.0
DISTANCE_TOLERANCE = 5.0

MIN_DISTANCE = TARGET_DISTANCE - DISTANCE_TOLERANCE
MAX_DISTANCE = TARGET_DISTANCE + DISTANCE_TOLERANCE

SENSOR_INTERVAL = 0.20

app = Flask(__name__)

# =====================================
# INITIALIZE HARDWARE
# =====================================

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

# Keep recent valid ultrasonic readings
distance_history = deque(maxlen=5)


# =====================================
# ULTRASONIC DISTANCE
# =====================================

def read_distance():

    try:
        distance = px.get_distance()

        if distance is None:
            return None

        distance = float(distance)

        if 2 <= distance <= 200:
            return distance

    except Exception as error:
        print("Ultrasonic error:", error)

    return None


# =====================================
# LIVE CAMERA STREAM
# =====================================

def generate_frames():

    previous_center = None

    stable_frames = 0
    missing_frames = 0

    distance_cm = None
    last_sensor_read = 0
    last_valid_sensor_time = 0

    while True:

        # Capture camera frame
        # RGB888 is already in OpenCV BGR byte order
        frame = camera.capture_array()

        # =================================
        # UPDATE ULTRASONIC SENSOR
        # =================================

        now = time.monotonic()

        if now - last_sensor_read >= SENSOR_INTERVAL:

            last_sensor_read = now

            reading = read_distance()

            if reading is not None:

                distance_history.append(reading)
                last_valid_sensor_time = now

                if len(distance_history) >= 3:
                    distance_cm = statistics.median(
                        distance_history
                    )

            else:
                distance_cm = None
                distance_history.clear()

        # Reject stale sensor measurements
        if now - last_valid_sensor_time > 0.7:
            distance_cm = None
            distance_history.clear()

        # =================================
        # FIND BALL
        # =================================

        candidates = find_ball_candidates(frame)

        selected = select_ball(
            candidates,
            previous_center
        )

        distance_ok = (
            distance_cm is not None
            and MIN_DISTANCE <= distance_cm <= MAX_DISTANCE
        )

        # =================================
        # TRACK BALL
        # =================================

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

            confirmed = (
                stable_frames >= CONFIRM_FRAMES
            )

            # =================================
            # DETECTION CONDITIONS
            # =================================

            if confirmed and distance_ok:

                label = "BALL DETECTED"
                color = (0, 255, 0)

            elif confirmed and not distance_ok:

                label = "BALL OUTSIDE 40 CM RANGE"
                color = (0, 165, 255)

            else:

                label = "CHECKING BALL..."
                color = (0, 255, 255)

            # Draw bounding box
            cv2.rectangle(
                frame,
                (max(0, x-radius), max(0, y-radius)),
                (min(WIDTH-1, x+radius),
                 min(HEIGHT-1, y+radius)),
                color,
                3
            )

            # Ball centre
            cv2.circle(
                frame,
                (x, y),
                5,
                (0, 0, 255),
                -1
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

            cv2.putText(
                frame,
                f"Radius: {radius}px",
                (10, 105),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2
            )

        else:

            missing_frames += 1

            if missing_frames >= 5:
                previous_center = None
                stable_frames = 0

            cv2.putText(
                frame,
                "NO BALL SHAPE DETECTED",
                (10, 75),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (0, 0, 255),
                2
            )

        # =================================
        # DISTANCE DISPLAY
        # =================================

        if distance_cm is not None:

            distance_text = (
                f"Ultrasonic: {distance_cm:.1f} cm"
            )

            if distance_ok:
                range_text = "40 CM RANGE: OK"
                range_color = (0, 255, 0)
            else:
                range_text = "OUTSIDE 40 CM RANGE"
                range_color = (0, 165, 255)

        else:

            distance_text = "Ultrasonic: NO READING"
            range_text = "RANGE UNKNOWN"
            range_color = (0, 0, 255)

        cv2.putText(
            frame,
            distance_text,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2
        )

        cv2.putText(
            frame,
            range_text,
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            range_color,
            2
        )

        # =================================
        # WEB IMAGE
        # =================================

        success, buffer = cv2.imencode(
            ".jpg",
            frame
        )

        if not success:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + buffer.tobytes()
            + b"\r\n"
        )


# =====================================
# WEB PAGE
# =====================================

@app.route("/")
def index():

    return """
    <html>
    <head>
        <title>PiCar-X Ball Detection</title>

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

        <h1>PiCar-X Ball Detection</h1>

        <h3>Colour-independent ball detection</h3>

        <img src="/video_feed">

        <p>Green box = ball detected within 40 cm range</p>
        <p>Orange box = ball detected outside the range</p>
        <p>Red message = no circular object detected</p>

    </body>
    </html>
    """


@app.route("/video_feed")
def video_feed():

    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


# =====================================
# START SERVER
# =====================================

if __name__ == "__main__":

    print("PiCar-X Ball Detection Web Server")
    print("Target distance:", TARGET_DISTANCE, "cm")
    print("Accepted range:", MIN_DISTANCE, "-", MAX_DISTANCE, "cm")

    try:

        app.run(
            host="0.0.0.0",
            port=5000,
            debug=False,
            threaded=False,
            use_reloader=False
        )

    finally:

        camera.stop()
        px.stop()
        print("Camera and robot stopped.")
