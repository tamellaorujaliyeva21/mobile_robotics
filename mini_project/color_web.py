
from flask import Flask, Response, render_template_string
from picamera2 import Picamera2
import cv2
import numpy as np
import time
import threading

app = Flask(__name__)

# Your calibrated color ranges
COLOR_RANGES = {
    "RED": [(0, 3), (175, 179)],
    "ORANGE": [(4, 21)],
    "YELLOW": [(22, 35)],
    "GREEN": [(36, 85)],
    "BLUE": [(86, 124)],
    "PURPLE": [(125, 174)]
}

MIN_SATURATION = 75
MIN_BRIGHTNESS = 65
MIN_COLOR_PERCENT = 35
CONFIRM_FRAMES = 8

camera = Picamera2()
config = camera.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
camera.configure(config)
camera.start()
time.sleep(2)

camera_lock = threading.Lock()

last_color = None
candidate = None
candidate_count = 0


def detect_color(frame):
    h, w = frame.shape[:2]

    x1, x2 = w * 3 // 8, w * 5 // 8
    y1, y2 = h * 3 // 8, h * 5 // 8

    center = frame[y1:y2, x1:x2]

    # PiCamera RGB888 array is BGR in this setup
    hsv = cv2.cvtColor(center, cv2.COLOR_BGR2HSV)

    hue = hsv[:, :, 0]
    sat = hsv[:, :, 1]
    val = hsv[:, :, 2]

    valid = (
        (sat >= MIN_SATURATION) &
        (val >= MIN_BRIGHTNESS)
    )

    values = np.median(
        hsv.reshape(-1, 3), axis=0
    ).astype(int)

    if np.count_nonzero(valid) < valid.size * 0.35:
        return None, values

    scores = {}

    for color, ranges in COLOR_RANGES.items():
        mask = np.zeros(hue.shape, dtype=bool)

        for low, high in ranges:
            mask |= (hue >= low) & (hue <= high)

        scores[color] = np.count_nonzero(mask & valid)

    best = max(scores, key=scores.get)
    percentage = scores[best] / valid.size * 100

    if percentage >= MIN_COLOR_PERCENT:
        return best, values

    return None, values


def generate_frames():
    global last_color, candidate, candidate_count

    while True:
        with camera_lock:
            frame = camera.capture_array()

            color, hsv_values = detect_color(frame)

            if color == candidate:
                candidate_count += 1
            else:
                candidate = color
                candidate_count = 1

            if candidate_count >= CONFIRM_FRAMES:
                if color != last_color:
                    last_color = color

                    if color is None:
                        print("PiCar-X: No color detected",
                              flush=True)
                    else:
                        print(f"PiCar-X: I see {color}!",
                              flush=True)

            h, w = frame.shape[:2]
            x1, x2 = w * 3 // 8, w * 5 // 8
            y1, y2 = h * 3 // 8, h * 5 // 8

            # Green detection rectangle
            cv2.rectangle(
                frame, (x1, y1), (x2, y2),
                (0, 255, 0), 2
            )

            label = last_color or "NONE"

            cv2.rectangle(
                frame, (5, 5), (390, 82),
                (0, 0, 0), -1
            )

            cv2.putText(
                frame, "Color: " + label,
                (15, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (255, 255, 255), 2
            )

            cv2.putText(
                frame,
                f"H:{hsv_values[0]} "
                f"S:{hsv_values[1]} "
                f"V:{hsv_values[2]}",
                (15, 65),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 2
            )

            success, buffer = cv2.imencode(
                ".jpg", frame,
                [cv2.IMWRITE_JPEG_QUALITY, 75]
            )

        if not success:
            continue

        jpg = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + jpg + b"\r\n"
        )

        time.sleep(0.05)


HTML = """
<!DOCTYPE html>
<html>
<head>
<title>PiCar-X Color Detection</title>
<meta name="viewport"
      content="width=device-width, initial-scale=1">
<style>
body {
    background: #111827;
    color: white;
    font-family: Arial, sans-serif;
    text-align: center;
    margin: 20px;
}
img {
    width: 100%;
    max-width: 800px;
    border: 3px solid #22c55e;
    border-radius: 12px;
}
</style>
</head>
<body>
<h1>PiCar-X Live Color Detection</h1>
<p>Place your color card in the green rectangle.</p>
<img src="/video_feed" alt="Live PiCar-X camera">
<p>Detected color and HSV values appear on the video.</p>
</body>
</html>
"""


@app.route("/")
def index():
    return render_template_string(HTML)


@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


if __name__ == "__main__":
    try:
        print("Starting PiCar-X browser camera")
        app.run(
            host="0.0.0.0",
            port=5000,
            debug=False,
            threaded=True
        )
    finally:
        camera.stop()
        camera.close()

