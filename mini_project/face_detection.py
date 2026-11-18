
from flask import Flask, Response
from picamera2 import Picamera2
import cv2
import time

app = Flask(__name__)

# Initialize the PiCar-X camera
camera = Picamera2()
camera.configure(
    camera.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"}
    )
)
camera.start()
time.sleep(2)

# Load face detection model
face_detector = cv2.CascadeClassifier(
    "/usr/share/opencv4/haarcascades/haarcascade_frontalface_default.xml"
)

if face_detector.empty():
    raise RuntimeError("Face detection model failed to load")

# Camera webpage
@app.route("/")
def home():
    return """
    <html>
    <head>
        <title>PiCar-X Camera</title>
        <style>
            body {
                background: #111827;
                color: white;
                font-family: Arial;
                text-align: center;
                padding: 20px;
            }
            h1 { color: #4ade80; }
            img {
                width: 90%;
                max-width: 800px;
                border: 3px solid #4ade80;
                border-radius: 10px;
            }
        </style>
    </head>
    <body>
        <h1>PiCar-X Live Camera</h1>
        <p>Face Detection Active</p>
        <img src="/video_feed">
    </body>
    </html>
    """

# Capture frames and detect faces
def generate_frames():
    while True:
        frame = camera.capture_array()

        gray = cv2.cvtColor(
            frame, cv2.COLOR_BGR2GRAY
        )

        faces = face_detector.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(40, 40)
        )

        for (x, y, w, h) in faces:
            cv2.rectangle(
                frame,
                (x, y),
                (x + w, y + h),
                (0, 255, 0),
                2
            )

            cv2.putText(
                frame,
                "FACE DETECTED",
                (x, max(y - 10, 20)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2
            )

        cv2.putText(
            frame,
            f"Faces detected: {len(faces)}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 0),
            2
        )

        success, buffer = cv2.imencode(
            ".jpg", frame
        )

        if not success:
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n"
            + buffer.tobytes()
            + b"\r\n"
        )

@app.route("/video_feed")
def video_feed():
    return Response(
        generate_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )

if __name__ == "__main__":
    try:
        app.run(
            host="0.0.0.0",
            port=5001,
            threaded=True
        )
    finally:
        camera.stop()
