
from picamera2 import Picamera2
import cv2
import numpy as np
import time

# ===== CALIBRATED COLOR RANGES =====

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

# ===== CAMERA SETUP =====

camera = Picamera2()
config = camera.create_video_configuration(
    main={"size": (640, 480), "format": "RGB888"}
)
camera.configure(config)
camera.start()
time.sleep(2)

print("PiCar-X calibrated color detection started")
print("Show your color cards to the camera")
print("Press Ctrl+C to stop")

def detect_color(frame):
    height, width = frame.shape[:2]

    # Center detection area
    center = frame[
        height * 3 // 8:height * 5 // 8,
        width * 3 // 8:width * 5 // 8
    ]

    hsv = cv2.cvtColor(
        center, cv2.COLOR_BGR2HSV
    )
    

    
    h = hsv[:, :, 0]
    s = hsv[:, :, 1]
    v = hsv[:, :, 2]

    valid = (
        (s >= MIN_SATURATION) &
        (v >= MIN_BRIGHTNESS)
    )

    if np.count_nonzero(valid) < valid.size * 0.35:
        return None

    scores = {}

    for color, ranges in COLOR_RANGES.items():
        mask = np.zeros(h.shape, dtype=bool)

        for lower, upper in ranges:
            mask |= (h >= lower) & (h <= upper)

        scores[color] = np.count_nonzero(mask & valid)

    best = max(scores, key=scores.get)

    percentage = scores[best] / valid.size * 100

    if percentage >= MIN_COLOR_PERCENT:
        return best

    return None


# ===== DETECTION LOOP =====

last_color = None
candidate = None
count = 0


try:
    while True:
        frame = camera.capture_array()

        color = detect_color(frame)

        height, width = frame.shape[:2]

        x1 = width * 3 // 8
        x2 = width * 5 // 8
        y1 = height * 3 // 8
        y2 = height * 5 // 8

        # Obtain HSV values for the center
        center = frame[y1:y2, x1:x2]
        hsv = cv2.cvtColor(center, cv2.COLOR_BGR2HSV)

        median_hsv = np.median(
            hsv.reshape(-1, 3), axis=0
        ).astype(int)

        # Confirm color over several frames
        if color == candidate:
            count += 1
        else:
            candidate = color
            count = 1

        if count >= CONFIRM_FRAMES and color != last_color:
            last_color = color

            if color is None:
                print("PiCar-X: No color detected")
            else:
                print(f"PiCar-X: I see {color}!")

        # Draw detection rectangle
        cv2.rectangle(
            frame, (x1, y1), (x2, y2),
            (0, 255, 0), 2
        )

        # Draw detected color label
        label = last_color if last_color else "NONE"

        cv2.rectangle(
            frame, (5, 5), (350, 72),
            (0, 0, 0), -1
        )

        cv2.putText(
            frame, f"Color: {label}",
            (15, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8, (255, 255, 255), 2
        )

        cv2.putText(
            frame,
            f"H:{median_hsv[0]} "
            f"S:{median_hsv[1]} "
            f"V:{median_hsv[2]}",
            (15, 58),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6, (255, 255, 255), 2
        )

        # Show live camera
        cv2.imshow("PiCar-X Color Detection", frame)

        # Press Q to close
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

        time.sleep(0.1)

except KeyboardInterrupt:
    print("\nStopping detection")

finally:
    camera.stop()
    camera.close()
    cv2.destroyAllWindows()
    print("Camera stopped")


