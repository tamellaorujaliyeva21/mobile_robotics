from picarx import Picarx
import time
import statistics

px = Picarx()

previous_distance = None

def get_stable_distance():
    readings = []

    for _ in range(7):
        distance = px.get_distance()

        if 2 <= distance <= 400:
            readings.append(distance)

        time.sleep(0.06)

    if len(readings) < 4:
        return None

    return statistics.median(readings)


try:
    while True:
        distance = get_stable_distance()

        if distance is None:
            print("Unreliable reading")
            continue

        if previous_distance is None:
            previous_distance = distance

        # Ignore small changes under 5 cm
        if abs(distance - previous_distance) >= 5:
            previous_distance = distance

        print(f"Filtered distance: {previous_distance:.1f} cm")

        time.sleep(0.3)

except KeyboardInterrupt:
    print("\nSensor stopped")
