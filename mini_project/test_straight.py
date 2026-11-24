from picarx import Picarx
import time

px = Picarx()

try:
    px.set_dir_servo_angle(0)
    px.forward(15)
    time.sleep(1)
finally:
    px.stop()
