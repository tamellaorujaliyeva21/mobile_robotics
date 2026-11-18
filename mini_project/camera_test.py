from vilib import Vilib
from time import sleep

Vilib.camera_start(vflip=False, hflip=False)
Vilib.display(local=False, web=True)

print("Camera running. Open http://<pi-ip>:9000/mjpg in a browser")

while True:
    sleep(1)
