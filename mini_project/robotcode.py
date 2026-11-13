from gpiozero import Motor
from time import sleep

# Left motor (L298N: IN1, IN2, ENA)
left = Motor(forward=17, backward=27, enable=22)

# Right motor (L298N: IN3, IN4, ENB)
right = Motor(forward=23, backward=24, enable=25)

SPEED = 0.7  # 0.0 to 1.0

def forward(speed=SPEED):
    left.forward(speed)
    right.forward(speed)

def stop():
    left.stop()
    right.stop()

try:
    print("Going forward...")
    forward()
    sleep(3)          # run for 3 seconds
finally:
    stop()
    print("Stopped")
