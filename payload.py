import RPi.GPIO as GPIO
import time

OPEN_ANGLE = 35
CLOSE_ANGLE = 110
PAYLOAD_PIN = 5
GPIO.setmode(GPIO.BCM)
GPIO.setup(PAYLOAD_PIN, GPIO.OUT)
GPIO.setwarnings(False)
p = GPIO.PWM(PAYLOAD_PIN, 50)

p.start(2.5)
for i in range(5):
    for j in range(CLOSE_ANGLE,OPEN_ANGLE, -3):
        change = j/10
        p.ChangeDutyCycle(change)
        time.sleep(0.1)
        print("open?")
    for j in range(OPEN_ANGLE, CLOSE_ANGLE, 6):
        change = j/10
        p.ChangeDutyCycle(change)
        time.sleep(0.1)
        print("close?")
    print("wtf")

p.stop()
GPIO.cleanup()
