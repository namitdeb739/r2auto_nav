import RPi.GPIO as GPIO
import time

PAYLOAD_PIN = 5
GPIO.setmode(GPIO.BCM)
GPIO.setup(PAYLOAD_PIN, GPIO.OUT)
GPIO.setwarnings(False)
p = GPIO.PWM(PAYLOAD_PIN, 50)

p.start(2.5)
for i in range(5):
    for j in range(90, 25, -2):
        change = j/10
        p.ChangeDutyCycle(change)
        time.sleep(0.1)
        print("open?")
    for j in range(25, 90, 2):
        change = j/10
        p.ChangeDutyCycle(change)
        time.sleep(0.1)
        print("close?")
    print("wtf")

p.stop()
GPIO.cleanup()
# Example usage:
<<<<<<< HEAD
if __name__ == "__main__":
    PAYLOAD_PIN = 18  # Change this to your desired GPIO pin
    control_servo(PAYLOAD_PIN)
=======
if name == "main":
    PAYLOAD_PIN = 18  # Change this to your desired GPIO pin
    control_servo(PAYLOAD_PIN)
>>>>>>> 5c3de9cc1c28110ae95dd7a2051ef96fb29ebbac
>>>>>>> 0c9c1ae38e6c7b3f44765b6ec520f7f2bd3aa061
