import RPi.GPIO as GPIO
import time

<<<<<<< HEAD
def control_servo(PAYLOAD_PIN):
    # Set GPIO mode
    GPIO.setmode(GPIO.BCM)

    # Set GPIO pin as output
    GPIO.setup(PAYLOAD_PIN, GPIO.OUT)

    # Create PWM instance with frequency
    pwm = GPIO.PWM(PAYLOAD_PIN, 50)  # 50 Hz frequency

    # Start PWM with 0 duty cycle (servo fully closed)
    pwm.start(0)

    try:
        for x in range(25, 76, 1):
            theta = x / 10.0

            # Open the servo (fully open)
            pwm.ChangeDutyCycle(theta)  # Adjust duty cycle to open fully
            time.sleep(0.5)  # Wait for a moment

        for x in range(75, 26, -1):
            theta = x / 10.0

            # Open the servo (fully open)
            pwm.ChangeDutyCycle(theta)  # Adjust duty cycle to open fully
            time.sleep(0.5)  # Wait for a moment

    except KeyboardInterrupt:
        # Clean up GPIO
        pwm.stop()
        GPIO.cleanup()

=======
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
>>>>>>> ec3980e9c5ea34aa45d70d0970ae3b7544ac2765
