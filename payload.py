import RPi.GPIO as GPIO
import time

# def control_servo(PAYLOAD_PIN):
#     # Set GPIO mode
#     GPIO.setmode(GPIO.BCM)

#     # Set GPIO pin as output
#     GPIO.setup(PAYLOAD_PIN, GPIO.OUT)

#     # Create PWM instance with frequency
#     pwm = GPIO.PWM(PAYLOAD_PIN, 50)  # 50 Hz frequency

#     # Start PWM with 0 duty cycle (servo fully closed)
#     pwm.start(2.5)

#     try:
#         pwm.ChangeDutyCycle(12.5)
#         time.sleep(3)
#         pwm.ChangeDutyCycle(2.5)
#         time.sleep(3)
#         for x in range(25, 76, 1):
#             theta = x / 10.0

#             # Open the servo (fully open)
#             pwm.ChangeDutyCycle(theta)  # Adjust duty cycle to open fully
#             time.sleep(0.5)  # Wait for a moment

#         for x in range(75, 26, -1):
#             theta = x / 10.0

#             # Open the servo (fully open)
#             pwm.ChangeDutyCycle(theta)  # Adjust duty cycle to open fully
#             time.sleep(0.5)  # Wait for a moment

#     except KeyboardInterrupt:
#         # Clean up GPIO
#         pwm.stop()
#         GPIO.cleanup()
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