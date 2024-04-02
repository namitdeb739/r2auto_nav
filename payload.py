import RPi.GPIO as GPIO
import time

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

# Example usage:
if __name__ == "__main__":
    PAYLOAD_PIN = 18  # Change this to your desired GPIO pin
    control_servo(PAYLOAD_PIN)
