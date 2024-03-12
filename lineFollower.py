import RPi.GPIO as GPIO
import time
import math
import rclpy
import rclpy.node import Node

LL_PIN = 1
L_PIN = 2
R_PIN = 3
RR_PIN = 4

class linerMover(Node):

    def __init__(self):
        super().__init__('auto_nav')
        #publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Publisher for Twist')
        self.counter = 0

    def GPIO_setup():
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(LL_PIN, GPIO.IN)
        GPIO.setup(L_PIN, GPIO.IN)
        GPIO.setup(R_PIN, GPIO.IN)
        GPIO.setup(RR_PIN, GPIO.IN)

    def turnRight(self):
        pass

    def turnLeft(self):
        pass

    def moveStraight(self):
        pass

    def reverse(self):
        pass

    def nudgeLeft(self):
        pass

    def nudgeRight(self):
        pass

    def checkPoint(self):
        pass

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def mover(self):
        try:
            while rclpy.ok():
                innerSensor = [GPIO.input(L_PIN), GPIO.input(R_PIN)]
                outerSensor = [GPIO.input(LL_PIN), GPIO.input(RR_PIN)]
                self.get_loader().info(sensors)
                if ([1, 1] == innerSensor):
                    if ([0, 0] == outerSensor):
                        moveStraight()
                    elif ([0, 1] == outerSensor):
                        turnRight()
                    elif ([0, 0] == outerSensor):
                        turnLeft()
                    elif ([1, 1] == outerSensor):
                        checkPoint()
                elif ([1, 0] == innerSensor):
                    nudgeRight()
                elif ([0, 1]) == innerSensor):
                    nudgeLeft()
                elif ([0, 0]) == innerSensor):
                    reverse()

        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            #stop moving
            self.stopbot()

def main(args = None):
    rclpy.init(args=args)

    nav = linerMover()
    nav.mover()
    GPIO_setup()

    nav.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()

