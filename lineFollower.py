import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time

LL_PIN = 1
L_PIN = 26
R_PIN = 5
RR_PIN = 4
rotatechange = 0.1
speedchange = 0.05
stop_distance = 0.25
def GPIO_setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LL_PIN, GPIO.IN)
    GPIO.setup(L_PIN, GPIO.IN)
    GPIO.setup(R_PIN, GPIO.IN)
    GPIO.setup(RR_PIN, GPIO.IN)

class linerMover(Node):
    def __init__(self):
        super().__init__('auto_nav')
        #publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Publisher for Twist')
        self.counter = 0
        twist = Twist()


    def turnRight(self):
        self.get_logger().info('turnRight')
        stopbot()

    def turnLeft(self):
        self.get_logger().info('turnLeft')
        stopbot()

    def moveStraight(self):
        self.get_logger().info('straight')
        self.twist.linear.x = speedchange
        self.twist.angular.z = 0

    def reverse(self):
        self.get_logger().info('reverse')
        self.twist.linear.x = -speedchange

    def nudgeLeft(self):
        self.get_logger().info('nudgeLeft')
        self.twist.angular.z = -rotatechange

    def nudgeRight(self):
        self.get_logger().info('nudgeRight')
        self.twist.angular.z = rotatechange

    def checkPoint(self):
        self.counter += 1

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0
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
                elif ([0, 1] == innerSensor):
                    nudgeLeft()
                elif ([0, 0] == innerSensor):
                    reverse()

        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            #stop moving
            self.stopbot()

def main(args = None):
    rclpy.init(args=args)
    GPIO_setup()
    nav = linerMover()
    nav.mover()

    nav.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()

