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
R_PIN = 6 
RR_PIN = 4
rotatechange = 2.0 #max 2.8
speedchange = 0.1 #max 0.2
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
        super().__init__('nav')
        #publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info('Publisher for Twist')
        self.counter = 0
        self.x = 1.0
        self.z = 1.0

    def turnRight(self):
        self.get_logger().info('turnRight')
        self.stopbot()

    def turnLeft(self):
        self.get_logger().info('turnLeft')
        self.stopbot()

    def moveStraight(self):
        self.get_logger().info('straight')
        self.x = -speedchange
        self.z = 0.0

    def reverse(self):
        self.get_logger().info('reverse')
        self.x = 0.01
        self.z = 0.0

    def nudgeLeft(self):
        self.get_logger().info('nudgeLeft')
        self.z = -rotatechange

    def nudgeRight(self):
        self.get_logger().info('nudgeRight')
        self.z = rotatechange

    def checkPoint(self):
        self.counter += 1

    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        self.x = 0.0
        self.z = 0.0
        # time.sleep(1)

    def mover(self):
        try:
            while True:
                innerSensor = [GPIO.input(L_PIN), GPIO.input(R_PIN)]
                outerSensor = [GPIO.input(LL_PIN), GPIO.input(RR_PIN)]
                print("----------------")
                print(innerSensor)
                print(outerSensor)
                if ([1, 1] == innerSensor):
                    if ([0, 0] == outerSensor):
                        self.moveStraight()
                    elif ([0, 1] == outerSensor):
                        self.turnRight()
                    elif ([0, 0] == outerSensor):
                        self.turnLeft()
                    elif ([1, 1] == outerSensor):
                        self.checkPoint()
                elif ([0, 1] == innerSensor):
                    self.nudgeRight()
                elif ([1, 0] == innerSensor):
                    self.nudgeLeft()
                elif ([0, 0] == innerSensor):
                    self.reverse()
                twist = Twist()
                # twist.linear.x = self.x
                # twist.angular.z = self.z
                print(self.x, self.z)
                print(twist.linear.x, twist.angular.z)
                self.publisher_.publish(twist)

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
    print("closing")
    nav.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()

if __name__ == '__main__':
    main()

