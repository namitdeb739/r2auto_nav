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

LL_PIN = 19
L_PIN = 6
R_PIN = 13
RR_PIN = 26
rotatechange = 2.75/4 #max 2.8
speedchange = -0.10 #max 0.22
stop_distance = 0.25
firstCheck = True
reverse = False

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
        self.x = 0.0
        self.z = 0.0

    def publish(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.angular.z = self.z
        print(f"{twist.linear.x}, {twist.angular.z}")
        self.publisher_.publish(twist)
        print(f"{twist.linear.x}, {twist.angular.z}")

    def turnRight(self):
        self.get_logger().info('turnRight')
        while GPIO.input(RR_PIN):
            self.x = 0.0
            self.z = -rotatechange
            self.publish()
        while not (GPIO.input(L_PIN) and GPIO.input(R_PIN)):
            self.x = 0.0
            self.z = -rotatechange
            self.publish()

    def turnLeft(self):
        self.get_logger().info('turnLeft')
        self.z = rotatechange
        self.x = speedchange/4
        while GPIO.input(LL_PIN):
            self.x = 0.0
            self.z = rotatechange
            self.publish()
        while not (GPIO.input(L_PIN) and GPIO.input(R_PIN)):
            self.x = 0.0
            self.z = rotatechange
            self.publish()

    def moveStraight(self):
        self.get_logger().info('stght')
        self.x = speedchange
        self.z = 0.0
        print(f"self.x: {self.x}, self.z: {self.z}")

    def reverse(self):
        self.get_logger().info('reverse')
        self.x = 0.01
        self.z = 0.0
        self.publish()
        nudge = 0
        first = True
        while ((not GPIO.input(L_PIN)) and (not GPIO.input(R_PIN))):
            if (first and GPIO.input(L_PIN)):
                first = False
                nudge = 0 #go left
            if (first and GPIO.input(R_PIN)):
                first = False
                nudge = 1 #go right
        self.x = 0.0
        while (GPIO.input(LL_PIN) or GPIO.input(RR_PIN)):
            if (nudge == 0):
                self.z = rotatechange #nudgeleft
            else:
                self.z =  -rotatechange #nudgeRight
            self.publish()

    def nudgeLeft(self):
        self.get_logger().info('nudgeLeft')
        self.x = speedchange/5
        self.z = rotatechange/5

    def nudgeRight(self):
        self.get_logger().info('nudgeRight')
        self.x = speedchange/5
        self.z = -rotatechange/5

    def checkPoint(self):
        global firstCheck, timed
        self.get_logger().info('check')
        if firstCheck or (time.time() - timed > 10):
            timed = time.time()
            firstCheck = False
            self.counter += 1
            self.get_logger().info('Checkpoint: ')
            self.get_logger().info(str(self.counter))


    def stopbot(self):
        self.get_logger().info('In stopbot')
        # publish to cmd_vel to move TurtleBot
        self.x = 0.0
        self.z = 0.0
        # time.sleep(1)

    def mover(self):
        global reverse
        try:
            while True:
                innerSensor = [GPIO.input(L_PIN), GPIO.input(R_PIN)]
                outerSensor = [GPIO.input(LL_PIN), GPIO.input(RR_PIN)]
                print("----------------")
                print(f"inner: {innerSensor}")
                print(f"outer: {outerSensor}")
                if ([1, 1] == innerSensor):
                    if ([0, 0] == outerSensor):
                        self.moveStraight()
                        if (reverse):
                            reverse = False
                    elif ([0, 1] == outerSensor):
                        self.turnRight()
                        pass
                    elif ([1, 0] == outerSensor):
                        self.turnLeft()
                        pass
                    elif ([1, 1] == outerSensor):
                        self.checkPoint()
                        pass
                elif ([0, 1] == innerSensor):
                    self.nudgeRight()
                elif ([1, 0] == innerSensor):
                    self.nudgeLeft()
                elif ([0, 0] == innerSensor):
                    self.reverse()
                self.publish()
                print(f"self.x: {self.x}, self.z: {self.z}")
                

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

