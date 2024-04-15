import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool
import numpy as np
import math
import cmath
import time
import subprocess
import requests
import json

OPEN_ANGLE = 35
CLOSE_ANGLE = 110
PAYLOAD_PIN = 5
GPIO.setmode(GPIO.BCM)
GPIO.setup(PAYLOAD_PIN, GPIO.OUT)
GPIO.setwarnings(False)
p = GPIO.PWM(PAYLOAD_PIN, 50)


def payload():
    p.start(2.5)
    for i in range(1):
        for j in range(CLOSE_ANGLE, OPEN_ANGLE, -3):
            change = j / 10
            p.ChangeDutyCycle(change)
            time.sleep(0.1)
            print("open?")
        time.sleep(1)
        for j in range(OPEN_ANGLE, CLOSE_ANGLE, 6):
            change = j / 10
            p.ChangeDutyCycle(change)
            time.sleep(0.1)
            print("close?")
        print("wtf")

    p.stop()
    GPIO.cleanup()


def door():
    url = "http://192.168.59.89/openDoor"
    myobj = {"action": "openDoor", "parameters": {"robotId": "34"}}
    x = requests.post(url, json=myobj)
    x_dict = json.loads(x.text)
    status = x_dict["status"]
    door_num = x_dict["data"]["message"]
    return door_num


LL_PIN = 19
L_PIN = 6
R_PIN = 13
RR_PIN = 26
rotatechange = -2.75 / 4  # max 2.8
speedchange = 0.10  # max 0.22
stop_distance = 0.25
firstCheck = True
reverse = False
# espTime = time.now()


def GPIO_setup():
    GPIO.setwarnings(False)
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(LL_PIN, GPIO.IN)
    GPIO.setup(L_PIN, GPIO.IN)
    GPIO.setup(R_PIN, GPIO.IN)
    GPIO.setup(RR_PIN, GPIO.IN)


class linerMover(Node):
    def __init__(self):
        super().__init__("nav")
        # publisher for moving TurtleBot
        self.publisher_twist = self.create_publisher(Twist, "cmd_vel", 10)
        self.publisher_bool = self.create_publisher(Bool, "checkpoint", 10)
        self.get_logger().info("Publisher for Twist")
        self.counter = 0
        self.x = 0.0
        self.z = 0.0

    def publish(self):
        twist = Twist()
        twist.linear.x = self.x
        twist.angular.z = self.z
        print(f"twist.linear.x: {twist.linear.x}, twist.angular.z: {twist.angular.z}")
        self.publisher_twist.publish(twist)

    def turnRight(self):
        self.get_logger().info("turnRight")
        while GPIO.input(RR_PIN):
            self.x = -0.01
            self.z = -rotatechange
            self.publish()
        while not (GPIO.input(L_PIN) and GPIO.input(R_PIN)):
            self.x = -0.0085
            self.z = -rotatechange
            self.publish()

    def turnLeft(self):
        self.get_logger().info("turnLeftttttttt")
        self.z = rotatechange
        self.x = speedchange / 4
        while GPIO.input(LL_PIN):
            self.x = -0.01
            self.z = rotatechange
            self.publish()
        while not (GPIO.input(L_PIN) and GPIO.input(R_PIN)):
            self.x = -0.0085
            self.z = rotatechange
            self.publish()

    def moveStraight(self):
        self.get_logger().info("stght")
        self.x = speedchange
        self.z = 0.0

    def reverse(self):
        global innerSensor
        global outerSensor
        self.get_logger().info("reverseeeeeee")
        self.x = 0.01
        self.z = 0.0
        self.publish()
        nudge = 0
        first = True
        while [0, 0] != innerSensor:
            innerSensor = [GPIO.input(L_PIN), GPIO.input(R_PIN)]
            outerSensor = [GPIO.input(LL_PIN), GPIO.input(RR_PIN)]
            print(f"innerrrr: {innerSensor}")
            print(f"outerrrr: {outerSensor}")

            if first and (innerSensor[0] or (outerSensor[0] and [0, 0] == innerSensor)):
                first = False
                nudge = 0  # go left
            if first and (innerSensor[1] or (outerSensor[1] and [0, 0] == innerSensor)):
                first = False
                nudge = 1  # go right
        self.get_logger().info("grhsbdruihgrdnughdr")
        self.x = 0.0
        print(f"nudge: {nudge}")
        while GPIO.input(LL_PIN) and GPIO.input(RR_PIN):
            if nudge == 0:
                self.z = rotatechange  # nudgeleft
            else:
                self.z = -rotatechange  # nudgeRight
            self.get_logger().info("gebhgbresg")
            self.publish()

    def nudgeLeft(self):
        self.get_logger().info("nudgeLeft")
        self.x = speedchange / 5
        self.z = rotatechange / 5

    def nudgeRight(self):
        self.get_logger().info("nudgeRight")
        self.x = speedchange / 5
        self.z = -rotatechange / 5

    def checkPoint(self):
        global firstCheck, timed
        self.get_logger().info("check")
        if firstCheck or (time.time() - timed > 2):
            timed = time.time()
            firstCheck = False
            self.counter += 1
            self.get_logger().info("Checkpoint: ")
            self.get_logger().info(str(self.counter))
            if self.counter == 1:
                door_num = door()
                if "1" in door_num:
                    turnLeft()
                elif "2" in door_num:
                    turnRight()

                # espTime = time.now()

            if self.counter == 2:
                payload()

    def stopbot(self):
        self.get_logger().info("In stopbot")
        # publish to cmd_vel to move TurtleBot
        self.x = 0.0
        self.z = 0.0
        # time.sleep(1)

    def mover(self):
        global reverse
        global innerSensor
        global outerSensor
        try:
            while True:
                self.publisher_bool.publish(False)
                if self.counter >= 3:
                    time.sleep(5)
                    self.publisher_bool.publish(True)
                    break

                innerSensor = [GPIO.input(L_PIN), GPIO.input(R_PIN)]
                outerSensor = [GPIO.input(LL_PIN), GPIO.input(RR_PIN)]
                print("----------------")
                print(f"inner: {innerSensor}")
                print(f"outer: {outerSensor}")
                if [1, 1] == innerSensor:
                    if [0, 0] == outerSensor:
                        self.moveStraight()
                    elif [0, 1] == outerSensor:
                        self.turnRight()
                    elif [1, 0] == outerSensor:
                        self.turnLeft()
                    elif [1, 1] == outerSensor:
                        self.checkPoint()
                elif [0, 1] == innerSensor:
                    self.nudgeRight()
                elif [1, 0] == innerSensor:
                    self.nudgeLeft()
                elif [0, 0] == innerSensor:
                    self.reverse()
                self.publish()
                print(f"self.x: {self.x}, self.z: {self.z}")
                print("grnusogunsrgrs")

        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            # stop moving
            self.stopbot()


def main(args=None):
    rclpy.init(args=args)
    GPIO_setup()
    nav = linerMover()
    nav.mover()
    print("closing")
    nav.destroy_node()
    rclpy.shutdown()
    GPIO.cleanup()


if __name__ == "__main__":
    main()
