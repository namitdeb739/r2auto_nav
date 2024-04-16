from random import randint
import rclpy
from std_msgs.msg import Bool
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import math
import sys
import time
import math

# Constants
rotate_change = 0.8
speed = 0.22
occupancy_bins = [-1, 0, 100, 101]
stop_distance = 0.50
front_angle = 30
scan_file = "lidar.txt"
map_file = "map.txt"


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z  # in radians

class Explore(Node):
    def __init__(self):
        super().__init__("explore")

        # Publisher to move the TurtleBot
        self.publisher_twist = self.create_publisher(Twist, "cmd_vel", 10)
        self.get_logger().info("Created publisher for twist")

        # Publisher to kill the Line Follower
        self.publisher_kill_line = self.create_publisher(Bool, "kill_line", 10)
        self.get_logger().info("Created publisher for killing line follower")

        # Subscriber to track odometry
        self.subscription_odometry = self.create_subscription(
            Odometry, "odom", self.odometry_callback, 10
        )
        self.subscription_odometry
        self.get_logger().info("Created subscriber for odometry")

        # Initialise odometry fields
        self.x, self.y = 0.0, 0.0
        self.roll, self.pitch, self.yaw = 0.0, 0.0, 0.0

        # Subscriber to track occupancy
        self.subscription_occupancy = self.create_subscription(
            OccupancyGrid,
            "map",
            self.occupancy_callback,
            qos_profile_sensor_data,
        )
        self.subscription_occupancy
        self.get_logger().info("Created subscriber for occupancy")

        # Initialise occupancy fields
        self.occupancy_data = np.empty((0,0))
        self.is_fully_explored = False

        # Subscriber to track checkpoint
        self.subscription_checkpoint = self.create_subscription(
            Bool, "checkpoint", self.checkpoint_callback, 10
        )
        self.subscription_checkpoint
        self.get_logger().info("Created subscriber for checkpoint")
        self.checkpoint = False

        # Subscriber to track lidar
        self.subscription_lidar = self.create_subscription(
            LaserScan, "scan", self.scan_callback, qos_profile_sensor_data
        )
        self.subscription_lidar
        self.get_logger().info("Created subscriber for lidar")
        self.laser_range = np.array([])

    def odometry_callback(self, msg):
        self.get_logger().info("In odometry_callback")
        orientation_quat = msg.pose.pose.orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            orientation_quat.x,
            orientation_quat.y,
            orientation_quat.z,
            orientation_quat.w,
        )

    def occupancy_callback(self, msg):
        self.get_logger().info("In occupancy_callback")
        
        # Create numpy array
        msg_data = np.array(msg.data)
        
        # Compute histogram to identify percent of bins with -1
        occ_counts = np.histogram(msg_data, occupancy_bins)
        
        # Calculate total number of bins
        total_bins = msg.info.width * msg.info.height
        self.get_logger().info(
            "Unmapped: %i Unoccupied: %i Occupied: %i Total: %i"
            % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)
        )

        # Make msg_data go from 0 instead of -1, reshape into 2D
        temp = msg_data + 1
        
        # Reshape to 2D array using column order
        self.occupancy_data = np.uint8(
            temp.reshape(msg.info.height, msg.info.width, order="F")
        )

        self.is_fully_explored = np.all(self.occupancy_data != 0)

    def checkpoint_callback(self, msg):
        self.get_logger().info("In checkpoint_callback")
        
        self.checkpoint = msg.data
        self.get_logger().info(f"Checkpoint: {self.checkpoint}")

    def scan_callback(self, msg):
        self.get_logger().info("In scan_callback")
        
        # Create numpy array
        self.laser_range = np.array(msg.ranges)
        
        # Print to file
        np.savetxt(scan_file, self.laser_range)
        
        # Replace 0's with nan
        self.laser_range[self.laser_range == 0] = np.nan

    def rotate(self, theta):
        self.get_logger().info("In rotate")

        # Create Twist object
        twist = Twist()

        current_yaw = self.yaw
        self.get_logger().info("Current: %f" % math.degrees(current_yaw))

        # Calculate desired yaw
        target_yaw = (current_yaw + math.radians(theta)) % (2 * math.pi)

        # Set the angular speed in the z-axis
        twist.angular.z = rotate_change if target_yaw > current_yaw else -rotate_change
        self.publisher_twist.publish(twist)

        while target_yaw - current_yaw > 0.1:
            rclpy.spin_once(self)
            current_yaw = self.yaw
            self.get_logger().info("Target: %f, Current: %f" % (math.degrees(target_yaw), math.degrees(current_yaw)))

        # Stop the robot after reaching the target
        twist.angular.z = 0.0
        self.publisher_twist.publish(twist)

    def go_to_furthest_point(self):
        if not self.checkpoint:
            return

        self.get_logger().info("In go_to_furthest_point")

        # Kill line follower code
        kill_line = Bool()
        kill_line.data = True
        self.publisher_kill_line.publish(kill_line)

        if self.laser_range.size != 0:
            # Use nanargmax as there are NaNs in laser_range added to replace  0's
            theta = np.nanargmax(self.laser_range)
            self.get_logger().info(
                "Picked direction: %d %f m" % (theta, self.laser_range[theta])
            )
        else:
            theta = 0
            self.get_logger().info("No data!")

        # Rotate to that direction
        self.rotate(float(theta))

        # Start moving
        self.get_logger().info("Start moving")
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0

        self.publisher_twist.publish(twist)

    def stop(self):
        self.get_logger().info("In stop")

        # Publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        time.sleep(1)
        self.publisher_twist.publish(twist)

    def mover(self):
        try:
            self.get_logger().info("In mover")

            while not self.checkpoint:
                # Wait for checkpoint
                rclpy.spin_once(self)

            while not self.is_fully_explored:
                # Allow callbacks to update variables
                rclpy.spin_once(self)

                self.go_to_furthest_point()

                # Check if any values within the front range are too close
                if np.nanmin(np.concatenate((self.laser_range[:front_angle], self.laser_range[-front_angle:]))) < stop_distance:
                    self.get_logger().info('Obstacle detected within stop distance. Stopping.')
                    self.stop()

                    time.sleep(5)
                    self.rotate(float(randint(-90, 90)))
                    
        except Exception as e:
            print(e)

        # Ctrl-c detected
        finally:
            # Stop moving
            self.stop()


def main(args=None):
    rclpy.init(args=args)
    explore = Explore()
    explore.mover()
    explore.destroy_node()
    rclpy.shutdown()
    sys.exit()


if __name__ == "__main__":
    explore = Explore()
    explore.main()
