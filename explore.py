from queue import Queue
from tkinter import N
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
import cmath

# Constants
rotate_change = 0.5
speed = 0.22
occupancy_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle, front_angle + 1, 1)
radius = 10
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
        self.x, self.y = None, None
        self.current_position = (None, None)
        self.roll, self.pitch, self.yaw = None, None, None

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
        self.occupancy_data = None

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

        self.visited = set()
        self.unvisited = Queue()
        self.current_position = None  # Keep track of the current position


    def odometry_callback(self, msg):
        # self.get_logger().info("In odometry_callback")
        orientation_quat = msg.pose.pose.orientation
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.current_position = (self.x, self.y)
        self.visited.add(self.current_position)

        self.roll, self.pitch, self.yaw = euler_from_quaternion(
            orientation_quat.x,
            orientation_quat.y,
            orientation_quat.z,
            orientation_quat.w,
        )

    def occupancy_callback(self, msg):
        # self.get_logger().info("In occupancy_callback")
        
        # Create numpy array
        msg_data = np.array(msg.data)
        
        # Compute histogram to identify percent of bins with -1
        # occ_counts = np.histogram(msg_data, occupancy_bins)
        
        # Calculate total number of bins
        # total_bins = msg.info.width * msg.info.height
        # self.get_logger().info(
        #     "Unmapped: %i Unoccupied: %i Occupied: %i Total: %i"
        #     % (occ_counts[0][0], occ_counts[0][1], occ_counts[0][2], total_bins)
        # )

        # Make msg_data go from 0 instead of -1, reshape into 2D
        temp = msg_data + 1
        
        # Reshape to 2D array using column order
        self.occupancy_data = np.uint8(
            temp.reshape(msg.info.height, msg.info.width, order="F")
        )
        self.occupancy_data = np.uint8(temp.reshape(msg.info.height, msg.info.width))

        # Add the new unvisited locations to the unvisited queue

        for i in range(msg.info.height):
            for j in range(msg.info.width):
                if self.occupancy_data[i, j] == 0:  # 0 represents unoccupied space
                    unvisited_position = (i, j)
                    if unvisited_position not in self.visited:
                        self.unvisited.put(unvisited_position)

    def checkpoint_callback(self, msg):
        self.get_logger().info("In checkpoint_callback")
        
        self.checkpoint = msg.data
        self.get_logger().info(f"Checkpoint: {self.checkpoint}")

    def scan_callback(self, msg):
        # self.get_logger().info("In scan_callback")
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

        # Get current yaw angle
        current_yaw = self.yaw
        self.get_logger().info("Current: %f" % math.degrees(current_yaw))

        # Use complex numbers to avoid problems when the angles go from
        complex_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))

        # Calculate desired yaw
        target_yaw = current_yaw + math.radians(theta)

        # Convert to complex notation
        complex_target_yaw = complex(math.cos(target_yaw), math.sin(target_yaw))
        self.get_logger().info(
            "Desired: %f" % math.degrees(cmath.phase(complex_target_yaw))
        )

        # Divide the two complex numbers to get the change in direction
        complex_change = complex_target_yaw / complex_yaw

        # Get the sign of the imaginary component to figure out which way we have to turn
        complex_change_direction = np.sign(complex_change.imag)

        # Set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0

        # Set the direction to rotate
        twist.angular.z = complex_change_direction * rotate_change

        # Start rotation
        self.publisher_twist.publish(twist)

        # We will use the complex_direction_difference variable to see if we can stop rotating
        complex_direction_difference = complex_change_direction
        self.get_logger().info(
            "complex_change_direction: %f complex_direction_difference: %f"
            % (complex_change_direction, complex_direction_difference)
        )

        # If the rotation direction was 1.0, then we will want to stop when the complex_direction_difference becomes -1.0, and vice versa
        while complex_change_direction * complex_direction_difference > 0:
            # Allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw

            # Convert the current yaw to complex form
            complex_yaw = complex(math.cos(current_yaw), math.sin(current_yaw))
            self.get_logger().info("Current Yaw: %f" % math.degrees(current_yaw))

            # Get difference in angle between current and target
            complex_change = complex_target_yaw / complex_yaw

            # Get the sign to see if we can stop
            complex_direction_difference = np.sign(complex_change.imag)

            self.get_logger().info(
                "complex_change_direction: %f complex_direction_difference: %f"
                % (complex_change_direction, complex_direction_difference)
            )

        self.get_logger().info("End Yaw: %f" % math.degrees(current_yaw))

        # Set the rotation speed to 0 and stop the rotation
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

        time.sleep(1)
        self.publisher_twist.publish(twist)

    def stop(self):
        self.get_logger().info("In stop")

        # Publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0

        time.sleep(1)
        self.publisher_twist.publish(twist)

    def all_surrounding_visited(self):
        self.get_logger().info("In all_surrounding_visited")

        # Check all surrounding points
        for (x, y) in [
            (1, 0),
            (0, 1),
            (-1, 0),
            (0, -1),
            (1, 1),
            (-1, -1),
            (1, -1),
            (-1, 1),
        ]:
            if (self.x + radius * x, self.y + radius * y) not in self.visited:
                return False
        return True
    
    def go_to_nearest_unvisited(self):        
        self.get_logger().info("In go_to_nearest_unvisited")

        # Check if there are any unvisited points
        if not self.unvisited.empty():
            nearest_unvisited = self.unvisited.get()
            self.go_to_point(nearest_unvisited)
    
    def go_to_point(self, point):
        self.get_logger().info("In go_to_point")

        # Get current position
        x = self.x
        y = self.y

        # Calculate angle to point
        angle = math.atan2(point[1] - y, point[0] - x)

        # Rotate to that direction
        self.rotate(angle)

        # Start moving
        twist = Twist()
        twist.linear.x = speed
        twist.angular.z = 0.0

        time.sleep(1)
        self.publisher_twist.publish(twist)

    def mover(self):
        try:
            self.get_logger().info("In mover")
            
            # Allow the callback functions to run
            rclpy.spin_once(self)
            
            # Pick direction point on algorithm and start moving towards it
            self.go_to_furthest_point()

            while rclpy.ok():
                # Allow the callback functions to run
                rclpy.spin_once(self)

                if not self.checkpoint:
                    continue

                if self.laser_range.size != 0:
                    # Check distances in front of TurtleBot and find values less
                    # than stop_distance
                    foward_distances = (
                        self.laser_range[front_angles] < float(stop_distance)
                    ).nonzero()
                    self.get_logger().info("Distances: %s" % str(foward_distances))

                    # If the list is not empty
                    if len(foward_distances[0]) > 0:
                        # Stop moving
                        self.stop()


                        # Check if there are any unvisited points
                        if self.all_surrounding_visited():
                            self.go_to_nearest_unvisited()

                        # Choose new point and go towards it
                        self.go_to_furthest_point()
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
