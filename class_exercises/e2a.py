import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import time

class LaserReader(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create subscription to track lidar
        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        self.laser_range = np.array([])

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def pick_direction(self):
        # self.get_logger().info('In pick_direction')
        if self.laser_range.size != 0:
            #a use nanargmax as there are nan's in laser_range added to replace 0's
            is_close = np.nanargmin(self.laser_range) <= 1.0
            self.get_logger().info('Object is within 1m: ' + is_close)
        else:
            is_close = False;
            self.get_logger().info('No data!')

        time.sleep(1)
        self.publisher_.publish(is_close)

def main(args=None):
    rclpy.init(args=args)

    laser_reader = LaserReader()

    # create matplotlib figure
    # plt.ion()
    # plt.show()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    laser_reader.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
