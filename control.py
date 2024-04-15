import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import heapq
import math
import random
import yaml
import scipy.interpolate as si
import sys
import threading
import time

with open("./config/params.yaml", 'r') as file:
    params = yaml.load(file, Loader=yaml.FullLoader)

lookahead_distance = params["lookahead_distance"]
speed = params["speed"]
expansion_size = params["expansion_size"]
target_error = params["target_error"]
robot_security_radius = params["robot_r"]

path_global = 0


def euler_from_quaternion(x, y, z, w):
    # t0 = +2.0 * (w * x + y * z)
    # t1 = +1.0 - 2.0 * (x * x + y * y)
    # roll_x = math.atan2(t0, t1)
    # t2 = +2.0 * (w * y - z * x)
    # t2 = +1.0 if t2 > +1.0 else t2
    # t2 = -1.0 if t2 < -1.0 else t2
    # pitch_y = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z


def heuristic(a, b):
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)


def astar(array, start, goal):
    neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1),
                 (-1, -1)]
    close_set = set()
    came_from = {}
    gscore = {start: 0}
    fscore = {start: heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            data = data + [start]
            data = data[::-1]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            tentative_g_score = gscore[current] + heuristic(current, neighbor)
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    # array bound y walls
                    continue
            else:
                # array bound x walls
                continue
            if (neighbor in close_set) and \
               (tentative_g_score >= gscore.get(neighbor, 0)):
                continue
            if (tentative_g_score < gscore.get(neighbor, 0)) or \
               (neighbor not in [i[1] for i in oheap]):
                came_from[neighbor] = current
                gscore[neighbor] = tentative_g_score
                fscore[neighbor] = tentative_g_score + \
                    heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    # If no path to goal was found, return closest path to goal
    if goal not in came_from:
        closest_node = None
        closest_dist = float('inf')
        for node in close_set:
            dist = heuristic(node, goal)
            if dist < closest_dist:
                closest_node = node
                closest_dist = dist
        if closest_node is not None:
            data = []
            while closest_node in came_from:
                data.append(closest_node)
                closest_node = came_from[closest_node]
            data = data + [start]
            data = data[::-1]
            return data
    return False


def bspline_planning(array, sn):
    try:
        array = np.array(array)
        x = array[:, 0]
        y = array[:, 1]
        N = 2
        t = range(len(x))
        x_tup = si.splrep(t, x, k=N)
        y_tup = si.splrep(t, y, k=N)

        x_list = list(x_tup)
        xl = x.tolist()
        x_list[1] = xl + [0.0, 0.0, 0.0, 0.0]

        y_list = list(y_tup)
        yl = y.tolist()
        y_list[1] = yl + [0.0, 0.0, 0.0, 0.0]

        ipl_t = np.linspace(0.0, len(x) - 1, sn)
        rx = si.splev(ipl_t, x_list)
        ry = si.splev(ipl_t, y_list)
        path = [(rx[i], ry[i]) for i in range(len(rx))]
    except (ValueError, TypeError):
        path = array
    return path


def pure_pursuit(current_x, current_y, current_heading, path, index):
    global lookahead_distance
    closest_point = None
    v = speed
    for i in range(index, len(path)):
        x = path[i][0]
        y = path[i][1]
        distance = math.hypot(current_x - x, current_y - y)
        if lookahead_distance < distance:
            closest_point = (x, y)
            print("Closest point: (%s, %s): " % (x, y))
            index = i
            break
    if closest_point is not None:
        target_heading = math.atan2(closest_point[1] - current_y,
                                    closest_point[0] - current_x)
        desired_steering_angle = target_heading - current_heading
    else:
        target_heading = math.atan2(path[-1][1] - current_y,
                                    path[-1][0] - current_x)
        desired_steering_angle = target_heading - current_heading
        index = len(path) - 1
    if desired_steering_angle > math.pi:
        desired_steering_angle -= 2 * math.pi
    elif desired_steering_angle < -math.pi:
        desired_steering_angle += 2 * math.pi
    if (desired_steering_angle > math.pi / 6) or \
       (desired_steering_angle < -math.pi / 6):
        sign = 1 if desired_steering_angle > 0 else -1
        desired_steering_angle = sign * math.pi / 4
        v = 0.0
    return v, desired_steering_angle, index


def frontier_B(matrix):
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            if matrix[i][j] == 0.0:
                if i > 0 and matrix[i - 1][j] < 0:
                    matrix[i][j] = 2
                elif i < len(matrix) - 1 and matrix[i + 1][j] < 0:
                    matrix[i][j] = 2
                elif j > 0 and matrix[i][j - 1] < 0:
                    matrix[i][j] = 2
                elif j < len(matrix[i]) - 1 and matrix[i][j + 1] < 0:
                    matrix[i][j] = 2
    return matrix


def assign_groups(matrix):
    group = 1
    groups = {}
    for i in range(len(matrix)):
        for j in range(len(matrix[0])):
            if matrix[i][j] == 2:
                group = dfs(matrix, i, j, group, groups)
    return matrix, groups


def dfs(matrix, i, j, group, groups):
    if i < 0 or i >= len(matrix) or j < 0 or j >= len(matrix[0]):
        return group
    if matrix[i][j] != 2:
        return group
    if group in groups:
        groups[group].append((i, j))
    else:
        groups[group] = [(i, j)]
    matrix[i][j] = 0
    dfs(matrix, i + 1, j, group, groups)
    dfs(matrix, i - 1, j, group, groups)
    dfs(matrix, i, j + 1, group, groups)
    dfs(matrix, i, j - 1, group, groups)
    dfs(matrix, i + 1, j + 1, group, groups)  # Lower right diagonal
    dfs(matrix, i - 1, j - 1, group, groups)  # Upper left diagonal
    dfs(matrix, i - 1, j + 1, group, groups)  # Upper right diagonal
    dfs(matrix, i + 1, j - 1, group, groups)  # Lower left diagonal
    return group + 1


def f_groups(groups):
    # print("Groups: %s" % (groups))
    sorted_groups = sorted(groups.items(), key=lambda x: len(x[1]),
                           reverse=True)
    # print("Sorted groups: %s" % (sorted_groups))
    top_five_groups = [g for g in sorted_groups[:5] if len(g[1]) >= 3]
    # print("T5 groups: %s" % (top_five_groups))
    return top_five_groups


def calculate_centroid(x_coords, y_coords):
    n = len(x_coords)
    sum_x = sum(x_coords)
    sum_y = sum(y_coords)
    mean_x = sum_x / n
    mean_y = sum_y / n
    centroid = (int(mean_x), int(mean_y))
    print("Centroid: (%s, %s)" % (int(mean_x), int(mean_y)))
    return centroid


def find_closest_group(matrix, groups, current, resolution, originX, originY):
    targetP = None
    distances = []
    paths = []
    score = []
    max_score = -1  # Max score index
    for i in range(len(groups)):
        middle = calculate_centroid([p[0] for p in groups[i][1]],
                                    [p[1] for p in groups[i][1]])
        # print("Middle: %s" % (middle))
        path = astar(matrix, current, middle)
        path = [(p[1] * resolution + originX, p[0] * resolution + originY)
                for p in path]
        total_distance = path_length(path)
        distances.append(total_distance)
        paths.append(path)
    for i in range(len(distances)):
        if distances[i] == 0:
            score.append(0)
        else:
            score.append(len(groups[i][1]) / distances[i])
    for i in range(len(distances)):
        if distances[i] > target_error * 3:
            if max_score == -1 or score[i] > score[max_score]:
                max_score = i
    if max_score != -1:
        targetP = paths[max_score]
    else:
        # Selects a random point as the target if it is closer than
        # target_error * 2. This allows the robot to get out of some
        # situations.
        index = random.randint(0, len(groups) - 1)
        target = groups[index][1]
        target = target[random.randint(0, len(target) - 1)]
        path = astar(matrix, current, target)
        targetP = [(p[1] * resolution + originX,
                    p[0] * resolution + originY) for p in path]
    return targetP


def path_length(path):
    for i in range(len(path)):
        path[i] = (path[i][0], path[i][1])
        points = np.array(path)
    differences = np.diff(points, axis=0)
    distances = np.hypot(differences[:, 0], differences[:, 1])
    total_distance = np.sum(distances)
    return total_distance


def costmap(data, width, height, resolution):
    data = np.array(data).reshape(height, width)
    wall = np.where(data == 100)
    for i in range(-expansion_size, expansion_size + 1):
        for j in range(-expansion_size, expansion_size + 1):
            if i == 0 and j == 0:
                continue
            x = wall[0] + i
            y = wall[1] + j
            x = np.clip(x, 0, height - 1)
            y = np.clip(y, 0, width - 1)
            data[x, y] = 100
    data = data * resolution
    return data


def exploration(data, width, height, resolution, column, row, originX,
                originY):
    global path_global  # Global variable
    data = costmap(data, width, height, resolution)  # Expand the barriers
    data[row][column] = 0  # Robot current location
    data[data > 5] = 1  # Those with score of 0 are a go to place \
    # those with a score of 100 are a definite obstacle.
    data = frontier_B(data)  # Find node points
    data, groups = assign_groups(data)  # Group node points
    # print("\nData: \n%s\nGroups: %s\n" % (data, groups))
    groups = f_groups(groups)  # Sort g roups from smallest to largest \
    # and take the top 5
    print(("Groups: %s\n" % (groups)[:20]))
    if len(groups) == 0:  # If there is no group, discovery is complete
        path = -1
    else:  # If there is a group, find the closest group
        data[data < 0] = 1  # -0.05 is an unknown location, \
        # mark it unvisitable. 0 = can go, 1 = cannot go.
        path = find_closest_group(data, groups, (row, column), resolution,
                                  originX, originY)  # Find closest group
        print("Path: %s" % (path)[:40])
        if path is not None:  # If there is a path, fix it with B-spline
            path = bspline_planning(path, len(path) * 5)
            print("Path: %s" % (path)[:40])
        else:
            path = -1
    path_global = path
    # print("Path found") if isinstance(path_global, int) else print("Path not found")
    return


def localControl(scan):
    # self.get_logger().info("LOCAL CONTROL")
    v = None
    w = None
    scan_range = len(scan)
    for i in range(int((1 / 6) * scan_range)):
        if scan[i] < robot_security_radius:
            v = 0.2
            w = - math.pi / 4
            break
    if v is None:
        for i in range(int((5 / 6) * scan_range), int(scan_range)):
            if scan[i] < robot_security_radius:
                v = 0.2
                w = math.pi / 4
                break
    return v, w


class NavigationControl(Node):
    def __init__(self):
        # self.get_logger().info("INITIALISING")
        super().__init__('Exploration')
        self.subscription = self.create_subscription(
                OccupancyGrid, 'map', self.map_callback,
                qos_profile_sensor_data)
        # self.get_logger().info('Created map subscriber')

        self.subscription = self.create_subscription(
                Odometry, 'odom', self.odom_callback, 10)
        # self.get_logger().info('Created odom subscriber')

        self.subscription = self.create_subscription(
                LaserScan, 'scan', self.scan_callback,
                qos_profile_sensor_data)
        # self.get_logger().info('Created scan subscriber')
        
        self.subscription = self.create_subscription(
                Bool, 'checkpoint', self.checkpoint_callback, 10)

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.get_logger().info("DISCOVERY MODE ACTIVE")

        self.discovery = True
        # Runs the discovery function as a thread.
        threading.Thread(target=self.exp).start()

    def exp(self):
        self.get_logger().info("STARTING EXPLORATION LOOP")
        twist = Twist()
        # self.get_logger().info("Twist: %s" % (twist))
        while True:  # Wait until sensor data arrives.
            has_map_data = hasattr(self, 'map_data')
            has_odom_data = hasattr(self, 'odom_data')
            has_scan_data = hasattr(self, 'scan_data')
            # self.get_logger().info("Has map: %s, Has odom: %s, Has scan: %s"
            #                       % (has_map_data, has_odom_data,
            #                           has_scan_data))
            if not has_map_data or not has_odom_data or not has_scan_data:
                time.sleep(0.1)
                continue
            if self.discovery is True:
                if isinstance(path_global, int) and path_global == 0:
                    column = int((self.x - self.originX) / self.resolution)
                    row = int((self.y - self.originY) / self.resolution)
                    self.get_logger().info("Map size (R, C): (%s, %s)" % (row, column))
                    exploration(self.data, self.width, self.height,
                                self.resolution, column, row,
                                self.originX, self.originY)
                    self.path = path_global
                    # self.get_logger().info("Self path: %s" % (self.path))
                else:
                    self.path = path_global
                    # self.get_logger().info("Self path: %s" % (self.path))
                if isinstance(self.path, int) and self.path == -1:
                    self.get_logger().info("EXPLORATION COMPLETE")
                    twist.linear.x = 0
                    twist.angular.z = 0
                    self.publisher.publish(twist)
                    sys.exit(0)
                self.c = int((self.path[-1][0] - self.originX)
                             / self.resolution)
                self.r = int((self.path[-1][1] - self.originY)
                             / self.resolution)
                print("Self (r, c): (%s, %s)" % (self.r, self.c))
                self.discovery = False
                self.i = 0
                self.get_logger().info("NEW TARGET SET")
                t = path_length(self.path) / speed
                self.get_logger().info("Path length: %s, Time: %s" %
                        (path_length(self.path), t))
                # According to x = v * t, 0.2seconds is subtracted
                # from the calculated time. After t time, the
                # discovery function is run.
                t = t - 0.2
                # Activates the discovery function shortly before the
                # target.
                self.t = threading.Timer(t, self.target_callback)
                self.t.start()

            # Route follow block start.
            else:
                v, w = localControl(self.scan)
                if v is None:
                    v, w, self.i = pure_pursuit(self.x, self.y, self.yaw,
                                                self.path, self.i)
                if ((abs(self.x - self.path[-1][0]) < target_error) and
                        (abs(self.y - self.path[-1][1]) < target_error)):
                    v = 0.0
                    w = 0.0
                    self.discovery = True
                    self.get_logger().info("TARGET REACHED")
                    self.t.join()  # Wait until the thread finishes.
                twist.linear.x = v
                twist.angular.z = w
                self.publisher.publish(twist)
                time.sleep(0.5)
            # Route tracking block ends.

    def target_callback(self):
        exploration(self.data, self.width, self.height, self.resolution,
                    self.c, self.r, self.originX, self.originY)
        # self.get_logger().info("Data: %s\nHeight, Width: %s, %s"
        #                       % (self.data, self.height, self.width))

    def checkpoint_callback(self, msg):
        self.checkpoint = msg.data
        self.get_logger().info(f"Checkpoint: {self.checkpoint}")

    def scan_callback(self, msg):
        # self.get_logger().info(msg)
        self.scan_data = msg
        self.scan = msg.ranges

    def map_callback(self, msg):
        # self.get_logger().info(msg)
        self.map_data = msg
        self.resolution = self.map_data.info.resolution
        self.originX = self.map_data.info.origin.position.x
        self.originY = self.map_data.info.origin.position.y
        self.width = self.map_data.info.width
        self.height = self.map_data.info.height
        self.data = self.map_data.data
        # self.get_logger().info("Data: %s" % (self.data))
        # self.get_logger().info("Origin: (%s, %s)"
        #                       % (self.originX, self.originY))
        # self.get_logger().info("Map height: %s, Map width: %s, Reso: %s"
        #                       % (self.height, self.width, self.resolution))

    def odom_callback(self, msg):
        self.odom_data = msg
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        self.yaw = euler_from_quaternion(msg.pose.pose.orientation.x,
                                         msg.pose.pose.orientation.y,
                                         msg.pose.pose.orientation.z,
                                         msg.pose.pose.orientation.w)


def main(args=None):
    rclpy.init(args=args)
    navigation_control = NavigationControl()
    rclpy.spin(navigation_control)
    navigation_control.destroy_node()
    rclpy.shutdown()
    sys.exit(0)


if __name__ == '__main__':
    main()
