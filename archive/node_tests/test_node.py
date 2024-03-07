import rclpy
from geometry_msgs.msg import Quaternion

def callback(data):
    x = data.x
    y = data.y
    z = data.z
    w = data.w
    print('Quaternion: ', x, y, z, w)
    pass

rclpy.init()
node = rclpy.create_node('test_node')
publisher = node.create_publisher(Quaternion, 'test_publisher_topic', 10)
subscriber = node.create_subscription(Quaternion, 'test_subscription', callback,
10)

Q = Quaternion()
Q.x = 1.0
Q.y = 2.0
Q.z = 5.0
Q.w = 3.0

publisher.publish(Q)

rclpy.spin(node)
