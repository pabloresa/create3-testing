import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from geometry_msgs.msg import Twist
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

import numpy as np

class WallPub(Node):

    def __init__(self):
        super().__init__('Wall_Follower_Node')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_capture,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        timer_period = 0.5
        self.msg = Twist()
        self.laser_wall = 0
        self.laser_front = 0
        self.timer = self.create_timer(timer_period, self.pub)
    def laser_capture(self, msg):
        self.laser_wall = msg.ranges[90]
        self.laser_front = msg.ranges[0]
    def pub(self):
        if(self.laser_wall > 0.5 and self.laser_front > 0.5):
            self.msg.linear.x = 0.05
            self.msg.angular.z = 0.03
        elif(self.laser_wall < 0.4 and self.laser_front > 0.5):
            self.msg.linear.x = 0.03
            self.msg.angular.z = -0.04
        elif(self.laser_wall >= 0.4 and self.laser_wall <= 0.5 and self.laser_front > 0.5):
            self.msg.linear.x = 0.05
            self.msg.angular.z = 0.0
        else:
            self.msg.linear.x = 0.01
            self.msg.angular.z = -1.0
        self.get_logger().info('Publishing Laser 90: "%s"' % self.laser_wall)
        self.publisher_.publish(self.msg)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    wall_publisher = WallPub()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(wall_publisher)
    # Explicity destroys the node
    wall_publisher.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()
