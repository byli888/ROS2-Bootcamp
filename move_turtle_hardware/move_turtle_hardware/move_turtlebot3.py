#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import transforms3d
import math

class MoveTurtleBot3(Node):

    def __init__(self):
        super().__init__('move_turtlebot3')
        self.declare_parameter('goal_x', 2.0)
        self.declare_parameter('goal_y', 2.0)
        self.declare_parameter('goal_theta', 0.0)
        
        self.goal_x = self.get_parameter('goal_x').get_parameter_value().double_value
        self.goal_y = self.get_parameter('goal_y').get_parameter_value().double_value
        self.goal_theta = self.get_parameter('goal_theta').get_parameter_value().double_value
        
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.goal_reached = False

    def odom_callback(self, msg):
        self.current_x = msg.pose.pose.position.x
        self.current_y = msg.pose.pose.position.y
        orientation_q = msg.pose.pose.orientation
        _, _, self.current_theta = transforms3d.euler.quat2euler(
            [orientation_q.w, orientation_q.x, orientation_q.y, orientation_q.z])

    def timer_callback(self):
        twist = Twist()
        distance = math.sqrt((self.goal_x - self.current_x) ** 2 + (self.goal_y - self.current_y) ** 2)
        angle_to_goal = math.atan2(self.goal_y - self.current_y, self.goal_x - self.current_x)

        if distance > 0.1:
            twist.linear.x = 0.2
            twist.angular.z = 0.5 * (angle_to_goal - self.current_theta)
            self.goal_reached = False
        else:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            if not self.goal_reached:
                self.get_logger().info('Goal reached!')
                self.goal_reached = True

        self.publisher_.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    move_turtlebot3 = MoveTurtleBot3()
    rclpy.spin(move_turtlebot3)
    move_turtlebot3.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
