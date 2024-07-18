import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
import random

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.pose_subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.update_pose,
            10)
        self.pose = Pose()
        self.timer = self.create_timer(0.1, self.move_turtle)
        self.goal_x = random.uniform(0, 11)
        self.goal_y = random.uniform(0, 11)
        self.goal_theta = random.uniform(-3.14, 3.14)
        self.reached_goal = False

    def update_pose(self, msg):
        self.pose = msg

    def move_turtle(self):
        if self.reached_goal:
            self.goal_x = random.uniform(0, 11)
            self.goal_y = random.uniform(0, 11)
            self.goal_theta = random.uniform(-3.14, 3.14)
            self.reached_goal = False
            self.get_logger().info(f'New goal: x={self.goal_x}, y={self.goal_y}, theta={self.goal_theta}')
            self.get_logger().info(f'Reached goal: x={self.goal_x}, y={self.goal_y}, theta={self.goal_theta}')
        distance = math.sqrt((self.goal_x - self.pose.x) ** 2 + (self.goal_y - self.pose.y) ** 2)
        angle_to_goal = math.atan2(self.goal_y - self.pose.y, self.goal_x - self.pose.x)
        angle_difference = angle_to_goal - self.pose.theta

        move_cmd = Twist()
        if distance > 0.1:
            move_cmd.linear.x = 1.5 * distance
            move_cmd.angular.z = 6 * angle_difference
        else:
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 1.5 * (self.goal_theta - self.pose.theta)

            if abs(self.goal_theta - self.pose.theta) < 0.1:
                self.reached_goal = True

        self.publisher.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
