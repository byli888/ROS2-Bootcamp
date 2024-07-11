import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose

class VacuumCleaner(Node):
    def __init__(self):
        super().__init__('vacuum_cleaner')
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            'turtle1/pose',
            self.pose_callback,
            10)
        self.pose = Pose()
        self.timer = self.create_timer(0.1, self.move_turtle)
        self.target_shape = 'circle'  # 'circle' or 'box'
        self.radius = 10
        self.box_size = 10
        self.spiral_param = 0.5
        self.angle = 0.0

    def pose_callback(self, msg):
        self.get_logger().info(f'Pose received: x={msg.x}, y={msg.y}, theta={msg.theta}')
        self.pose = msg

    def move_turtle(self):
        twist = Twist()
        self.get_logger().info(f'Moving turtle. Current pose: x={self.pose.x}, y={self.pose.y}, theta={self.pose.theta}')

        # Check if the turtle hits the boundary or the radius is larger than 11
        if self.pose.x <= 0.5 or self.pose.x >= 11 or self.pose.y <= 0.5 or self.pose.y >= 11 or self.spiral_param > 11:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            if self.spiral_param > 11.5:
                self.get_logger().info('Radius exceeded 11: stopping turtle')
            else:
                self.get_logger().info('Boundary hit: stopping turtle')
            rclpy.shutdown()
            return

        if self.target_shape == 'circle':
            twist.linear.x = self.spiral_param
            twist.angular.z = 1.0
            self.spiral_param += 0.01
        elif self.target_shape == 'box':
            # Implement box vacuum cleaning logic
            if self.pose.x > self.box_size and self.pose.y > self.box_size:
                twist.linear.x = 0.0
                twist.angular.z = 1.57
            elif self.pose.x < self.box_size and self.pose.y > 0:
                twist.linear.x = self.spiral_param
                twist.angular.z = 0.0
            elif self.pose.x < self.box_size and self.pose.y < self.box_size:
                twist.linear.x = 0.0
                twist.angular.z = 1.57

        self.publisher_.publish(twist)
        self.get_logger().info(f'Published Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}')

def main(args=None):
    rclpy.init(args=args)
    vacuum_cleaner = VacuumCleaner()
    rclpy.spin(vacuum_cleaner)
    vacuum_cleaner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
