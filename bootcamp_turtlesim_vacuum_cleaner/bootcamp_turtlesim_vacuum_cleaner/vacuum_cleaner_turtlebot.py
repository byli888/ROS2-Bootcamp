import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

class VacuumCleaner(Node):
    def __init__(self):
        super().__init__('vacuum_cleaner')
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.pose = Odometry()
        self.timer = self.create_timer(0.1, self.move_turtlebot)
        self.target_shape = 'circle'  # 'circle' or 'box'
        self.spiral_param = 0.5

    def odom_callback(self, msg):
        self.get_logger().info(f'Odom received: x={msg.pose.pose.position.x}, y={msg.pose.pose.position.y}, theta={msg.pose.pose.orientation.z}')
        self.pose = msg

    def move_turtlebot(self):
        twist = Twist()
        x = self.pose.pose.pose.position.x
        y = self.pose.pose.pose.position.y
        self.get_logger().info(f'Moving TurtleBot. Current pose: x={x}, y={y}')

        # Check if the TurtleBot hits the boundary or the radius is larger than 11
        #if self.spiral_param > 11:
        if x > 6:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
          #  if self.spiral_param > 11:
            if x > 6:
                self.get_logger().info('Radius exceeded 6: stopping TurtleBot')
            else:
                self.get_logger().info('Boundary hit: stopping TurtleBot')
            rclpy.shutdown()
            return

        if self.target_shape == 'circle':
            twist.linear.x = self.spiral_param
            twist.angular.z = 1.0
            self.spiral_param += 0.01
        elif self.target_shape == 'box':
            # Implement box vacuum cleaning logic
            if x > self.box_size and y > self.box_size:
                twist.linear.x = 0.0
                twist.angular.z = 1.57
            elif x < self.box_size and y > 0:
                twist.linear.x = self.spiral_param
                twist.angular.z = 0.0
            elif x < self.box_size and y < self.box_size:
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
