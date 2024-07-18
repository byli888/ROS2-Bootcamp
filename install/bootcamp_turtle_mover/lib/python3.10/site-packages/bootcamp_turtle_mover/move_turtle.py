import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import random

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(10.0, self.move_turtle)
        self.x, self.y, self.theta = 5.5, 5.5, 0 # Initial rough center position
    
    def move_turtle(self):
        # Generate random goal position and orientation
        goal_x = random.uniform(0, 11) # Turtlesim boundaries for x
        goal_y = random.uniform(0, 11) # Turtlesim boundaries for y
        goal_theta = random.uniform(-3.14, 3.14) # Full range of orientation

        # Log the new goal
        self.get_logger().info(f'New goal: x={goal_x}, y={goal_y}, theta={goal_theta}')

        # Calculate the required movement to reach the goal
        move_cmd = Twist()
        # Simple proportional control
        move_cmd.linear.x = 0.5 * ((goal_x - self.x)**2 + (goal_y - self.y)**2)**0.5
        move_cmd.angular.z = 2 * (goal_theta - self.theta)

        # Update current position roughtly
        self.x = goal_x
        self.y = goal_y
        self.theta = goal_theta

        # Publish the move command
        self.publisher.publish(move_cmd)

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()