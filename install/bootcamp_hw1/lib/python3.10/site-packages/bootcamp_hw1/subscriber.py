import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class StateSubscriber(Node):

    def __init__(self):
        super().__init__('state_subscriber')
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'seven_dim_state',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Float64MultiArray, 'six_dim_state', 10)

    def listener_callback(self, msg):
        new_msg = Float64MultiArray()
        new_msg.data = msg.data[:6]
        self.publisher.publish(new_msg)

def main(args=None):
    rclpy.init(args=args)
    state_subscriber = StateSubscriber()
    rclpy.spin(state_subscriber)
    state_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
