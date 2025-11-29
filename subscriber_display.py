import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class DisplaySubscriber(Node):
    def __init__(self):
        super().__init__('display_subscriber')

        self.subscription = self.create_subscription(
            Float32,
            'ultrasonic_distance',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Distance received: {msg.data} cm')

def main(args=None):
    rclpy.init(args=args)
    node = DisplaySubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
