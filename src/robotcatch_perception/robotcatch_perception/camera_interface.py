import rclpy
from rclpy.node import Node

from std_msgs.msg import String # Import the String message type

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher') # Name of your node
        self.publisher_ = self.create_publisher(String, 'topic', 10) # Create a publisher
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher) # Keep the node running
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
