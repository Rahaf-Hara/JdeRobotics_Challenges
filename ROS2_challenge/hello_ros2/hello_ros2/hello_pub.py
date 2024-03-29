"""
A simple ROS2 publisher node using rclpy.

This script creates a ROS2 node named 'hello_publisher' that periodically
publishes a "Hello! ROS2 is fun" message on the 'hello' topic. The message is
published at a fixed interval of 1 second.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloPublisher(Node):
    """
    A node that publishes "Hello! ROS2 is fun" at a fixed interval.

    Attributes:
        publisher_ (Publisher): The publisher object for sending messages.
        timer (Timer): Timer to control the publishing interval.
    """
    def __init__(self):
        """
        Initializes the HelloPublisher node, creating a publisher and a timer
        to publish messages at a fixed interval.
        """
        super().__init__('hello_publisher')
        self.publisher_ = self.create_publisher(String, 'hello', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Callback function for the timer. It publishes the "Hello! ROS2 is fun"
        message and logs the publication.
        """
        msg = String()
        msg.data = 'Hello! ROS2 is fun'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    """
    Main function to initialize the ROS2 node and spin the node to keep it
    from exiting.
    """
    rclpy.init(args=args)
    hello_publisher = HelloPublisher()
    rclpy.spin(hello_publisher)
    hello_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
