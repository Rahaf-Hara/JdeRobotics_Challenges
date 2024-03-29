"""
A simple ROS2 subscriber node using rclpy.

This script creates a ROS2 node named 'hello_subscriber' that subscribes to the
'hello' topic. Upon receiving messages on this topic, it logs the message,
demonstrating basic subscriber functionality in ROS2 with Python.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class HelloSubscriber(Node):
    """
    A ROS2 node that subscribes to messages on the 'hello' topic.

    This node demonstrates the subscription mechanism in ROS2, listening for
    String messages and logging their content when received.
    """

    def __init__(self):
        """
        Initializes the HelloSubscriber node with a subscription to the 'hello'
        topic.
        """
        super().__init__('hello_subscriber')
        self.subscription = self.create_subscription(
            String,
            'hello',
            self.listener_callback,
            10)
        self.subscription  # This is to prevent an unused variable warning

    def listener_callback(self, msg):
        """
        Callback function that is invoked upon receiving a message on the
        'hello' topic.

        Args:
            msg (std_msgs.msg.String): The message received by the subscriber.
        """
        self.get_logger().info(f'Heard: "{msg.data}"')


def main(args=None):
    """
    Function that initializes the ROS2 node and keeps it alive by spinning.

    This function sets up the ROS2 system and creates an instance of the
    HelloSubscriber node, then enters a loop where the node remains alive
    waiting for incoming messages.
    """
    rclpy.init(args=args)
    hello_subscriber = HelloSubscriber()
    rclpy.spin(hello_subscriber)
    hello_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
