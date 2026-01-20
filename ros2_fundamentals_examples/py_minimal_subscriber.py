#!/usr/bin/env python3

"""
Description: this ROS2 node subscribes to "Hello World" messages from a topic.
--------------------------------------------------------------
Publishing Topic: 
          None
--------------------------------------------------------------
Subscribing Topic: 
            The channel containing the "Hello World" messages.
            /py_example_topic - std_msgs/String
--------------------------------------------------------------
Author: Lalita Shanmugam
Date: Dec 11, 2025
"""

import rclpy    # ROS2 Python client library
from rclpy.node import Node   #Import Node class, used to create ROS2 nodes
from std_msgs.msg import String   # Import String message type from std_msgs package for ROS2

class MinimalPySubscriber(Node):
    """
    Create a minimal subscriber node.
    """
    def __init__(self):
        """
             Create a custom node class for subscribing to messages.
        """
        # Initialize the Node class using the super() function
        super().__init__('minimal_py_subscriber')

        # Create a subscriber object that will subscribe to String messages from the 'py_example_topic' topic with a queue size of 10
        self.subscription_1 = self.create_subscription(
            String,
            'py_example_topic',
            self.listener_callback,
            10)

        # Prevent unused variable warning
        self.subscription_1

    def listener_callback(self, msg):
        """
        Callback function that is executed every time a message is received on the subscribed topic.
        """
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    """
    Main function to initialize and run the subscriber node.
    """
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of the MinimalPySubscriber node
    minimal_py_subscriber = MinimalPySubscriber()

    # Keep the node running and listening for messages
    rclpy.spin(minimal_py_subscriber)

    # Destroy the node explicitly (optional)
    minimal_py_subscriber.destroy_node()

    # Shutdown the rclpy library
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()