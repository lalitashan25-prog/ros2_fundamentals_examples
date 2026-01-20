#!/usr/bin/env python3

"""
Description: this ROS2 node publishes "Hello World" messages to a topic.
--------------------------------------------------------------
Publishing Topic: 
          The channel containing the "Hello World" messages.
          /py_example_topic - std_msgs/String
--------------------------------------------------------------
Subscribing Topic: 
          None
--------------------------------------------------------------
Author: Lalita Shanmugam
Date: Dec 10, 2025
"""

import rclpy    # ROS2 Python client library
from rclpy.node import Node   #Import Node class, used to create ROS2 nodes
from std_msgs.msg import String   # Import String message type from std_msgs package for ROS2

class MinimalPyPublisher(Node):
    """
    Create a minimal publisher node.
    """
    def __init__(self):
        """
             Create a custom node class for publishing messages.
        """
        # Initialize the Node class using the super() function
        super().__init__('minimal_py_publisher')

        # Create a publisher object that will publish String messages to the 'py_example_topic' topic with a queue size of 10
        self.publisher_1 = self.create_publisher(String, 'py_example_topic', 10)

        # Set a timer to call the timer_callback function every 0.5 seconds to trigger message publishing
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize a counter to keep track of the number of messages published
        self.i = 0

    def timer_callback(self):
        """
        Callback function that is exectuted every time the timer triggers.
        """
        # Create a new String message object
        msg = String()

        # Set the data with counter value
        msg.data = 'Hello World: %d' % self.i

        # Publish the message u created above to the topic
        self.publisher_1.publish(msg)

        # Log the message indicating it has been published
        self.get_logger().info(f'Publishing: "%s"' % msg.data)

        # Increment the message counter for the next message
        self.i += 1

def main(args=None):
    """
    Main function to start the ROS2 node.
    """
    # Initialize the ROS2 Python client library
    rclpy.init(args=args)

    # Create an instance of the MinimalPyPublisher class
    minimal_py_publisher = MinimalPyPublisher()

    # Keep the node running and processing callbacks until it is shut down
    rclpy.spin(minimal_py_publisher)

    # Destroy the node explicitly (optional)
    minimal_py_publisher.destroy_node()

    # Shutdown the ROS2 client library
    rclpy.shutdown()

if __name__ == '__main__':
    # Call the main function to start the node
    main()