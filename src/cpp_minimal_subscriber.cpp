/**
 * @file cpp_minimal_subscriber.cpp
 * @author Lalita Shanmugam
 * @brief A minimal ROS2 C++ subscriber node that listens to messages on a topic.   
 * @date 2025-Dec-18
 * 
 * ------------------------------------------------------------------------------
 * 
 * Subscription Topics:
 * String message
 * /cpp_example_topic - std_msgs/msg/String
 * 
 * ------------------------------------------------------------------------------
 * 
 * Publisher Topic:
 * None
 */


#include "rclcpp/rclcpp.hpp"  // Ros2 C++ client library
#include "std_msgs/msg/string.hpp" // Standard String message type

using std::placeholders::_1; // Placeholder for callback function 

class MinimalCppSubscriber : public rclcpp::Node // Define a class inheriting from rclcpp::Node
{
public:
  MinimalCppSubscriber() : Node("minimal_cpp_subscriber") // Initialize the node with the name "minimal_cpp_subscriber"
  {
    // Create a subscriber for std_msgs::msg::String messages on the "/cpp_example_topic" topic
    subscriber_ = create_subscription<std_msgs::msg::String>
    (
      "/cpp_example_topic", 10, std::bind(&MinimalCppSubscriber::topicCallback, this, _1)
    );

    RCLCPP_INFO(get_logger(), "Subscriber to /cpp_example_topic");
  }
  void topicCallback(const std_msgs::msg::String & msg) const // Callback function to be called when a message is received
  {
    RCLCPP_INFO_STREAM(get_logger(), "I heard:'" <<msg.data.c_str()); // Log the received message
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_; // Subscriber for String messages
};

int main(int argc, char * argv[]) // Main function
{
  rclcpp::init(argc, argv); // Initialize the ROS 2 client library

  auto minimal_cpp_subscriber_node = std::make_shared<MinimalCppSubscriber>(); // Create an instance of the MinimalCppSubscriber node

  rclcpp::spin(minimal_cpp_subscriber_node); // Create and spin the node to process callbacks

  rclcpp::shutdown(); // Shutdown the ROS 2 client library
  return 0;
}