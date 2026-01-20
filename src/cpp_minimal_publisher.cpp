#include "rclcpp/rclcpp.hpp"  // Ros2 C++ client library
#include "std_msgs/msg/string.hpp" // Standard String message type

using namespace std::chrono_literals; // For using time duration literals like 500ms

class MinimalCppPublisher : public rclcpp::Node // Define a class inheriting from rclcpp::Node
{
public:
  MinimalCppPublisher() : Node("minimal_cpp_publisher"), count(0) // Initialize the node with the name "minimal_cpp_publisher"
  {
    // Create a publisher for std_msgs::msg::String messages on the "/cpp_example_topic" topic
    publisher_ = this->create_publisher<std_msgs::msg::String>("/cpp_example_topic", 10);
    
    // Create a timer that triggers every 500 milliseconds
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalCppPublisher::timerCallback, this));

    RCLCPP_INFO(get_logger(), "Publisher every 2Hz");
  }
  void timerCallback() // Callback function to be called by the timer
  {
    auto message = std_msgs::msg::String(); // Create a new String message
    message.data = "Hello, world! " + std::to_string(count++); // Set the message data with a count

    RCLCPP_INFO(get_logger(), "Publishing: '%s'", message.data.c_str()); // Log the message being published

    publisher_->publish(message); // Publish the message
  }

private:

  size_t count; // Counter for the messages
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_; // Publisher for String messages
  rclcpp::TimerBase::SharedPtr timer_; // Timer to trigger periodic callbacks
};

int main(int argc, char * argv[]) // Main function
{
  rclcpp::init(argc, argv); // Initialize the ROS 2 client library

  auto minimal_cpp_publisher_node = std::make_shared<MinimalCppPublisher>(); // Create an instance of the MinimalCppPublisher node

  rclcpp::spin(minimal_cpp_publisher_node); // Create and spin the node to process callbacks

  rclcpp::shutdown(); // Shutdown the ROS 2 client library
  return 0;
}