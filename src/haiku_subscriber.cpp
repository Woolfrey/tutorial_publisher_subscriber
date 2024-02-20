#include <rclcpp/rclcpp.hpp>                                                                        // Fundamental ROS2 C++ packages
#include <std_msgs/msg/string.hpp>                                                                  // String message type

using namespace std::placeholders;                                                                  // For the std::bind() function below

// NOTE: This style is no longer recommended with ROS2.
rclcpp::Node::SharedPtr node = nullptr;                                                             // Forward declaration     

/**
 * Forward declaration of callback function.
 * It reads the '/haiku' topic and prints out the message.
 * @param message A string message obtained from the specified ROS topic.
 */
void callback(const std_msgs::msg::String &message)
{
     RCLCPP_INFO(node->get_logger(), message.data.c_str());                                         // Print out message to console
}

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            MAIN                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
     rclcpp::init(argc,argv);                                                                       // Starts up ROS2
     
     node = rclcpp::Node::make_shared("haiku_subscriber");                                          // Create and assign node to previously declared memory address

     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber
     = node->create_subscription<std_msgs::msg::String>("haiku", 10, callback);                     // Create subscriber to "haiku" topic, queue length 10, using callback()
     
     RCLCPP_INFO(node->get_logger(), "Subscribing to '/haiku'.");                                   // Inform user
     
     rclcpp::spin(node);                                                                            // Run the node indefinitely
     
     rclcpp::shutdown();                                                                            // Shut down ROS
        
     return 0;                                                                                      // No problems with main()
}
