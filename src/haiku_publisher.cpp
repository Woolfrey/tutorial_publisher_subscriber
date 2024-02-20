#include <rclcpp/rclcpp.hpp>                                                                        // Fundamental ROS2 C++ packages
#include <std_msgs/msg/string.hpp>                                                                  // String message type

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            MAIN                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
     rclcpp::init(argc,argv);                                                                       // Starts up ROS2
     
     rclcpp::Node node("haiku_publisher");                                                          // Create node with name "haiku_publisher"

     rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher
     = node.create_publisher<std_msgs::msg::String>("haiku",10);                                    // Advertise topic with given name, queue length 10
     
     rclcpp::Rate loopRate(1.0);                                                                    // Create loop timer of 1Hz
 
     RCLCPP_INFO(node.get_logger(), "Publishing haiku. "
                                    "Use 'ros2 topic echo /haiku' in another terminal to view output."); // Inform the user
     
     int line = 1;                                                                                  // Select which line of Haiku to publish
     
     // Run indefinitely until this node is forcibly cancelled
     while(rclcpp::ok())
     {
          std_msgs::msg::String message;                                                            // To be published
          
          if(line == 1)      message.data = "Worker bees can leave.";
          else if(line == 2) message.data = "Even drones can fly away.";
          else if(line == 3) message.data = "The Queen is their slave.";
          
          if(line < 3) line++;                                                                      // Go to next line of poem
          else         line = 1;                                                                    // Back to first

          publisher->publish(message);                                                              // Send message over ROS network
 
          loopRate.sleep();                                                                         // Regulate while() loop at given rate
     }
     
     return 0;                                                                                      // No problems with main()
}
