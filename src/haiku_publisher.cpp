#include <iostream>
#include <rclcpp/rclcpp.hpp>

  ////////////////////////////////////////////////////////////////////////////////////////////////////
 //                                            MAIN                                                //
////////////////////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
     rclcpp::init(argc,argv);                                                                       // Starts up ROS2
     
     rclcpp::Node node("haiku_publisher");                                                          // Create node
     
     std::cout << "\nWorker bees can leave.\n"
               <<   "Even drones can fly away.\n"
               <<   "The Queen is their slave.\n";
               
     return 0;                                                                                      // No problems with main()
}
