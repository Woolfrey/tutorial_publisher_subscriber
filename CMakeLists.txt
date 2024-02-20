cmake_minimum_required(VERSION 3.8)
project(tutorial_publisher_subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################################################################################################
#                                         DEPENDENCIES                                             #
####################################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
     find_package(ament_lint_auto REQUIRED)

     set(ament_cmake_copyright_FOUND TRUE)

     set(ament_cmake_cpplint_FOUND TRUE)
     ament_lint_auto_find_test_dependencies()
endif()

####################################################################################################
#                                          EXECUTABLES                                             #
####################################################################################################

add_executable(haiku_publisher src/haiku_publisher.cpp)
ament_target_dependencies(haiku_publisher
                          "rclcpp"
                          "std_msgs")

# This is so ROS2 can find the executable when we call `ros2 run`                        
install(TARGETS
        haiku_publisher
        DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
