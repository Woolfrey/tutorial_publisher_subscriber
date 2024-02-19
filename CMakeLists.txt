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
                          "rclcpp")

ament_package()
