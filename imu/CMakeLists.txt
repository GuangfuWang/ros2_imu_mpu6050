cmake_minimum_required(VERSION 3.5)
project(imu)

set(default_imu "mpu6050uart")

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(${default_imu} ./src/mpu6050uart.cpp)
#target_link_libraries(${default_imu} )
file(GLOB ROS2_INCLUDE LIST_DIRECTORIES true "/opt/ros/foxy/include")
target_include_directories(${default_imu} PUBLIC 
			   $<BUILD_INTERFACE:/${ROS2_INCLUDE}>)

ament_target_dependencies(${default_imu} rclcpp sensor_msgs)

install(TARGETS ${default_imu}
        DESTINATION lib/${PROJECT_NAME})

ament_package()
