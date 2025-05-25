cmake_minimum_required(VERSION 3.8)
project(nhatbot_twist_teleop)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)



add_executable(joy_to_twist src/joy_to_twist.cpp)
ament_target_dependencies(joy_to_twist rclcpp sensor_msgs geometry_msgs)

add_executable(twist_relay src/twist_relay.cpp)
ament_target_dependencies(twist_relay rclcpp  geometry_msgs)


install(PROGRAMS
 ${PROJECT_NAME}/joy_to_twist.py 
 DESTINATION lib/${PROJECT_NAME}
)


install(TARGETS
    joy_to_twist
    twist_relay
    DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
