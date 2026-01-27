cmake_minimum_required(VERSION 3.8)
project(nhatbot_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)

install(PROGRAMS
# ${PROJECT_NAME}/cmd_to_drive.py 
DESTINATION lib/${PROJECT_NAME}
)

# compute path client executable
add_executable(compute_path_client src/compute_path_client.cpp)
ament_target_dependencies(compute_path_client 
  rclcpp
  rclcpp_action
  geometry_msgs
  nav2_msgs
)
target_include_directories(compute_path_client
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>
)



add_executable(follow_path_client src/follow_path_client.cpp)
ament_target_dependencies(
  follow_path_client
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
)




install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


install(TARGETS compute_path_client
  follow_path_client
  DESTINATION lib/${PROJECT_NAME})



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
