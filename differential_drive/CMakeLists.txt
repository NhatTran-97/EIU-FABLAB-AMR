cmake_minimum_required(VERSION 3.8)
project(differential_drive)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)
#include_directories(include)


include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(motor_controller_node
    src/motor_controller.cpp)

ament_target_dependencies(motor_controller_node
    rclcpp
    std_msgs
    geometry_msgs

)
target_include_directories(motor_controller_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS} 
)    


add_executable(odom_estimator_node
    src/odom_estimator.cpp
)
ament_target_dependencies(odom_estimator_node
    rclcpp
    nav_msgs
    sensor_msgs
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_srvs
)

target_include_directories(odom_estimator_node
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)    

install(TARGETS
    motor_controller_node
    odom_estimator_node
    DESTINATION lib/${PROJECT_NAME}
)


install(PROGRAMS

 ${PROJECT_NAME}/differential_drive_controller.py 
 ${PROJECT_NAME}/motor_controller_node.py 
 ${PROJECT_NAME}/odom_estimator_node.py 

  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
