cmake_minimum_required(VERSION 3.8)
project(nhatbot_ros2_basic)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
#find_package(ament_python REQUIRED)

find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED) 

find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
#find_package(custom_interfaces REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(tf2_ros REQUIRED) 
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()



install(TARGETS

  DESTINATION lib/${PROJECT_NAME}

 )




 #install python Executables
install(PROGRAMS
  # scripts/py_node.py
 
  DESTINATION lib/${PROJECT_NAME}
 )


install(DIRECTORY
  launch
  rviz_config
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
