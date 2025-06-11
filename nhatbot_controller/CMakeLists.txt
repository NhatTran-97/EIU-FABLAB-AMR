cmake_minimum_required(VERSION 3.8)
project(nhatbot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_msgs REQUIRED)


set(PURE_PURSUIT_SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/purePursuit_controller")
set(PD_SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/pd_controller")




set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

set(nav2_dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav2_msgs
)


# Raw Controller

add_executable(pure_pursuit ${PURE_PURSUIT_SRC_PATH}/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit ${dependencies})
target_include_directories(pure_pursuit PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)    


add_executable(pd_motion_planner ${PD_SRC_PATH}/pd_motion_planner.cpp)
ament_target_dependencies(pd_motion_planner ${dependencies})
target_include_directories(pd_motion_planner PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)    



# Costmap-Nav2 Controller Integration 

add_library(nav2_pd_motion_planner SHARED ${PD_SRC_PATH}/nav2_pd_motion_planner.cpp)
ament_target_dependencies(nav2_pd_motion_planner ${nav2_dependencies})
target_include_directories(nav2_pd_motion_planner PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)    



add_library(nav2_pure_pursuit_motion_planner SHARED ${PURE_PURSUIT_SRC_PATH}/nav2_pure_pursuit_motion_planner.cpp)
ament_target_dependencies(nav2_pure_pursuit_motion_planner ${nav2_dependencies})
target_include_directories(nav2_pure_pursuit_motion_planner PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)    


install(TARGETS
  pure_pursuit
  pd_motion_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  nav2_pd_motion_planner
  nav2_pure_pursuit_motion_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})



pluginlib_export_plugin_description_file(nav2_core motion_planner_plugins.xml)

install(PROGRAMS
  ${PROJECT_NAME}/tracking.py
  ${PROJECT_NAME}/tracking_controller.py
  # ${PROJECT_NAME}/pure_pursuit.py
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY  waypoints 
  DESTINATION share/${PROJECT_NAME}
)



install(FILES motion_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_export_dependencies(${nav2_dependencies})
ament_export_libraries(pd_nav2_motion_planner pure_pursuit_nav2_motion_planner)
ament_package()
