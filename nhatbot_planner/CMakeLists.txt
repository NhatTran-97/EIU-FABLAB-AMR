cmake_minimum_required(VERSION 3.8)
project(nhatbot_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_msgs REQUIRED)

set(DIJKSTRA_SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/dijkstra_planner")
set(A_STAR_SRC_PATH "${CMAKE_CURRENT_SOURCE_DIR}/src/a_star_planner")
set(dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
)

set(nav2_dependencies
  rclcpp
  nav_msgs
  geometry_msgs
  tf2_ros
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
  nav2_msgs
  rclcpp_action
)

add_executable(a_star_planner ${A_STAR_SRC_PATH}/a_star_planner.cpp)
ament_target_dependencies(a_star_planner ${dependencies})
target_include_directories(a_star_planner
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)    

add_executable(dijkstra_planner ${DIJKSTRA_SRC_PATH}/dijkstra_planner.cpp)
ament_target_dependencies(dijkstra_planner ${dependencies})
target_include_directories(dijkstra_planner
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)    



add_executable(a_star_planner_costmap ${A_STAR_SRC_PATH}/a_star_planner_costmap.cpp)
ament_target_dependencies(a_star_planner_costmap ${dependencies})
target_include_directories(a_star_planner_costmap
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)   

add_executable(dijkstra_planner_costmap ${DIJKSTRA_SRC_PATH}/dijkstra_planner_costmap.cpp)
ament_target_dependencies(dijkstra_planner_costmap ${dependencies})
target_include_directories(dijkstra_planner_costmap
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)    


add_library(dijkstra_nav2_planner SHARED ${DIJKSTRA_SRC_PATH}/dijkstra_nav2_planner.cpp)
ament_target_dependencies(dijkstra_nav2_planner ${nav2_dependencies})
target_include_directories(dijkstra_nav2_planner
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)    

add_library(a_star_nav2_planner SHARED ${A_STAR_SRC_PATH}/a_star_nav2_planner.cpp)
ament_target_dependencies(a_star_nav2_planner ${nav2_dependencies})
target_include_directories(a_star_nav2_planner
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)      


add_library(dijkstra_nav2_smoother SHARED ${DIJKSTRA_SRC_PATH}/dijkstra_nav2_smoother.cpp)
ament_target_dependencies(dijkstra_nav2_smoother ${nav2_dependencies})
target_include_directories(dijkstra_nav2_smoother
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)    

add_library(a_star_nav2_smoother SHARED  ${A_STAR_SRC_PATH}/a_star_nav2_smoother.cpp)
ament_target_dependencies(a_star_nav2_smoother ${nav2_dependencies})
target_include_directories(a_star_nav2_smoother
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>$<INSTALL_INTERFACE:include>)    


pluginlib_export_plugin_description_file(nav2_core global_planner_plugins.xml)


install(TARGETS
  a_star_planner
  dijkstra_planner_costmap
  a_star_planner_costmap
  DESTINATION lib/${PROJECT_NAME})


install(TARGETS
  dijkstra_nav2_planner
  a_star_nav2_planner
  dijkstra_nav2_smoother
  a_star_nav2_smoother


  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})


install(FILES global_planner_plugins.xml
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS

  ${PROJECT_NAME}/dijkstra_planner.py 
  ${PROJECT_NAME}/a_star_planner.py

  nhatbot_planner_costmap/dijkstra_planner_costmap.py 
  nhatbot_planner_costmap/a_star_planner_costmap.py 
 
 DESTINATION lib/${PROJECT_NAME}
)



install(
  DIRECTORY  config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

ament_export_dependencies(rclcpp nav_msgs geometry_msgs tf2_ros  nav2_core nav2_costmap_2d nav2_util pluginlib rclcpp_action nav2_msgs)
ament_export_libraries(dijkstra_nav2_planner a_star_nav2_planner  dijkstra_nav2_smoother a_star_nav2_smoother)
ament_package()
