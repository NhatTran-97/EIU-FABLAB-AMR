cmake_minimum_required(VERSION 3.8)
project(nhatbot_firmware)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-psabi)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBMODBUS REQUIRED libmodbus)

include_directories(
  include
  ${LIBMODBUS_INCLUDE_DIRS}
)

# ------------------------------
# Executable node (for testing driver)
# ------------------------------
add_executable(zlac_driver_node
  src/main.cpp
  src/zlac_sdk.cpp
)
target_link_libraries(zlac_driver_node ${LIBMODBUS_LIBRARIES})
ament_target_dependencies(zlac_driver_node rclcpp)

# ------------------------------
# ZLAC SDK as shared lib
# ------------------------------
add_library(zlac_sdk SHARED src/zlac_sdk.cpp)
target_include_directories(zlac_sdk PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(zlac_sdk ${LIBMODBUS_LIBRARIES})
ament_target_dependencies(zlac_sdk rclcpp)

# ------------------------------
# Hardware interface plugin
# ------------------------------
add_library(${PROJECT_NAME} SHARED src/nhatbot_hw_interface.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} zlac_sdk)
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export plugin description
pluginlib_export_plugin_description_file(hardware_interface motor_interfaces.xml)

# ------------------------------
# Install rules
# ------------------------------
install(
  TARGETS zlac_driver_node zlac_sdk ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

# ------------------------------
# Export package
# ------------------------------
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} zlac_sdk)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
