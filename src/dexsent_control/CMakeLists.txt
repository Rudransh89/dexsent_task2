cmake_minimum_required(VERSION 3.8)
project(dexsent_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 1. Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(controller_interface REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(Eigen3 REQUIRED)

# 2. Define Library
add_library(dexsent_cartesian_controller SHARED
  src/cartesian_controller.cpp
)

# 3. Include Headers
target_include_directories(dexsent_cartesian_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)

# 4. LINKING FIX: Use ament_target_dependencies instead of target_link_libraries
# This automatically handles the "geometry_msgs" linking that failed before.
ament_target_dependencies(dexsent_cartesian_controller
  rclcpp
  pluginlib
  controller_interface
  geometry_msgs
  kdl_parser
  orocos_kdl
)

# 5. Export Plugin
pluginlib_export_plugin_description_file(controller_interface dexsent_control.xml)

# 6. Install Rules
install(TARGETS dexsent_cartesian_controller
  EXPORT export_dexsent_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/ DESTINATION include)
install(FILES dexsent_control.xml DESTINATION share/${PROJECT_NAME})
# Install launch and config folders so ros2 launch can find them
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

# 7. Export Dependencies for downstream packages
ament_export_libraries(dexsent_cartesian_controller)
ament_export_dependencies(rclcpp controller_interface geometry_msgs)
ament_package()
