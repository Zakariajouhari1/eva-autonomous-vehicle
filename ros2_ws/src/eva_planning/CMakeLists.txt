cmake_minimum_required(VERSION 3.5)
project(eva_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# NEW: Add CURL and JSON for OSRM routing
find_package(CURL REQUIRED)
find_package(nlohmann_json REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${CURL_INCLUDE_DIRS}
)

# A* Planner Library (COMMENTED OUT - not used with OSRM)
# add_library(astar_planner src/astar_planner.cpp)
# ament_target_dependencies(astar_planner rclcpp)

# Cubic Spline Planner Library (KEEP THIS)
add_library(cubic_spline_planner src/cubic_spline_planner.cpp)
target_link_libraries(cubic_spline_planner ${EIGEN3_LIBRARIES})
ament_target_dependencies(cubic_spline_planner rclcpp)

# Main Planning Node (UPDATED)
add_executable(eva_planning_node src/eva_planning_node.cpp)
target_link_libraries(eva_planning_node 
  cubic_spline_planner
  ${CURL_LIBRARIES}
  nlohmann_json::nlohmann_json
)
ament_target_dependencies(eva_planning_node
  rclcpp nav_msgs geometry_msgs sensor_msgs std_msgs
  visualization_msgs tf2 tf2_geometry_msgs
)

# Install
install(TARGETS cubic_spline_planner
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS eva_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(PROGRAMS
  scripts/eva_test_generator.py
  scripts/planning_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(cubic_spline_planner)

ament_package()
