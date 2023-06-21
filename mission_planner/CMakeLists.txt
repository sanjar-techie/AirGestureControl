cmake_minimum_required(VERSION 3.0.2)
project(mission_planner)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  cv_bridge
)

# Include Eigen3
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR} # Add Eigen3 includes
)

add_executable(mission_planner_node src/mission_planner.cpp)
add_executable(mission_planner_node_copy src/mission_planner_copy.cpp)

target_link_libraries(mission_planner_node
  ${catkin_LIBRARIES}
)

target_link_libraries(mission_planner_node_copy
  ${catkin_LIBRARIES}
)
