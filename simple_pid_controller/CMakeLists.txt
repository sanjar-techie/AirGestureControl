cmake_minimum_required(VERSION 3.0.2)
project(simple_pid_controller)

## Compile as C++11, supported in ROS Noetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  std_msgs
  cv_bridge      
  image_transport
  std_srvs       
)


###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES simple_pid_controller
   CATKIN_DEPENDS geometry_msgs roscpp std_msgs cv_bridge image_transport
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node src/sub_goal_pid.cpp)
#add_executable(mission_planner_node src/mission_planner.cpp)
# add_executable(selfie_image_saver_node src/image_saver_node.cpp)   ## Add this line for image_saver
# add_executable(combined_node src/combined_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)

#target_link_libraries(mission_planner_node
 # ${catkin_LIBRARIES}
#)

# target_link_libraries(selfie_image_saver_node    ## And this block for image_saver
#   ${catkin_LIBRARIES}
# )

# target_link_libraries(combined_node    ## And this block for image_saver
#   ${catkin_LIBRARIES}
# )
