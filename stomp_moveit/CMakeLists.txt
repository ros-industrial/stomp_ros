cmake_minimum_required(VERSION 2.8.3)
project(stomp_moveit)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  moveit_core
  moveit_ros_planning
  moveit_experimental
  stomp_core
  cmake_modules
  pluginlib
)
find_package(Eigen REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_definitions("-std=c++11")


###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS moveit_ros_planning moveit_core stomp_core cmake_modules pluginlib roscpp moveit_experimental
  DEPENDS Eigen
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include
    ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/stomp_optimization_task.cpp
  src/stomp_planner.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# planner manager plugin
add_library(${PROJECT_NAME}_planner_manager
  src/stomp_planner_manager.cpp)
target_link_libraries(${PROJECT_NAME}_planner_manager ${PROJECT_NAME} ${catkin_LIBRARIES})

# cost function plugin(s)
add_library(${PROJECT_NAME}_cost_functions
  src/cost_functions/obstacle_avoidance.cpp
  src/cost_functions/collision_check.cpp
 )
target_link_libraries(${PROJECT_NAME}_cost_functions ${catkin_LIBRARIES})

# filter plugin(s)
add_library(${PROJECT_NAME}_filters
  src/filters/joint_limits.cpp
)
target_link_libraries(${PROJECT_NAME}_filters ${catkin_LIBRARIES})

# cost function plugin(s)
add_library(${PROJECT_NAME}_smoothers
  src/smoothers/control_cost_projection.cpp
  src/smoothers/update_logger.cpp
 )
target_link_libraries(${PROJECT_NAME}_cost_functions ${catkin_LIBRARIES})

#############
## Install ##
#############

#############
## Testing ##
#############
