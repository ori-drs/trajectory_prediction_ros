cmake_minimum_required(VERSION 2.8.3)
project(trajectory_prediction)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs finean_msgs)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
find_package(gpmp2 REQUIRED)
include_directories(${gpmp2_INCLUDE_DIR})

## Declare a catkin package
catkin_package()


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(trajectory_prediction scripts/trajectory_prediction.cpp)
add_dependencies(trajectory_prediction ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_prediction gtsam gpmp2 ${catkin_LIBRARIES})




