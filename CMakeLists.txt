cmake_minimum_required(VERSION 3.0.2)
project(trajectory_prediction)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
find_package(GTSAM REQUIRED)
include_directories(${GTSAM_INCLUDE_DIR})
find_package(gpmp2 REQUIRED)
include_directories(${gpmp2_INCLUDE_DIR})

## Declare a catkin package
catkin_package(INCLUDE_DIRS include 
                LIBRARIES trajectory_prediction)

add_library(trajectory_prediction  
src/trajectory_prediction.cpp
)

target_link_libraries(trajectory_prediction 
    gtsam       
    gpmp2 
    ${catkin_LIBRARIES}
    )

include_directories(
                    include 
                    ${catkin_INCLUDE_DIRS}
)


add_executable(trajectory_prediction_node scripts/trajectory_prediction_node.cpp
                src/trajectory_prediction.cpp
)
# add_dependencies(trajectory_prediction_node 
#                  ${catkin_EXPORTED_TARGETS})
target_link_libraries(trajectory_prediction_node 
                        trajectory_prediction
                        gtsam 
                        gpmp2 
                        ${catkin_LIBRARIES})


install(TARGETS trajectory_prediction
ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include
DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

