cmake_minimum_required(VERSION 2.8.3)
project(auro_lidar_tools)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
  auro_lidar_msgs
  )

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED COMPONENTS common io)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp executable
add_executable(sweep_to_point_cloud
  src/sweep_to_point_cloud.cpp
  src/sweep_to_point_cloud_node.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(sweep_to_point_cloud
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES}
 )

#############
## Install ##
#############

## Mark executables and/or libraries for installation
install(TARGETS sweep_to_point_cloud
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)




