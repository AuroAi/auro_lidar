cmake_minimum_required(VERSION 2.8.3)
project(auro_lidar_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  roscpp
  geometry_msgs
  message_generation
  )

add_message_files(
  DIRECTORY msg
  FILES
  LidarSweepPoint.msg
  LidarSweepLayer.msg
  LidarSweep.msg
  CompactLidarSweepPoint.msg
  CompactLidarSweepLayer.msg
  CompactLidarSweep.msg
  )

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  message_generation
    DEPENDS auro_lidar_msgs_generate_messages_cpp
  )
