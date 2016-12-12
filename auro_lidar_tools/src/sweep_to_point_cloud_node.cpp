
#include <ros/ros.h>
#include <auro_lidar_tools/sweep_to_point_cloud.h>

int main(int argc, char** argv) {
  ros::init(argc, argv, "SweepToPointCloud_node");

  SweepToPointCloud sweepToPointCloud_;

  sweepToPointCloud_.spin();

  //ros::spin();


  return 0;
}
