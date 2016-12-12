#include <cmath>
#include <vector>
#include <string>
#include <boost/shared_ptr.hpp>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <auro_lidar_msgs/LidarSweep.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

#ifndef SWEEP_TO_POINT_CLOUD_H
#define SWEEP_TO_POINT_CLOUD_H

typedef pcl::PointCloud<pcl::PointXYZI> PointCloudXYZI;
class SweepToPointCloud
{
public:

  SweepToPointCloud();
  void spin();
  void callback(auro_lidar_msgs::LidarSweep::ConstPtr const & cloudPtr);
  PointCloudXYZI::PointType polarToCart(auro_lidar_msgs::LidarSweepPoint const & ls_point,float v_angle);

private:
  //ROS pub/sub/service
    ros::NodeHandle m_nh;
    ros::Subscriber m_lidar_sweep_sub;
    ros::Publisher m_point_cloud_pub;
};
#endif
