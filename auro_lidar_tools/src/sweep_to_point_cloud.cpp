#include <auro_lidar_tools/sweep_to_point_cloud.h>

SweepToPointCloud::SweepToPointCloud():
m_nh("~")
{

  m_lidar_sweep_sub = m_nh.subscribe<auro_lidar_msgs::LidarSweep>(
        "lidar_sweep", 100, &SweepToPointCloud::callback, this);
   m_point_cloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>(
        "point_cloud", 10);

}

void SweepToPointCloud::spin()
{
  ros::spin();
}

void SweepToPointCloud::callback(auro_lidar_msgs::LidarSweep::ConstPtr const & cloudPtr)
    {
      if (!cloudPtr) return;
      pcl::PointCloud<pcl::PointXYZI>::Ptr point_cloud(
          new pcl::PointCloud<pcl::PointXYZI>());
      point_cloud->header.stamp =
        pcl_conversions::toPCL(cloudPtr->header).stamp;
      point_cloud->header.frame_id = cloudPtr->header.frame_id;
      point_cloud->height = 1;

      for (size_t i = 0; i < cloudPtr->layer_count; ++i) {
         for (size_t j = 0; j < cloudPtr->layers[i].points.size(); ++j) {
          pcl::PointXYZI point = polarToCart(cloudPtr->layers[i].points[j],cloudPtr->layers[i].altitude);
          point_cloud->points.push_back(point);
          ++point_cloud->width;
        }
      }
      point_cloud->is_dense=true;
      m_point_cloud_pub.publish(point_cloud);
    }
/*
 *            pcl::PointXYZI point;
           point.intensity = cloudPtr->layers[i].points[j].intensity;


                 double const cos_horizontal_angle = std::cos(cloudPtr->layers[i].points[j].azimuth);
                 double const sin_horizontal_angle = std::sin(cloudPtr->layers[i].points[j].azimuth);

                 double const cos_vertical_angle = std::cos(cloudPtr->layers[i].altitude);
                 double const sin_vertical_angle = std::sin(cloudPtr->layers[i].altitude);

                 // get the distance to the XY plane
                 double xy_distance = cloudPtr->layers[i].points[j].distance * cos_vertical_angle;

                 point.y = static_cast<float> (xy_distance * sin_horizontal_angle);

                 point.x = static_cast<float> (xy_distance * cos_horizontal_angle);

                 point.z = static_cast<float> (cloudPtr->layers[i].points[j].distance * sin_vertical_angle);
 */

PointCloudXYZI::PointType SweepToPointCloud::polarToCart(auro_lidar_msgs::LidarSweepPoint const & ls_point,float v_angle)
    {
      PointCloudXYZI::PointType to;

      to.intensity = ls_point.intensity;

      if (std::isnan (ls_point.distance))
      {
        to.x = to.y = to.z = std::numeric_limits<float>::quiet_NaN ();
        return to;
      }

      double const cos_horizontal_angle = std::cos(ls_point.azimuth);
      double const sin_horizontal_angle = std::sin(ls_point.azimuth);

      double const cos_vertical_angle = std::cos(v_angle);
      double const sin_vertical_angle = std::sin(v_angle);

      // get the distance to the XY plane
      double xy_distance = ls_point.distance * cos_vertical_angle;

      to.y = static_cast<float> (xy_distance * sin_horizontal_angle);

      to.x = static_cast<float> (xy_distance * cos_horizontal_angle);

      to.z = static_cast<float> (ls_point.distance * sin_vertical_angle);

      return to;
    }




