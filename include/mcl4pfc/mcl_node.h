#pragma once

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>


class MclNode
{
 public:
  MclNode(const ros::NodeHandle& nh);
  ~MclNode();
 private:
  void cmdVelCb(const geometry_msgs::TwistConstPtr& twist);
  void scanCb(const sensor_msgs::LaserScanConstPtr& scan);
  void mapCb(const nav_msgs::OccupancyGridConstPtr& map);
  
 private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber sub_cmd_vel_;
  ros::Subscriber sub_scan_;
  ros::Subscriber sub_map_;
  ros::Publisher pub_particlecloud_;
  // Vel
  double nu_;
  double omega_;
  // Laser scan
  std::vector<float> scan_;
  float range_min_;
  float range_max_;
  // Grid map
  std::vector<int8_t> map_;
  float resolution_;
  uint32_t width_;
  uint32_t height_;
};
