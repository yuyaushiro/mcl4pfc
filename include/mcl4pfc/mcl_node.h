#pragma once

#include <ros/ros.h>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <mcl4pfc/mcl.h>


class MclNode
{
 public:
  MclNode(const ros::NodeHandle& nh);
  ~MclNode();
 private:
  void cmdVelCb(const geometry_msgs::TwistConstPtr& twist);
  void scanCb(const sensor_msgs::LaserScanConstPtr& scan);
  void mapCb(const nav_msgs::OccupancyGridConstPtr& map);
  void pubParticlecloud();
  void RPYToQuaternion(double roll, double pitch, double yaw,
                        geometry_msgs::Quaternion& q);
  
 private:
  // ROS
  ros::NodeHandle nh_;
  ros::Subscriber cmd_vel_sub_;
  ros::Subscriber scan_sub_;
  ros::Subscriber map_sub_;
  ros::Publisher particlecloud_pub_;
  // Vel
  float nu_;
  float omega_;
  // Laser scan
  int i_;
  std::vector<float> ranges_;
  float angle_min_;
  float angle_max_;
  float angle_increment_;
  std::vector<float> angles_;
  // Grid map
  std::vector<int8_t> map_;
  float resolution_;
  uint32_t width_;
  uint32_t height_;
  // MCL
  Mcl mcl_;
};
