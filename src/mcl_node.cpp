#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <iterator>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <mcl4pfc/mcl_node.h>


MclNode::MclNode(ros::NodeHandle& nh)
: nh_(nh)
{
  sub_cmd_vel_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,
                                                     &MclNode::cmdVelCb, this);
  sub_scan_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1,
                                                    &MclNode::scanCb, this);
  sub_map_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1,
                                                    &MclNode::mapCb, this);
  pub_particlecloud_ = nh_.advertise<geometry_msgs::PoseArray>("/particlecloud",
                                                               10);
}

MclNode::~MclNode()
{
}

void MclNode::cmdVelCb(const geometry_msgs::TwistConstPtr& twist)
{
  nu_ = twist->linear.x;
  omega_ = twist->angular.z;
}

void MclNode::scanCb(const sensor_msgs::LaserScanConstPtr& scan)
{
  scan_ = scan->ranges;
  range_min_ = scan->range_min;
  range_max_ = scan->range_max;
}

void MclNode::mapCb(const nav_msgs::OccupancyGridConstPtr& map)
{
  map_ = map->data;
  resolution_ = map->info.resolution;
  width_ = map->info.width;
  height_ = map->info.height;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mcl4pfc");
  ros::NodeHandle nh;
  MclNode mcl_node(nh);
  ros::spin();
}
