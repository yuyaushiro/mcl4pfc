#include <ros/ros.h>
#include <cmath>
#include <vector>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sensor_msgs/LaserScan.h>
#include <tf/transform_datatypes.h>
#include <mcl4pfc/mcl_node.h>
#include <mcl4pfc/mcl.h>


MclNode::MclNode(const ros::NodeHandle& nh) :
  nh_(nh)
{
  ros::Rate loop_rate(10);
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,
                                                     &MclNode::cmdVelCb, this);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1,
                                                    &MclNode::scanCb, this);
  map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1,
                                                    &MclNode::mapCb, this);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/particlecloud",
                                                               10);
  int particle_num = 1000;
  double initial_pose[] = {1.0, 0.5, 0.0};
  double state_transition_sigma = 0.01;
  double likelihood_field_sigma = 0.05;
  mcl_ = Mcl(particle_num, initial_pose,
             state_transition_sigma, likelihood_field_sigma);
}

MclNode::~MclNode()
{
}

void MclNode::cmdVelCb(const geometry_msgs::TwistConstPtr& twist)
{
  nu_ = twist->linear.x;
  omega_ = twist->angular.z;
  float u[] = {nu_, omega_};

  MclNode::pubParticlecloud();
  mcl_.updateWithMotion(u, 0.1);
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

void MclNode::pubParticlecloud()
{
  geometry_msgs::PoseArray pose_array;
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = "/map";
  for (auto itr = mcl_.particles_.begin(); itr != mcl_.particles_.end();
       ++itr)
  {
    geometry_msgs::Pose pose;
    pose.position.x = itr->pose_[0];
    pose.position.y = itr->pose_[1];
    geometry_msgs::Quaternion geo_msg_quaternion;
    MclNode::RPYToQuaternion(0, 0, itr->pose_[2], geo_msg_quaternion);
    pose.orientation = geo_msg_quaternion;
    pose_array.poses.push_back(pose);
  }
  particlecloud_pub_.publish(pose_array);
}

void MclNode::RPYToQuaternion(double roll, double pitch, double yaw,
                               geometry_msgs::Quaternion& q)
{
   tf::Quaternion quat=tf::createQuaternionFromRPY(roll,pitch,yaw);
   quaternionTFToMsg(quat, q);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "mcl4pfc");
  ros::NodeHandle nh;
  MclNode mcl_node(nh);
  ros::spin();
}
