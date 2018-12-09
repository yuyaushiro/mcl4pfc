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
  nh_(nh),
  i_(0)
{
  cmd_vel_sub_ = nh_.subscribe<geometry_msgs::Twist>("/cmd_vel", 10,
                                                     &MclNode::cmdVelCb, this);
  scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>("/scan", 1,
                                                    &MclNode::scanCb, this);
  map_sub_ = nh_.subscribe<nav_msgs::OccupancyGrid>("/map", 1,
                                                    &MclNode::mapCb, this);
  particlecloud_pub_ = nh_.advertise<geometry_msgs::PoseArray>("/particlecloud",
                                                               10);
  int particle_num = 10000;
  double initial_pose[] = {3.0, 2.5, 0.0};
  double state_transition_sigma = 0.02;
  double x_range[] = {2.0, 4.0};
  double y_range[] = {2.0, 4.0};
  double theta_range[] = {-M_PI, M_PI};
  //mcl_ = Mcl(particle_num, state_transition_sigma, initial_pose);
  mcl_ = Mcl(particle_num, state_transition_sigma,
             x_range, y_range, theta_range);

  double goal_position[] = {3.0, 3.7};
  double goal_radius = 0.1;
  mcl_.setGoal(goal_position, goal_radius);
}

MclNode::~MclNode()
{
}

void MclNode::cmdVelCb(const geometry_msgs::TwistConstPtr& twist)
{
  nu_ = twist->linear.x;
  omega_ = twist->angular.z;
  if (nu_ != 0 || omega_ != 0)
  {
    i_++;
  }
  float u[] = {nu_, omega_};

  MclNode::pubParticlecloud();
  mcl_.updateWithMotion(u, 1.0);
}

void MclNode::scanCb(const sensor_msgs::LaserScanConstPtr& scan)
{
  if (i_ > 1)
  {
    ranges_ = scan->ranges;

    angle_min_ = scan->angle_min;
    angle_max_ = scan->angle_max;
    angle_increment_ = scan->angle_increment;
    angles_.resize(ranges_.size());
    for (int i = 0; i < angles_.size(); i++)
    {
      angles_[i] = i*angle_increment_;
    }

    mcl_.updateWithObservation(ranges_, angles_);
    i_ = 0;
  }
}

void MclNode::mapCb(const nav_msgs::OccupancyGridConstPtr& map)
{
  std::vector<int8_t> map_image = map->data;
  float resolution = map->info.resolution;
  uint32_t width = map->info.width;
  uint32_t height = map->info.height;
  mcl_.setLikelihoodField(map_image, resolution, width, height, 0.05);
}

void MclNode::pubParticlecloud()
{
  geometry_msgs::PoseArray pose_array;
  pose_array.header.stamp = ros::Time::now();
  pose_array.header.frame_id = "/map";
  for (auto itr = mcl_.particles_.begin(); itr != mcl_.particles_.end(); ++itr)
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
