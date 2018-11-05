#pragma once

#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <iterator>
#include <mcl4pfc/particle.h>


class Mcl
{
 public:
  Mcl(int particle_num, const int& initial_pose,
      float state_transition_sigma, float likelihood_field_sigma);
  ~Mcl();
  void updateWithMotion();
  void updateWithObservation();
  void setInitialPose();
 private:
  void transitionState();
  void resampling();
  void calcLikelihood();

 private:
  std::vector<Particle> particles_;
  int particle_num_;
  int initial_pose_[3];
  float state_transition_sigma_;
  float likelihood_field_sigma_;
};
