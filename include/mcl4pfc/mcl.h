#pragma once

#include <ros/ros.h>
#include <vector>
#include <algorithm>
#include <iterator>
#include <mcl4pfc/particle.h>


class Mcl
{
 public:
  Mcl();
  Mcl(int particle_num, double state_transition_sigma,
      double (&initial_pose)[3]);
  Mcl(int particle_num, double state_transition_sigma,
      double (&x_range)[2], double (&y_range)[2], double (&theta_range)[2]);
  ~Mcl();
  void setLikelihoodField(const std::vector<int8_t>& map_image,
                          float resolution,
                          uint32_t width, uint32_t height,
                          double likelihood_field_sigma);
  void setGoal(double (&postion)[2], double radius);
  void updateWithMotion(float (&u)[2], double dt);
  void updateWithObservation(std::vector<float>& ranges,
                             std::vector<float>& angles);

  std::vector<Particle> particles_;
  std::vector<float> likelihood_field_;

 private:
  void transitionState(double (&pose)[3], float (&u)[2], double dt);
  double calcLikelihood(std::vector<float>& ranges, 
                        std::vector<float>& angles, double (&pose)[3]);
  void resampling();

  int particle_num_;
  double weight_sum_;
  double state_transition_sigma_;
  double likelihood_field_sigma_;
  float resolution_;
  uint32_t width_;
  uint32_t height_;
  double goal_position_[2];
  double goal_radius_;
};
