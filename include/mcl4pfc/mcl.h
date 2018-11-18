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
  Mcl(int particle_num, double (&initial_pose)[3],
      double state_transition_sigma, double likelihood_field_sigma);
  ~Mcl();
  void updateWithMotion(float (&u)[2], double dt);
  void updateWithObservation();

  std::vector<Particle> particles_;

 private:
  void transitionState(double (&pose)[3], float (&u)[2], double dt);
  void resampling();
  void calcLikelihood();

  int particle_num_;
  double state_transition_sigma_;
  double likelihood_field_sigma_;
};
