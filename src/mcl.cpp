#include <ros/ros.h>
#include <mcl4pfc/mcl.h>
#include <mcl4pfc/particle.h>
#include <vector>
#include <algorithm>
#include <iterator>


Mcl::Mcl() :
  particle_num_(5),
  state_transition_sigma_(0.2),
  likelihood_field_sigma_(0.2)
{
  double initial_pose[] = {0, 0, 0};
  particles_.resize(particle_num_);
  for (int i = 0; i < particle_num_; i++)
  {
    particles_[i] = Particle(initial_pose, 1.0/particle_num_);
  }
}

Mcl::Mcl(int particle_num, double (&initial_pose)[3],
         double state_transition_sigma, double likelihood_field_sigma) :
  particle_num_(particle_num),
  state_transition_sigma_(state_transition_sigma),
  likelihood_field_sigma_(likelihood_field_sigma)
{
  particles_.resize(particle_num_);
  for (int i = 0; i < particle_num_; i++)
  {
    particles_[i] = Particle(initial_pose, 1.0/particle_num_);
  }
  ROS_INFO_STREAM(particles_[0].weight_);
}

Mcl::~Mcl()
{
}

void Mcl::transitionState()
{
}

void Mcl::updateWithMotion()
{
}

void Mcl::updateWithObservation()
{
}

void Mcl::resampling()
{
}

void Mcl::calcLikelihood()
{
}
