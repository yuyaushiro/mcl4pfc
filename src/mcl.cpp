#include <ros/ros.h>
#include <mcl4pfc/mcl.h>
#include <mcl4pfc/particle.h>
#include <vector>
#include <algorithm>
#include <iterator>


Mcl::Mcl(int particle_num, int& initial_pose,
         float state_transition_sigma, float likelihood_field_sigma);
{
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

void Mcl::setInitialPose()
{
}
