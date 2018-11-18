#include <ros/ros.h>
#include <mcl4pfc/mcl.h>
#include <mcl4pfc/particle.h>
#include <vector>
#include <random>
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
}

Mcl::~Mcl()
{
}

void Mcl::transitionState(double (&pose)[3], float (&u)[2], double dt)
{
  double t0 = pose[2];
  float nu = u[0];
  float omega = u[1];

  if (omega == 0)
  {
    pose[0] += nu*cos(t0) * dt;
    pose[1] += nu*sin(t0) * dt;
  }else{
    pose[0] += nu/omega*(sin(t0 + omega*dt) - sin(t0));
    pose[1] += nu/omega*(-cos(t0 + omega*dt) + cos(t0));
    pose[2] += omega * dt;
  }
}

void Mcl::updateWithMotion(float (&u)[2], double dt)
{
  float nu = u[0];
  float omega = u[1];

  //乱数生成クラス
  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::normal_distribution<> normal(0, state_transition_sigma_);
  float ns[4];

  for (auto p = particles_.begin(); p != particles_.end(); ++p)
  {
    // 乱数生成
    for (int i = 0; i < 4; i++)
    {
      ns[i] = normal(mt);
    }
    float pnu = nu + ns[0]*sqrt(fabsf(nu)/dt) + ns[1]*sqrt(fabsf(omega)/dt);
    float pomega =
        omega + ns[2]*sqrt(fabsf(nu)/dt) + ns[3]*sqrt(fabsf(omega)/dt);
    float pu[2] = {pnu, pomega};
    Mcl::transitionState(p->pose_, pu, dt);
  }
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
