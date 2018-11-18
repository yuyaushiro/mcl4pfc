#include <mcl4pfc/particle.h>


Particle::Particle() :
  weight_(0.0)
{
  for (int i = 0; i < 3; i++)
  {
    pose_[i] = 0.0;
  }
}

Particle::Particle(double (&pose)[3], double weight) :
  weight_(weight)
{
  for (int i = 0; i < 3; i++)
  {
    pose_[i] = pose[i];
  }
}

Particle::~Particle()
{
}
