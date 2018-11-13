#include <mcl4pfc/particle.h>


Particle::Particle()
{
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
