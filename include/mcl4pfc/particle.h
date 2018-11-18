#pragma once


class Particle
{
 public:
  Particle();
  Particle(double (&pose)[3], double weight);
  ~Particle();

  double pose_[3];
  double weight_;
};
