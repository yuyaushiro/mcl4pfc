#pragma once


class Particle
{
 public:
  Particle();
  Particle(double (&pose)[3], double weight);
  ~Particle();

 //private:
  double pose_[3];
  double weight_;
};
