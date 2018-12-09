#include <ros/ros.h>
#include <mcl4pfc/mcl.h>
#include <mcl4pfc/particle.h>
#include <vector>
#include <cmath>
#include <random>
#include <algorithm>
#include <iterator>


Mcl::Mcl() :
  particle_num_(5),
  weight_sum_(0.0),
  state_transition_sigma_(0.2),
  likelihood_field_sigma_(0.1),
  likelihood_field_(1, 0),
  resolution_(0.01),
  width_(1),
  height_(1)
{
  double initial_pose[] = {0, 0, 0};
  particles_.resize(particle_num_);
  for (int i = 0; i < particle_num_; i++)
  {
    particles_[i] = Particle(initial_pose, 1.0/particle_num_);
  }
}

Mcl::Mcl(int particle_num, double (&initial_pose)[3],
         double state_transition_sigma) :
  particle_num_(particle_num),
  weight_sum_(0.0),
  state_transition_sigma_(state_transition_sigma),
  likelihood_field_sigma_(0.1),
  likelihood_field_(1, 0),
  resolution_(0.01),
  width_(1),
  height_(1)
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

void Mcl::setLikelihoodField(const std::vector<int8_t>& map_image,
                             float resolution,
                             uint32_t width, uint32_t height,
                             double sigma)
{
  likelihood_field_sigma_ = sigma;
  width_ = width;
  height_ = height;
  likelihood_field_.resize(map_image.size());

  // 障害物の座標リスト
  std::vector<std::pair<float, float> > obstacles_pos;
  std::pair<float, float> pos;
  size_t idx;
  for (auto itr = map_image.begin(); itr != map_image.end(); ++itr)
  {
    if ((float)(*itr) >= 100)
    {
      idx = std::distance(map_image.begin(), itr);
      // セルの中心座標を格納
      pos.first = (float)((idx%width));
      pos.second = (float)((idx/width));
      obstacles_pos.push_back(pos);
    }
  }

  // すべての座標の障害物との最小距離を計算
  if (obstacles_pos.size() != 0)
  {
    std::vector<float> dists_tmp(obstacles_pos.size());
    float dist = 1000.0;
    float x = 0;
    float y = 0;
    for (auto itr = map_image.begin(); itr != map_image.end(); ++itr)
    {
      idx = std::distance(map_image.begin(), itr);
      x = (float)(idx%width);
      y = (float)(idx/width);
      for (int i = 0; i < obstacles_pos.size(); i++)
      {
        dists_tmp[i] =
            std::hypot(x-obstacles_pos[i].first, y-obstacles_pos[i].second);
      }
      dist = *std::min_element(dists_tmp.begin(), dists_tmp.end()) * resolution;
      likelihood_field_[idx] =
          (float)(exp(-pow(dist, 2) / (2 * pow(sigma, 2))) /
                  (sqrt(2 * M_PI) * sigma));
    }
  }
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
  float pnu;
  float pomega;

  //乱数生成クラスj
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
    pnu = nu + ns[0]*sqrt(fabsf(nu)/dt) + ns[1]*sqrt(fabsf(omega)/dt);
    pomega = omega + ns[2]*sqrt(fabsf(nu)/dt) + ns[3]*sqrt(fabsf(omega)/dt);
    float pu[2] = {pnu, pomega};
    Mcl::transitionState(p->pose_, pu, dt);
  }
}

void Mcl::updateWithObservation(std::vector<float>& ranges,
                                std::vector<float>& angles)
{
  weight_sum_ = 0;
  for (auto p = particles_.begin(); p != particles_.end(); ++p)
  {
    p->weight_ = Mcl::calcLikelihood(ranges, angles, p->pose_);
    weight_sum_ += p->weight_;
  }
  Mcl::resampling();
}

double Mcl::calcLikelihood(std::vector<float>& ranges,
                           std::vector<float>& angles, double (&pose)[3])
{
  double q = 1;
  double x = pose[0];
  double y = pose[1];
  double theta = pose[2];
  double x_z;
  double y_z;
  double x_idx;
  double y_idx;
  double idx;

  //for (auto k = scans.begin(); k != scans.end(); ++k)
  //{
  //  if (!std::isinf(*k))
  //  {
  //    x_z = pose[0] + *k * cos(theta + *k);
  //    y_z = pose[1] + *k * sin(theta + *k);
  //    std::cout << x_z << ", " << y_z << std::endl;
  //    //////////////////////////////////////////////
  //  }
  //}

  for (int k = 0; k < 1; k++)
  {
    if (!std::isinf(ranges[k]))
    {
      x_z = pose[0] + ranges[k] * cos(theta + 0);
      y_z = pose[1] + ranges[k] * sin(theta + 0);
      x_idx = (int)(x_z / resolution_);
      y_idx = (int)(y_z / resolution_);

      if ((x_idx >= 0) && (x_idx < width_) &&
          (y_idx >= 0) && (y_idx < height_))
      {
        idx = x_idx + y_idx * width_;
        q *= likelihood_field_[idx];
      }
    }
  }

  return q;
}

void Mcl::resampling()
{
  std::vector<Particle> particles(particle_num_);

  std::random_device rnd;
  std::mt19937 mt(rnd());
  std::uniform_real_distribution<double> dist(0, weight_sum_/particle_num_);
  double r = dist(mt);
  double c = particles_[0].weight_;
  double i = 0;

  double U;
  for (int m = 0; m < particle_num_; m++)
  {
    U = r + m * weight_sum_/particle_num_;
    while (U > c)
    {
      i = i + 1;
      c = c + particles_[i].weight_;
    }
    particles[m] = Particle(particles_[i].pose_, 1/particle_num_);
  }
  particles_ = particles;
}
