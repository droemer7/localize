#include <algorithm>
#include <chrono>
#include <limits>
#include <math.h>

#include "mcl/common.h"

using namespace localize;

const unsigned int localize::SENSOR_TH_SAMPLE_COUNT = 8;
const std::string localize::DATA_PATH = "/home/dane/sw/ros/master/src/localize/mcl/data/";

// ========== Particle ========== //
Particle::Particle(const double x,
                   const double y,
                   const double th,
                   const double weight,
                   const double weight_normed
                  ):
  x_(x),
  y_(y),
  th_(th),
  weight_(weight),
  weight_normed_(weight_normed),
  weights_(SENSOR_TH_SAMPLE_COUNT, 0.0)
{}

int Particle::compare(const Particle& lhs, const Particle& rhs) const
{
  int result;
  if (lhs.weight_normed_ < rhs.weight_normed_) {
    result = -1;
  }
  else if (lhs.weight_normed_ > rhs.weight_normed_) {
    result = 1;
  }
  else { // lhs.weight_normed_ == rhs.weight_normed_
    result = 0;
  }
  return result;
}

Ray::Ray(const float range,
         const float th
        ) :
  range_(range),
  th_(th)
{}
// ========== End Particle ========== //

// ========== RaySample ========== //
RaySample::RaySample(const float range,
                     const float th,
                     const double weight_new_obj_sum,
                     const double weight_sum
                    ) :
  Ray(range, th),
  weight_new_obj_sum_(weight_new_obj_sum),
  weight_sum_(weight_sum)
{}

RaySample::RaySample(const Ray& ray) :
  RaySample(ray.range_, ray.th_)
{}
// ========== End RaySample ========== //

// ========== RayScan ========== //
RayScan::RayScan(RayVector rays,
                 float th_inc,
                 float t_inc,
                 float t_dur
                ) :
  rays_(rays),
  th_inc_(th_inc),
  t_inc_(t_inc),
  t_dur_(t_dur)
{}

RayScan::RayScan(size_t num_rays) :
  RayScan()
{
  rays_.resize(num_rays);
}
// ========== End RayScan ========== //

// ========== Map ========== //
// Note: This was derived from RangeLib author's definition of PyOMap in RangLibc.pyx
Map::Map(const unsigned int width,
         const unsigned int height,
         const float x_origin,
         const float y_origin,
         const float th_origin,
         const float scale,
         const std::vector<int8_t> data
        ) :
  ranges::OMap(height, width)
{
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      if (data[i * width + j] > 10) {
        grid[i][j] = true;
      }
    }
  }
  this->x_origin = x_origin;
  this->y_origin = y_origin;
  this->th_origin = -th_origin;
  this->sin_th = std::sin(-th_origin);
  this->cos_th = std::cos(-th_origin);
  this->scale = scale;
}

bool Map::occupied(float x, float y) const
{
  rosWorldToGrid(x, y);

  return isOccupiedNT(x, y);
}
// ========== Map ========== //

// ========== ParticleRandomSampler ========== //
ParticleRandomSampler::ParticleRandomSampler(const Map& map) :
  map_(map),
  x_dist_(map_.x_origin, map_.width * map_.scale + map_.x_origin),
  y_dist_(map_.y_origin, map_.height * map_.scale + map_.y_origin),
  th_dist_(std::nextafter(-L_PI, std::numeric_limits<double>::max()),
           std::nextafter(L_PI, std::numeric_limits<double>::max())
          )
{}

Particle ParticleRandomSampler::operator()()
{
  Particle particle;
  bool occupied = true;

  // Regenerate x & y until free space is found
  while (occupied) {
    particle.x_ = x_dist_(rng_.engine());
    particle.y_ = y_dist_(rng_.engine());
    occupied = map_.occupied(particle.x_, particle.y_);
  }
  // Any theta is allowed
  particle.th_ = th_dist_(rng_.engine());

  // Particle weight is 0.0 until determined by the sensor model
  return particle;
}
// ========== End ParticleRandomSampler ========== //