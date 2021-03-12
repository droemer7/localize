#include <chrono>
#include <float.h>

#include "mcl/util.h"

using namespace localize;

const std::string localize::DATA_PATH = "/home/dane/sw/ros/master/src/localize/mcl/data/";

Particle::Particle(const double x,
                   const double y,
                   const double th,
                   const double weight
                  ):
  x_(x),
  y_(y),
  th_(th),
  weight_(weight)
{}

ParticleDistribution::ParticleDistribution(const size_t num_particles_max) :
  particles_(num_particles_max),
  num_particles_(0)
{}

ParticleDistribution::ParticleDistribution(const ParticleVector& particles,
                                           const size_t num_particles
                                          ) :
  particles_(particles),
  num_particles_(num_particles)
{}

Particle& ParticleDistribution::particle(size_t p)
{
  return particles_[p];
}

size_t ParticleDistribution::size() const
{
  return num_particles_;
}

void ParticleDistribution::resize(size_t num_particles)
{
  if (num_particles > particles_.size()) {
    particles_.resize(num_particles);
  }
  num_particles_ = num_particles;
}

double ParticleDistribution::weightSum()
{
  double sum = 0.0;

  for (size_t i = 0; i < size(); ++i) {
    sum += particles_[i].weight_;
  }
  return sum;
}

double ParticleDistribution::weightAvg()
{
  double avg = 0.0;

  if (size() > 0) {
    avg = weightSum() / size();
  }
  return avg;
}

double ParticleDistribution::weightVar()
{
  double var = 0.0;
  double avg = weightAvg();

  for (size_t i = 0; i < size(); ++i) {
    var += (  (particles_[i].weight_ - avg)
            * (particles_[i].weight_ - avg)
           );
  }
  return var;
}

void ParticleDistribution::normWeights()
{
  double sum = weightSum();
  double norm = sum > 0.0 ? 1 / sum : 0.0;

  for (size_t i = 0; i < num_particles_; ++i) {
    particles_[i].weight_ *= norm;
  }
  return;
}

Ray::Ray(const float range,
         const float th
        ) :
  range_(range),
  th_(th)
{}

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

ParticleHistogram::ParticleHistogram(const double x_res,
                                     const double y_res,
                                     const double th_res,
                                     const Map& map
                                    ) :
  x_res_(x_res),
  y_res_(y_res),
  th_res_(th_res),
  x_size_(std::round(map.width * map.scale / x_res_)),
  y_size_(std::round(map.height * map.scale / y_res_)),
  th_size_(std::round((M_2PI + map.th_origin) / th_res)),
  x_origin_(map.x_origin),
  y_origin_(map.y_origin),
  th_origin_(map.th_origin),
  hist_(x_size_,
        std::vector<std::vector<int>>(y_size_,
                                      std::vector<int>(th_size_, false)
                                     )
       ),
  count_(0)
{}

void ParticleHistogram::update(const Particle& particle)
{
  // Calculate indexes
  size_t x_i = std::min(std::max(0.0, (particle.x_ - x_origin_) / x_res_),
                        static_cast<double>(x_size_ - 1)
                       );
  size_t y_i = std::min(std::max(0.0, (particle.y_ - y_origin_) / y_res_),
                        static_cast<double>(y_size_ - 1)
                       );
  size_t th_i = std::min(std::max(0.0, (particle.th_ - th_origin_) / th_res_),
                         static_cast<double>(th_size_ - 1)
                        );
  // Update histogram
  if (!hist_[x_i][y_i][th_i]) {
    hist_[x_i][y_i][th_i] = true;
    ++count_;
  }
  return;
}

size_t ParticleHistogram::count()
{
  return count_;
}

void ParticleHistogram::reset()
{
  for (size_t i = 0; i < hist_.size(); ++i) {
    for (size_t j = 0; j < hist_[0].size(); ++j) {
      for (size_t k = 0; k < hist_[0][0].size(); ++k) {
        hist_[i][j][k] = false;
      }
    }
  }
  count_ = 0;
}

SmoothedValue::SmoothedValue(const double rate,
                             const double val
                            ) :
  rate_(rate),
  val_(val)
{}

double SmoothedValue::update(const double val)
{
  val_ += rate_ * (val - val_);
  return val_;
}

double SmoothedValue::val()
{
  return val_;
}

RNG::RNG()
{
  // Initialize random number generator
  // Source: https://stackoverflow.com/a/13446015
  std::random_device dev;
  std::chrono::_V2::system_clock::duration time = std::chrono::_V2::system_clock::now().time_since_epoch();
  std::mt19937::result_type time_seconds = std::chrono::duration_cast<std::chrono::seconds>(time).count();
  std::mt19937::result_type time_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time).count();
  std::mt19937::result_type seed = dev() ^ (time_seconds + time_microseconds);
  gen_.seed(seed);
}