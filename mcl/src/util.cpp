#include <chrono>
#include <float.h>

#include "mcl/util.h"

using namespace localize;

const std::string localize::DATA_PATH = "/home/dane/sw/ros/master/src/localize/mcl/data/";

static const double WEIGHT_AVG_SLOW_RATE = 0.05;
static const double WEIGHT_AVG_FAST_RATE = 0.30;

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
  weight_normed_(weight_normed)
{}

ParticleDistribution::ParticleDistribution() :
  size_(0),
  weight_sum_(0.0),
  weight_avg_(0.0),
  weight_avg_slow_(0.0, WEIGHT_AVG_SLOW_RATE),
  weight_avg_fast_(0.0, WEIGHT_AVG_FAST_RATE),
  weight_var_(0.0)
{}

ParticleDistribution::ParticleDistribution(const size_t max_size) :
  ParticleDistribution()
{
  if (max_size > 0) {
    particles_.reserve(max_size);
    particles_.resize(max_size);
  }
}

ParticleDistribution::ParticleDistribution(const ParticleVector& particles,
                                           const size_t size
                                          ) :
  ParticleDistribution()
{
  update(particles, size);
}

void ParticleDistribution::update()
{
  if (size() > 0) {
    // Calculate weight sum
    weight_sum_ = 0.0;

    for (size_t i = 0; i < size(); ++i) {
      weight_sum_ += particles_[i].weight_;
    }
    // Calculate and update weight averages
    weight_avg_ = weight_sum_ / size();
    weight_avg_slow_.update(weight_avg_);
    weight_avg_fast_.update(weight_avg_);

    double weight = 0.0;
    double weight_diff = 0.0;
    double weight_normalizer = weight_sum_ > 0.0 ? 1 / weight_sum_ : 0.0;

    // Calculate weight variance and update normalized weights
    for (size_t i = 0; i < size(); ++i) {
      weight = particles_[i].weight_;
      weight_diff = weight - weight_avg_;
      weight_var_ += weight_diff * weight_diff;
      particles_[i].weight_normed_ = weight * weight_normalizer;
    }
  }
  else {
    weight_sum_ = 0.0;
    weight_avg_ = 0.0;
    weight_avg_slow_.reset();
    weight_avg_fast_.reset();
    weight_var_ = 0.0;
  }
  printf("weight_sum_ = %.4e\n", weight_sum_);
  printf("weight_avg_ = %.4e\n", weight_avg_);
  printf("weight_avg_fast_ = %.4e\n", double(weight_avg_fast_));
  printf("weight_avg_slow_ = %.4e\n", double(weight_avg_slow_));
  printf("weight_var_ = %.4e\n", weight_var_);
}


void ParticleDistribution::update(const ParticleVector& particles,
                                  const size_t size
                                 )
{
  if (size > particles_.size()) {
    particles_.resize(size);
  }
  size_ = size;

  for (size_t i = 0; i < this->size(); ++i) {
    particles_[i] = particles[i];
  }
  update();
}

Particle& ParticleDistribution::particle(size_t p)
{
  return particles_[p];
}

size_t ParticleDistribution::size() const
{
  return size_;
}

double ParticleDistribution::confidenceAvg() const
{
  return weight_avg_;
}

double ParticleDistribution::confidenceVar() const
{
  return weight_var_;
}

double ParticleDistribution::confidenceRatio() const
{
  double ratio = 0.0;
  if (weight_avg_slow_ > 0.0) {
    ratio = std::min(1.0, weight_avg_fast_ / weight_avg_slow_);
  }
  return ratio;
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
  hist_(x_size_ * y_size_ * th_size_, false),
  count_(0)
{}

bool ParticleHistogram::update(const Particle& particle)
{
  bool count_inc = false;

  // Calculate index
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
  if (!cell(x_i, y_i, th_i)) {
    cell(x_i, y_i, th_i) = true;
    count_inc = true;
    ++count_;
  }
  return count_inc;
}

size_t ParticleHistogram::count() const
{
  return count_;
}

void ParticleHistogram::clear()
{
  if (count_ > 0) {
    std::fill(hist_.begin(), hist_.end(), false);
    count_ = 0;
  }
}

std::vector<bool>::reference ParticleHistogram::cell(const size_t x_i,
                                                     const size_t y_i,
                                                     const size_t th_i
                                                    )
{
  return hist_[x_i * y_size_ * th_size_ + y_i * th_size_ + th_i];
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