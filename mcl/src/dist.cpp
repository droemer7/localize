#include "mcl/dist.h"

static const double WEIGHT_AVG_SLOW_RATE = 0.01;
static const double WEIGHT_AVG_FAST_RATE = 0.20;

using namespace localize;

ParticleDistribution::ParticleDistribution() :
  count_(0),
  weight_sum_(0.0),
  weight_avg_(-1.0),
  weight_avg_slow_(0.0, WEIGHT_AVG_SLOW_RATE),
  weight_avg_fast_(0.0, WEIGHT_AVG_FAST_RATE),
  weight_var_(0.0),
  weight_std_dev_(0.0),
  weight_relative_std_dev_(0.0),
  sample_s_(0),
  sample_step_(0.0),
  sample_sum_(0.0),
  sample_sum_target_(0.0),
  prob_(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()))
{}

ParticleDistribution::ParticleDistribution(const size_t max_count) :
  ParticleDistribution()
{
  if (max_count > 0) {
    particles_.reserve(max_count);
    particles_.resize(max_count);
  }
}

ParticleDistribution::ParticleDistribution(const ParticleVector& particles,
                                           const size_t count
                                          ) :
  ParticleDistribution()
{
  update(particles, count);
}

void ParticleDistribution::copy(const ParticleVector& particles,
                                const size_t count
                               )
{
  // If count is 0 (specified or not), use particles input as is
  if (count == 0) {
    count_ = particles.size();
    particles_.resize(count_);
  }
  // Resize particle vector, but only if it needs to be bigger
  else if (count > particles_.size()) {
    count_ = count;
    particles_.resize(count_);
  }
  // Otherwise just update the count
  else {
    count_ = count;
  }
  // Copy particles over based on count
  for (size_t i = 0; i < count_; ++i) {
    particles_[i] = particles[i];
  }
}

void ParticleDistribution::update()
{
  // printf("Last sample taken = %lu\n", sample_s_);  // TBD remove
  calcWeightStats();
  resetSampler();
}

void ParticleDistribution::update(const ParticleVector& particles,
                                  const size_t count
                                 )
{
  copy(particles, count);
  update();
}

Particle& ParticleDistribution::particle(size_t p)
{
  return particles_[p];
}

Particle& ParticleDistribution::sample()
{
  // Sum weights until we reach the target
  while (sample_sum_ < sample_sum_target_) {
    sample_sum_ += particles_[++sample_s_].weight_normed_;

    // If we've reached the end, wrap back around
    if (sample_s_ + 1 >= count_) {
      resetSampler();
    }
  }
  // Increase target for the next sample
  sample_sum_target_ += sample_step_;

  return particles_[sample_s_];
}

size_t ParticleDistribution::count() const
{
  return count_;
}

double ParticleDistribution::weightAvg() const
{
  return weight_avg_;
}

double ParticleDistribution::weightVar() const
{
  return weight_var_;
}

double ParticleDistribution::weightStdDev() const
{
  return weight_std_dev_;
}

double ParticleDistribution::weightRelativeStdDev() const
{
  return weight_relative_std_dev_;
}

double ParticleDistribution::weightAvgRatio() const
{
  double ratio = 0.0;
  if (weight_avg_slow_ > 0.0) {
    ratio = std::min(1.0, weight_avg_fast_ / weight_avg_slow_);
  }
  return ratio;
}

void ParticleDistribution::calcWeightStats()
{
  if (count_ > 0) {
    // Calculate weight sum
    weight_sum_ = 0.0;

    for (size_t i = 0; i < count_; ++i) {
      weight_sum_ += particles_[i].weight_;
    }
    // Calculate and update weight averages
    if (weight_avg_ >= 0.0) {
      weight_avg_ = weight_sum_ / count_;
      weight_avg_slow_.update(weight_avg_);
      weight_avg_fast_.update(weight_avg_);
    }
    // Initialize weight averages
    else {
      weight_avg_ = weight_sum_ / count_;
      weight_avg_slow_.reset(weight_avg_);
      weight_avg_fast_.reset(weight_avg_);
    }

    // Update normalized weights and calculate weight variance
    weight_var_ = 0.0;
    double weight_normalizer = weight_sum_ > 0.0 ? 1 / weight_sum_ : 0.0;
    //double weight_avg_normed = 1.0 / count_; // TBD remove or restore
    double weight_diff = 0.0;

    for (size_t i = 0; i < count_; ++i) {
      particles_[i].weight_normed_ = particles_[i].weight_ * weight_normalizer;
      weight_diff = particles_[i].weight_ - weight_avg_;
      weight_var_ += weight_diff * weight_diff;
    }
    weight_var_ /= count_;

    // Calculate standard deviation
    weight_std_dev_ = weight_var_ > 0.0 ? std::sqrt(weight_var_) : 0.0;
    weight_relative_std_dev_ = weight_avg_ > 0.0 ? weight_std_dev_ / weight_avg_ : 0.0;
  }
  else {
    // No particles, zero initialize
    weight_sum_ = 0.0;
    weight_avg_ = 0.0;
    weight_avg_slow_.reset(0.0);
    weight_avg_fast_.reset(0.0);
    weight_var_ = 0.0;
    weight_std_dev_ = 0.0;
    weight_relative_std_dev_ = 0.0;
  }
  printf("Weight average = %.2e\n", weightAvg());
  printf("Weight average [fast] = %.2e\n", static_cast<double>(weight_avg_fast_));
  printf("Weight average [slow] = %.2e\n", static_cast<double>(weight_avg_slow_));
  printf("Weight ratio = %.2f\n", weightAvgRatio());
  printf("Weight variance = %.2e\n", weightVar());
  printf("Weight std dev = %.2e\n", weightStdDev());
  printf("Weight relative std dev = %.2e\n", weightRelativeStdDev());
}

void ParticleDistribution::resetSampler()
{
  if (count_ > 0) {
    sample_s_ = 0;
    sample_step_ = 1.0 / count_;
    sample_sum_target_ = prob_(rng_.engine()) * sample_step_;
    sample_sum_ = particles_[0].weight_normed_;
  }
  else {
    sample_s_ = 0;
    sample_step_ = 0.0;
    sample_sum_target_ = 0.0;
    sample_sum_ = 0.0;
  }
}