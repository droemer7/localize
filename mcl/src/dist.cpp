#include "mcl/dist.h"

static const double WEIGHT_AVG_SLOW_RATE = 0.01; // 0.05
static const double WEIGHT_AVG_FAST_RATE = 0.30;

using namespace localize;

ParticleDistribution::ParticleDistribution() :
  count_(0),
  weight_sum_(0.0),
  weight_avg_(-1.0),
  weight_avg_slow_(0.0, WEIGHT_AVG_SLOW_RATE),
  weight_avg_fast_(0.0, WEIGHT_AVG_FAST_RATE),
  weight_var_(0.0),
  weight_std_dev_(0.0),
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
  // Resize particle vector if it needs to be bigger
  if (count > particles_.size()) {
    particles_.resize(count);
  }
  // Update number of particles in use
  count_ = count;

  for (size_t i = 0; i < this->count(); ++i) {
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
  // If we've reached the end, wrap back around
  if (sample_s_ + 1 >= count()) {
    resetSampler();
  }
  // Sum weights until we reach the target
  while (sample_sum_ < sample_sum_target_) {
    // Update the weight sum
    if (sample_s_ + 1 >= count()) {
      printf("!!!!! Segmentation fault warning: sampling past particles count !!!!!\n");
    }
    sample_sum_ += particles_[++sample_s_].weight_normed_;
  }
  // Update target we'll use for the next sample
  sample_sum_target_ += sample_step_;
  return particle(sample_s_);
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
  if (count() > 0) {
    // Calculate weight sum
    weight_sum_ = 0.0;

    for (size_t i = 0; i < count(); ++i) {
      weight_sum_ += particles_[i].weight_;
    }
    // Calculate and update weight averages
    if (weight_avg_ >= 0.0) {
      weight_avg_ = weight_sum_ / count();
      weight_avg_slow_.update(weight_avg_);
      weight_avg_fast_.update(weight_avg_);
    }
    // Initialize weight averages
    else {
      weight_avg_ = weight_sum_ / count();
      weight_avg_slow_.reset(weight_avg_);
      weight_avg_fast_.reset(weight_avg_);
    }

    double weight_normalizer = weight_sum_ > DBL_MIN ? 1 / weight_sum_ : 0.0;
    double weight_avg_normed = 1.0 / count();
    double weight_diff = 0.0;

    // Normalize weights and calculate weight variance based on the normalized weights
    for (size_t i = 0; i < count(); ++i) {
      particles_[i].weight_normed_ = particles_[i].weight_ * weight_normalizer;
      weight_diff = particles_[i].weight_normed_ - weight_avg_normed;
      weight_var_ += weight_diff * weight_diff;
    }
    weight_var_ /= count();
    if (weight_var_ > DBL_MIN) {
      weight_std_dev_ = std::sqrt(weight_var_);
    }
    else {
      weight_std_dev_ = 0.0;
    }
  }
  else {
    // No particles, zero initialize
    weight_sum_ = 0.0;
    weight_avg_ = 0.0;
    weight_avg_slow_.reset(0.0);
    weight_avg_fast_.reset(0.0);
    weight_var_ = 0.0;
  }
  printf("\n");
  printf("Weight average = %.2e\n", weightAvg());
  printf("Weight average [fast] = %.2e\n", static_cast<double>(weight_avg_fast_));
  printf("Weight average [slow] = %.2e\n", static_cast<double>(weight_avg_slow_));
  printf("Weight ratio = %.2f\n", weightAvgRatio());
  printf("Weight variance = %.2e\n", weightVar());
  printf("Weight std dev = %.2e\n", weightStdDev());
}

void ParticleDistribution::resetSampler()
{
  if (count() > 0) {
    sample_s_ = 0;
    sample_step_ = 1.0 / count();
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