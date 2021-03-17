#include "mcl/dist.h"

static const double WEIGHT_AVG_SLOW_RATE = 0.01; // 0.05
static const double WEIGHT_AVG_FAST_RATE = 0.30;

using namespace localize;

ParticleDistribution::ParticleDistribution() :
  size_(0),
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

ParticleDistribution::ParticleDistribution(const size_t max_size) :
  ParticleDistribution()
{
  if (max_size > 0) {
    particles_.reserve(max_size);
    particles_.resize(max_size);
  }
}

ParticleDistribution::ParticleDistribution(const ParticleVector& particles,
                                           const size_t num_particles
                                          ) :
  ParticleDistribution()
{
  update(particles, num_particles);
}

void ParticleDistribution::assign(const ParticleVector& particles,
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
}

void ParticleDistribution::update()
{
  calcWeightStats();
  resetSampler();
}

void ParticleDistribution::update(const ParticleVector& particles,
                                  const size_t size
                                 )
{
  assign(particles, size);
  update();
}

Particle& ParticleDistribution::particle(size_t p)
{
  return particles_[p];
}

Particle& ParticleDistribution::sample()
{
  // If we've reached the end, wrap back around
  if (sample_s_ + 1 >= size()) {
    resetSampler();
  }
  // Sum weights until we reach the target
  while (sample_sum_ < sample_sum_target_) {
    // Update the weight sum
    if (sample_s_ + 1 >= size()) {
      printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!! samples_s_ will overrun particles size\n");
    }
    sample_sum_ += particles_[++sample_s_].weight_normed_;
  }
  // Update target we'll use for the next sample
  sample_sum_target_ += sample_step_;
  printf("Chose particle %lu with weight normed = %.3e\n", sample_s_, particles_[sample_s_].weight_normed_);
  return particle(sample_s_);
}

size_t ParticleDistribution::size() const
{
  return size_;
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
  if (size() > 0) {
    // Calculate weight sum
    weight_sum_ = 0.0;

    for (size_t i = 0; i < size(); ++i) {
      weight_sum_ += particles_[i].weight_;
    }
    // Calculate and update weight averages
    if (weight_avg_ >= 0.0) {
      weight_avg_ = weight_sum_ / size();
      weight_avg_slow_.update(weight_avg_);
      weight_avg_fast_.update(weight_avg_);
    }
    // Initialize weight averages
    else {
      weight_avg_ = weight_sum_ / size();
      weight_avg_slow_.reset(weight_avg_);
      weight_avg_fast_.reset(weight_avg_);
    }

    double weight_normalizer = weight_sum_ > DBL_MIN ? 1 / weight_sum_ : 0.0;
    double weight_avg_normed = 1.0 / size();
    double weight_diff = 0.0;

    // Normalize weights and calculate weight variance based on the normalized weights
    for (size_t i = 0; i < size(); ++i) {
      particles_[i].weight_normed_ = particles_[i].weight_ * weight_normalizer;
      weight_diff = particles_[i].weight_normed_ - weight_avg_normed;
      weight_var_ += weight_diff * weight_diff;
    }
    weight_var_ /= size();
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
  printf("Weight sum = %.2e\n", weight_sum_);
  printf("Weight average = %.2e\n", weightAvg());
  printf("Weight average [fast] = %.2e\n", static_cast<double>(weight_avg_fast_));
  printf("Weight average [slow] = %.2e\n", static_cast<double>(weight_avg_slow_));
  printf("Weight ratio = %.2f\n", weightAvgRatio());
  printf("Weight variance = %.2e\n", weightVar());
  printf("Weight std dev = %.2e\n", weightStdDev());
}

void ParticleDistribution::resetSampler()
{
  if (size() > 0) {
    sample_s_ = 0;
    sample_step_ = 1.0 / size();
    sample_sum_target_ = prob_(rng_.engine()) * sample_step_;
    sample_sum_ = particle(0).weight_normed_;
    printf("sample_sum_target_ = %.3e\n", sample_sum_target_);
    printf("sample_sum_ = %.3e\n", sample_sum_);
  }
  else {
    sample_s_ = 0;
    sample_step_ = 0.0;
    sample_sum_target_ = 0.0;
    sample_sum_ = 0.0;
  }
}