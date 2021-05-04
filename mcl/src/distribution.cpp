#include "mcl/distribution.h"

static const double WEIGHT_AVG_CREEP_RATE = 0.01;  // Weight average smoothing rate, very slow
static const double WEIGHT_AVG_SLOW_RATE = 0.40;   // Weight average smoothing rate, slow
static const double WEIGHT_AVG_FAST_RATE = 0.75;   // Weight average smoothing rate, fast

using namespace localize;

// ========== ParticleDistribution ========== //
ParticleDistribution::ParticleDistribution(const size_t max_count,
                                           const Map& map
                                          ) :
  count_(0),
  weight_sum_(0.0),
  weight_avg_creep_(WEIGHT_AVG_CREEP_RATE),
  weight_avg_slow_(WEIGHT_AVG_SLOW_RATE),
  weight_avg_fast_(WEIGHT_AVG_FAST_RATE),
  weight_var_(0.0),
  weight_std_dev_(0.0),
  weight_relative_std_dev_(0.0),
  sample_s_(0),
  sample_step_(0.0),
  sample_sum_(0.0),
  sample_sum_target_(0.0),
  hist_(map),
  prob_(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()))
{
  if (max_count > 0) {
    particles_.resize(max_count);
  }
}

void ParticleDistribution::populate(const ParticleVector& particles, const size_t count)
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

void ParticleDistribution::update(const ParticleVector& particles, const size_t count)
{
  populate(particles, count);
  calcWeightStats();
  resetSampler();
}

void ParticleDistribution::update()
{
  calcWeightStats();
  resetSampler();
}

const ParticleVector& ParticleDistribution::estimates()
{
  hist_.reset();

  for (size_t i = 0; i < count_; ++i) {
    hist_.add(particles_[i]);
  }
  return hist_.estimates();
}

Particle& ParticleDistribution::particle(const size_t i)
{ return particles_[i]; }

size_t ParticleDistribution::count() const
{ return count_; }

const Particle& ParticleDistribution::sample()
{
  // If we've reached the end, wrap back around
  if (sample_s_ + 1 >= count_) {
    resetSampler();
  }
  // Sum weights until we reach the target
  while (sample_sum_ < sample_sum_target_) {
    sample_sum_ += particles_[++sample_s_].weight_normed_;
  }
  // Increase target for the next sample
  sample_sum_target_ += sample_step_;

  return particles_[sample_s_];
}

void ParticleDistribution::resetSampler()
{
  sample_s_ = 0;

  if (count_ > 0) {
    sample_step_ = 1.0 / count_;
    sample_sum_target_ = prob_(rng_.engine()) * sample_step_;
    sample_sum_ = particles_[0].weight_normed_;
  }
  else {
    sample_step_ = 0.0;
    sample_sum_target_ = 0.0;
    sample_sum_ = 0.0;
  }
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
    weight_avg_curr_ = weight_sum_ / count_;
    weight_avg_creep_.update(weight_avg_curr_);
    weight_avg_slow_.update(weight_avg_curr_);
    weight_avg_fast_.update(weight_avg_curr_);

    // Reinitialize weight variance, normalizer and histogram
    weight_var_ = 0.0;
    double weight_normalizer = weight_sum_ > 0.0 ? 1.0 / weight_sum_ : 0.0;
    double weight_diff = 0.0;

    for (size_t i = 0; i < count_; ++i) {
      // Normalize weight
      particles_[i].weight_normed_ = particles_[i].weight_ * weight_normalizer;

      // Calculate weight variance sum
      weight_diff = particles_[i].weight_ - weight_avg_curr_;
      weight_var_ += weight_diff * weight_diff;
    }
    weight_var_ /= count_;

    // Calculate standard deviation
    weight_std_dev_ = std::sqrt(weight_var_);
    weight_relative_std_dev_ = weight_avg_curr_ > 0.0 ? weight_std_dev_ / weight_avg_curr_ : 0.0;
  }
  else {
    // No particles, reinitialize
    weight_avg_curr_ = 0.0;
    weight_avg_creep_.reset(0.0);
    weight_avg_slow_.reset(0.0);
    weight_avg_fast_.reset(0.0);
    weight_sum_ = 0.0;
    weight_var_ = 0.0;
    weight_std_dev_ = 0.0;
    weight_relative_std_dev_ = 0.0;
  }
}

void ParticleDistribution::resetWeightAvgHistory()
{
  weight_avg_creep_.reset(weight_avg_curr_);
  weight_avg_slow_.reset(weight_avg_curr_);
  weight_avg_fast_.reset(weight_avg_curr_);
}

double ParticleDistribution::weightAvgCurr() const
{ return weight_avg_curr_;}

double ParticleDistribution::weightAvgFast() const
{ return weight_avg_fast_;}

double ParticleDistribution::weightAvgSlow() const
{ return weight_avg_slow_;}

double ParticleDistribution::weightAvgCreep() const
{ return weight_avg_creep_;}

double ParticleDistribution::weightAvgRatio() const
{
  double ratio = 1.0;
  if (weight_avg_creep_ > 0.0) {
    ratio = std::min(1.0, weight_avg_slow_ / weight_avg_creep_);
  }
  return ratio;
}

double ParticleDistribution::weightVar() const
{ return weight_var_;}

double ParticleDistribution::weightStdDev() const
{ return weight_std_dev_;}

double ParticleDistribution::weightRelStdDev() const
{ return weight_relative_std_dev_;}

void ParticleDistribution::printWeightStats() const
{
  printf("Weight average fast = %.2e\n", weightAvgFast());
  printf("Weight average slow = %.2e\n", weightAvgSlow());
  printf("Weight average creep = %.2e\n", weightAvgCreep());
  printf("Weight ratio = %.2f\n", weightAvgRatio());
  printf("Weight relative std dev = %.2e\n", weightRelStdDev());
}