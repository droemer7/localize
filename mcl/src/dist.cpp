#include "mcl/dist.h"

static const size_t NUM_ESTIMATES = 5;            // Number of pose estimates to provide
static const double HIST_POS_RES = 0.50;          // Histogram resolution for x and y position (meters per cell)
static const double HIST_TH_RES = M_PI / 4.0;     // Histogram resolution for heading angle (rad per cell)
static const double WEIGHT_AVG_CREEP_RATE = 0.01; // Weight average smoothing rate, very slow
static const double WEIGHT_AVG_SLOW_RATE = 0.25;  // Weight average smoothing rate, slow
static const double WEIGHT_AVG_FAST_RATE = 0.50;  // Weight average smoothing rate, fast

using namespace localize;

ParticleEstimateHistogramCell::ParticleEstimateHistogramCell() :
  x_(0.0),
  y_(0.0),
  th_(0.0),
  weight_normed_(0.0),
  count_(0)
{}

bool ParticleEstimateHistogramCell::compWeightGreater(const ParticleEstimateHistogramCell& cell_1,
                                                      const ParticleEstimateHistogramCell& cell_2
                                                     )
{
  return cell_1.weight_normed_ > cell_2.weight_normed_;
}

ParticleEstimateHistogram::ParticleEstimateHistogram(const Map& map) :
  x_size_(std::round(map.width * map.scale / HIST_POS_RES)),
  y_size_(std::round(map.height * map.scale / HIST_POS_RES)),
  th_size_(std::round((M_2PI + map.th_origin) / HIST_TH_RES)),
  x_origin_(map.x_origin),
  y_origin_(map.y_origin),
  th_origin_(map.th_origin),
  hist_(x_size_ * y_size_ * th_size_),
  hist_sorted_(x_size_ * y_size_ * th_size_),
  estimates_(NUM_ESTIMATES),
  modified_(false),
  count_(0)
{}

void ParticleEstimateHistogram::add(const Particle& particle)
{
  // Flag as modified since this impacts local averaging used to determine estimates
  modified_ = true;

  // Calculate index
  size_t x_i = std::min(std::max(0.0, (particle.x_ - x_origin_) / HIST_POS_RES),
                        static_cast<double>(x_size_ - 1)
                       );
  size_t y_i = std::min(std::max(0.0, (particle.y_ - y_origin_) / HIST_POS_RES),
                        static_cast<double>(y_size_ - 1)
                       );
  size_t th_i = std::min(std::max(0.0, (particle.th_ - th_origin_) / HIST_TH_RES),
                         static_cast<double>(th_size_ - 1)
                        );
  // Update histogram
  ParticleEstimateHistogramCell& cell_ = cell(x_i, y_i, th_i);

  if (cell_.count_ == 0) {
    ++count_;
  }
  // Update corresponding estimate
  cell_.x_ += particle.x_;
  cell_.y_ += particle.y_;
  cell_.th_ += unwrapAngle(particle.th_);
  cell_.weight_normed_ += particle.weight_normed_;
  ++cell_.count_;

  return;
}

void ParticleEstimateHistogram::updateEstimates()
{
  if (modified_) {
    // Sort the histogram by weight, best first
    hist_sorted_ = hist_;
    std::sort(hist_sorted_.begin(), hist_sorted_.end(), ParticleEstimateHistogramCell::compWeightGreater);

    // Copy the best estimates, locally averaging each histogram cell
    size_t i = 0;
    double normalizer = 0;

    printf("Pose estimate histogram count = %lu\n", count_);
    printf("Pose estimates:\n");

    while (i < count_ && i < estimates_.size()) {
      normalizer = hist_sorted_[i].count_ > 0 ? 1.0 / hist_sorted_[i].count_ : 0.0;
      estimates_[i].x_ = hist_sorted_[i].x_ * normalizer;
      estimates_[i].y_ = hist_sorted_[i].y_ * normalizer;
      estimates_[i].th_ = wrapAngle(hist_sorted_[i].th_ * normalizer);
      estimates_[i].weight_normed_ = hist_sorted_[i].weight_normed_;

      printf("Estimate %lu = %.3f, %.3f, %.3f (weight = %.2e)\n",
             i + 1,
             estimates_[i].x_,
             estimates_[i].y_,
             estimates_[i].th_ * 180.0 / M_PI,
             estimates_[i].weight_normed_
            );
      ++i;
    }
    modified_ = false;
  }
}

const ParticleVector& ParticleEstimateHistogram::estimates()
{
  updateEstimates();
  return estimates_;
}

const Particle& ParticleEstimateHistogram::estimate(size_t i)
{
  updateEstimates();
  return estimates_[i];
}

size_t ParticleEstimateHistogram::count() const
{
  return count_;
}

void ParticleEstimateHistogram::reset()
{
  if (count_ > 0) {
    std::fill(hist_.begin(), hist_.end(), ParticleEstimateHistogramCell());
    count_ = 0;
  }
  std::fill(estimates_.begin(), estimates_.end(), Particle());
}

ParticleEstimateHistogramCell& ParticleEstimateHistogram::cell(const size_t x_i,
                                                               const size_t y_i,
                                                               const size_t th_i
                                                              )
{
  return hist_[x_i * y_size_ * th_size_ + y_i * th_size_ + th_i];
}

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
    particles_.reserve(max_count);
    particles_.resize(max_count);
  }
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

size_t ParticleDistribution::count() const
{
  return count_;
}

double ParticleDistribution::weightAvg() const
{
  return weight_avg_fast_;
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
  if (weight_avg_creep_ > 0.0) {
    // Using slow / creep instead of fast / slow reduces the number of random samples
    ratio = std::min(1.0, weight_avg_slow_ / weight_avg_creep_);
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
    double weight_avg = weight_sum_ / count_;
    weight_avg_creep_.update(weight_avg);
    weight_avg_slow_.update(weight_avg);
    weight_avg_fast_.update(weight_avg);

    // Reinitialize weight variance, normalizer and histogram
    weight_var_ = 0.0;
    double weight_normalizer = weight_sum_ > 0.0 ? 1 / weight_sum_ : 0.0;
    double weight_diff = 0.0;
    hist_.reset();

    for (size_t i = 0; i < count_; ++i) {
      // Normalize weight
      particles_[i].weight_normed_ = particles_[i].weight_ * weight_normalizer;

      // Calculate weight variance sum
      weight_diff = particles_[i].weight_ - weight_avg;
      weight_var_ += weight_diff * weight_diff;

      // Update histogram now that weight has been normalized
      hist_.add(particles_[i]);
    }
    hist_.updateEstimates();
    weight_var_ /= count_;

    // Calculate standard deviation
    weight_std_dev_ = weight_var_ > 0.0 ? std::sqrt(weight_var_) : 0.0;
    weight_relative_std_dev_ = weight_avg > 0.0 ? weight_std_dev_ / weight_avg : 0.0;
  }
  else {
    // No particles, reinitialize
    hist_.reset();
    weight_sum_ = 0.0;
    weight_avg_creep_.reset(0.0);
    weight_avg_slow_.reset(0.0);
    weight_avg_fast_.reset(0.0);
    weight_var_ = 0.0;
    weight_std_dev_ = 0.0;
    weight_relative_std_dev_ = 0.0;
  }
  printf("Weight average = %.2e\n", weightAvg());
  printf("Weight ratio = %.2f\n", weightAvgRatio());
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