#include "mcl/dist.h"

static const size_t NUM_ESTIMATES = 5;            // Number of pose estimates to provide
static const double HIST_POS_RES = 0.50;          // Histogram resolution for x and y position (meters per cell)
static const double HIST_TH_RES = M_2PI / 3.0;    // Histogram resolution for heading angle (rad per cell)
static const double WEIGHT_AVG_CREEP_RATE = 0.01; // Weight average smoothing rate, very slow
static const double WEIGHT_AVG_SLOW_RATE = 0.25;  // Weight average smoothing rate, slow
static const double WEIGHT_AVG_FAST_RATE = 0.50;  // Weight average smoothing rate, fast

using namespace localize;

ParticleEstimateHistogramCell::ParticleEstimateHistogramCell() :
  x_sum_(0.0),
  y_sum_(0.0),
  th_top_sum_(0.0),
  th_bot_sum_(0.0),
  th_top_count_(0),
  th_bot_count_(0),
  weight_normed_sum_(0.0),
  count_(0)
{}

bool ParticleEstimateHistogramCell::compWeightGreater(const ParticleEstimateHistogramCell& cell_1,
                                                      const ParticleEstimateHistogramCell& cell_2
                                                     )
{
  return cell_1.weight_normed_sum_ > cell_2.weight_normed_sum_;
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
  size_t th_i = std::min(std::max(0.0, (unwrapAngle(particle.th_ - th_origin_)) / HIST_TH_RES),
                         static_cast<double>(th_size_ - 1)
                        );
  // Update histogram
  ParticleEstimateHistogramCell& cell_ = cell(x_i, y_i, th_i);

  if (cell_.count_ == 0) {
    ++count_;
  }
  cell_.x_sum_ += particle.x_;
  cell_.y_sum_ += particle.y_;

  if (particle.th_ >= 0.0) {
    cell_.th_top_sum_ += particle.th_;
    ++cell_.th_top_count_;
  }
  else {
    cell_.th_bot_sum_ += particle.th_;
    ++cell_.th_bot_count_;
  }
  cell_.weight_normed_sum_ += particle.weight_normed_;
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
    double normalizer = 0.0;
    double th = 0.0;
    double th_delta = 0.0;
    double th_top_avg = 0.0;
    double th_bot_avg = 0.0;
    size_t th_top_count = 0.0;
    size_t th_bot_count = 0.0;

    printf("Estimate histogram count = %lu\n", count_);

    while (i < count_ && i < estimates_.size() && i < hist_sorted_.size()) {
      normalizer = hist_sorted_[i].count_ > 0 ? 1.0 / hist_sorted_[i].count_ : 0.0;
      estimates_[i].x_ = hist_sorted_[i].x_sum_ * normalizer;
      estimates_[i].y_ = hist_sorted_[i].y_sum_ * normalizer;

      // To compute the average angle, we first need to average the top half plane (positive) angles and bottom half
      // plane (negative) angles
      th_top_count = hist_sorted_[i].th_top_count_;
      th_bot_count = hist_sorted_[i].th_bot_count_;
      th_top_avg = th_top_count > 0 ? hist_sorted_[i].th_top_sum_ / th_top_count : 0.0;
      th_bot_avg = th_bot_count > 0 ? hist_sorted_[i].th_bot_sum_ / th_bot_count : 0.0;

      // Calculate the delta between the two angles - choose whichever is smaller
      // The delta is applied to the top angle to bring it closer to the bottom angle, so if top - bottom < pi,
      // the delta is negative
      if (th_top_avg - th_bot_avg < M_PI) {
        th_delta = th_bot_avg - th_top_avg;
      }
      else {
        th_delta = (M_PI - th_top_avg) + (M_PI + th_bot_avg);
      }
      // Offset from the top angle by a fraction of the delta between the two angles, proportional to the weight of the bottom
      th = th_top_avg + th_delta * th_bot_count / (th_bot_count + th_top_count);
      estimates_[i].th_ = wrapAngle(th);
      estimates_[i].weight_normed_ = hist_sorted_[i].weight_normed_sum_;

      printf("Estimate %lu = %.3f, %.3f, %.3f (weight = %.2e) (top count = %lu, bot count = %lu)\n",
             i + 1,
             estimates_[i].x_,
             estimates_[i].y_,
             estimates_[i].th_ * 180.0 / M_PI,
             estimates_[i].weight_normed_,
             th_top_count,
             th_bot_count
            );
      ++i;
    }
    modified_ = false;
  }
}

ParticleVector ParticleEstimateHistogram::estimates()
{
  updateEstimates();
  return estimates_;
}

Particle ParticleEstimateHistogram::estimate(size_t e)
{
  updateEstimates();
  return estimates_[e];
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

ParticleVector ParticleDistribution::estimates()
{
  return hist_.estimates();
}

Particle ParticleDistribution::estimate(const size_t e)
{
  return hist_.estimate(e);
}

Particle& ParticleDistribution::particle(const size_t p)
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