#include <algorithm>
#include <cmath>
#include <math.h>

#include "mcl/histogram.h"
#include "mcl/util.h"

static const size_t NUM_ESTIMATES = 5;                    // Number of pose estimates to provide
static const double ESTIMATE_MERGE_DXY_MAX = 1e-1;        // Maximum x or y delta for two estimates to be combined
static const double ESTIMATE_MERGE_DTH_MAX = L_PI / 72.0; // Maximum angular delta for two estimates to be combined
static const double ESTIMATE_WEIGHT_MIN = 1e-1;           // Minimum normalized weight for an estimate to be used
static const double HIST_OCCUPANCY_POS_RES = 0.10;        // Occupancy histogram resolution for x and y position (meters per cell)
static const double HIST_OCCUPANCY_TH_RES = L_PI / 18.0;  // Occupancy histogram resolution for heading angle (rad per cell)
static const double HIST_ESTIMATE_POS_RES = 0.5;          // Estimate histogram resolution for x and y position (meters per cell)
static const double HIST_ESTIMATE_TH_RES = L_PI / 2.0;    // Estimate histogram resolution for heading angle (rad per cell)

using namespace localize;

// ========== ParticleOccupancyHistogram ========== //
ParticleOccupancyHistogram::ParticleOccupancyHistogram(const Map& map) :
  x_size_(std::round(map.width * map.scale / HIST_OCCUPANCY_POS_RES)),
  y_size_(std::round(map.height * map.scale / HIST_OCCUPANCY_POS_RES)),
  th_size_(std::round((L_2PI + map.th_origin) / HIST_OCCUPANCY_TH_RES)),
  x_origin_(map.x_origin),
  y_origin_(map.y_origin),
  th_origin_(map.th_origin),
  hist_(x_size_ * y_size_ * th_size_, false),
  count_(0)
{}

bool ParticleOccupancyHistogram::add(const Particle& particle)
{
  bool count_increased = false;

  // Calculate index
  size_t x_i = std::min(std::max(0.0, particle.x_ - x_origin_) / HIST_OCCUPANCY_POS_RES,
                        static_cast<double>(x_size_ - 1)
                       );
  size_t y_i = std::min(std::max(0.0, particle.y_ - y_origin_) / HIST_OCCUPANCY_POS_RES,
                        static_cast<double>(y_size_ - 1)
                       );
  size_t th_i = std::min(std::max(0.0, unwrapAngle(particle.th_ - th_origin_) / HIST_OCCUPANCY_TH_RES),
                         static_cast<double>(th_size_ - 1)
                        );
  // Update histogram
  if (!cell(x_i, y_i, th_i)) {
    cell(x_i, y_i, th_i) = true;
    count_increased = true;
    ++count_;
  }
  return count_increased;
}

size_t ParticleOccupancyHistogram::count() const
{
  return count_;
}

void ParticleOccupancyHistogram::reset()
{
  if (count_ > 0) {
    std::fill(hist_.begin(), hist_.end(), false);
    count_ = 0;
  }
}

std::vector<bool>::reference ParticleOccupancyHistogram::cell(const size_t x_i,
                                                              const size_t y_i,
                                                              const size_t th_i
                                                             )
{
  return hist_[x_i * y_size_ * th_size_ + y_i * th_size_ + th_i];
}

// ========== ParticleEstimateHistogramCell ========== //
ParticleEstimateHistogramCell::ParticleEstimateHistogramCell() :
  x_sum_(0.0),
  y_sum_(0.0),
  th_top_sum_(0.0),
  th_bot_sum_(0.0),
  th_top_count_(0),
  weight_normed_sum_(0.0),
  count_(0)
{}

int ParticleEstimateHistogramCell::compare(const ParticleEstimateHistogramCell& lhs,
                                           const ParticleEstimateHistogramCell& rhs
                                          ) const
{
  int result;
  if (lhs.weight_normed_sum_ < rhs.weight_normed_sum_) {
    result = -1;
  }
  else if (lhs.weight_normed_sum_ > rhs.weight_normed_sum_) {
    result = 1;
  }
  else { // lhs.weight_normed_sum_ == rhs.weight_normed_sum_
    result = 0;
  }
  return result;
}

ParticleEstimateHistogramCell& ParticleEstimateHistogramCell::operator+=(const ParticleEstimateHistogramCell& rhs)
{
  x_sum_ += rhs.x_sum_;
  y_sum_ += rhs.y_sum_;
  th_top_sum_ += rhs.th_top_sum_;
  th_bot_sum_ += rhs.th_bot_sum_;
  th_top_count_ += rhs.th_top_count_;
  weight_normed_sum_ += rhs.weight_normed_sum_;
  count_ += rhs.count_;

  return *this;
}

const ParticleEstimateHistogramCell ParticleEstimateHistogramCell::operator+(const ParticleEstimateHistogramCell& rhs) const
{
  ParticleEstimateHistogramCell lhs = *this;
  lhs += rhs;

  return lhs;
}

void ParticleEstimateHistogramCell::add(const Particle& particle)
{
  x_sum_ += particle.x_;
  y_sum_ += particle.y_;

  if (std::signbit(particle.th_)) {
    th_top_sum_ += particle.th_;
    ++th_top_count_;
  }
  else {
    th_bot_sum_ += particle.th_;
  }
  weight_normed_sum_ += particle.weight_normed_;
  ++count_;
}

Particle localize::particle(const ParticleEstimateHistogramCell& cell)
{
  // Calculate average x and y
  double normalizer = cell.count_ > 0 ? 1.0 / cell.count_ : 0.0;
  double x_avg = cell.x_sum_ * normalizer;
  double y_avg = cell.y_sum_ * normalizer;

  // Calculate average angle
  // First average the top half plane (positive) angles and bottom half plane (negative) angles
  size_t th_top_count = cell.th_top_count_;
  size_t th_bot_count = cell.count_ - th_top_count;
  double th_top_avg = th_top_count > 0 ? cell.th_top_sum_ / th_top_count : 0.0;
  double th_bot_avg = th_bot_count > 0 ? cell.th_bot_sum_ / th_bot_count : 0.0;

  // Get the delta from top -> bottom, in whichever direction is closest
  double th_avg_delta = angleDelta(th_top_avg, th_bot_avg);

  // Offset from the top angle by a fraction of the delta between the two angles, proportional to the weight of the bottom
  double th_avg = wrapAngle(th_top_avg + th_avg_delta * th_bot_count / (th_bot_count + th_top_count));

  return Particle(x_avg, y_avg, th_avg, cell.weight_normed_sum_);
}

// ========== ParticleEstimateHistogram ========== //
ParticleEstimateHistogram::ParticleEstimateHistogram(const Map& map) :
  x_size_(std::round(map.width * map.scale / HIST_ESTIMATE_POS_RES)),
  y_size_(std::round(map.height * map.scale / HIST_ESTIMATE_POS_RES)),
  th_size_(std::round((L_2PI + map.th_origin) / HIST_ESTIMATE_TH_RES)),
  x_origin_(map.x_origin),
  y_origin_(map.y_origin),
  th_origin_(map.th_origin),
  hist_(x_size_ * y_size_ * th_size_),
  hist_sorted_(x_size_ * y_size_ * th_size_),
  estimates_(NUM_ESTIMATES),
  update_estimates_(false),
  count_(0)
{}

void ParticleEstimateHistogram::add(const Particle& particle)
{
  // Flag as modified since this impacts local averaging used to determine estimates
  update_estimates_ = true;

  // Calculate index
  size_t x_i = std::min(std::max(0.0, particle.x_ - x_origin_) / HIST_ESTIMATE_POS_RES,
                        static_cast<double>(x_size_ - 1)
                       );
  size_t y_i = std::min(std::max(0.0, particle.y_ - y_origin_) / HIST_ESTIMATE_POS_RES,
                        static_cast<double>(y_size_ - 1)
                       );
  size_t th_i = std::min(std::max(0.0, unwrapAngle(particle.th_ - th_origin_) / HIST_ESTIMATE_TH_RES),
                         static_cast<double>(th_size_ - 1)
                        );
  // Add particle to cell
  cell(x_i, y_i, th_i).add(particle);

  // If this is the cell's first particle, increment the histogram's count
  if (cell(x_i, y_i, th_i).count_ == 1) {
    ++count_;
  }
  return;
}

void ParticleEstimateHistogram::updateEstimates()
{
  if (update_estimates_) {
    // Sort the histogram by weight, largest (best) first
    hist_sorted_ = hist_;
    std::sort(hist_sorted_.begin(), hist_sorted_.end(), Greater());

    // Copy the best estimates, locally averaging each histogram cell
    printf("Estimate histogram count = %lu\n", count_);
    size_t min_size = std::min(std::min(count_, estimates_.size()), hist_sorted_.size());

    for (size_t i = 0; i < min_size; ++i) {
      // If estimate confidence is too low to be useful, don't provide it
      if (   hist_sorted_[i].weight_normed_sum_ < ESTIMATE_WEIGHT_MIN
          && i > 0  // Always generate at least one estimate
         ) {
        break;
      }
      // Convert cell to a particle by averaging all particles that were added to the cell
      estimates_[i] = particle(hist_sorted_[i]);

      printf("Estimate %lu = %.3f, %.3f, %.3f (weight = %.2e)\n",
             i + 1,
             estimates_[i].x_,
             estimates_[i].y_,
             estimates_[i].th_ * 180.0 / L_PI,
             estimates_[i].weight_normed_
            );
    }
    // Go back through the estimates and average again if they are close to each other
    // This smoothes discretization effects that occur when a cluster of particles moves cross cell boundaries
    ParticleEstimateHistogramCell cell_empty;

    for (size_t i = 0; i < min_size; ++i) {
      bool update_estimate = false;

      for (size_t j = 0; j < min_size; ++j) {
        // Compute deltas and determine if estimates are sufficiently close to each other to be merged
        if (   i != j
            && hist_sorted_[j].count_ > 0
            && std::abs(estimates_[i].x_ - estimates_[j].x_) < ESTIMATE_MERGE_DXY_MAX
            && std::abs(estimates_[i].y_ - estimates_[j].y_) < ESTIMATE_MERGE_DXY_MAX
            && std::abs(angleDelta(estimates_[i].th_, estimates_[j].th_)) < ESTIMATE_MERGE_DTH_MAX
           ) {
          // Sum histogram cells so the correct weighted average can be determined for the particle estimate later
          hist_sorted_[i] += hist_sorted_[j];
          hist_sorted_[j] = cell_empty;
          update_estimate = true;
        }
      }
      // Update particle estimate by averaging the updated sums for this histogram cell
      if (update_estimate) {
        estimates_[i] = particle(hist_sorted_[i]);
        printf("Updated estimate %lu = %.3f, %.3f, %.3f (weight = %.2e)\n",
               i + 1,
               estimates_[i].x_,
               estimates_[i].y_,
               estimates_[i].th_ * 180.0 / L_PI,
               estimates_[i].weight_normed_
              );
      }
    }
    update_estimates_ = false;
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