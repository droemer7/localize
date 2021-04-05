#include "mcl/histogram.h"

static const size_t NUM_ESTIMATES = 5;                    // Number of pose estimates to provide
static const double ESTIMATE_MERGE_DXY_MAX = 0.10;        // Maximum x or y delta for two estimates to be combined
static const double ESTIMATE_MERGE_DTH_MAX = L_PI / 72.0; // Maximum angular delta for two estimates to be combined
static const double HIST_OCCUPANCY_POS_RES = 0.10;        // Occupancy histogram resolution for x and y position (meters per cell)
static const double HIST_OCCUPANCY_TH_RES = L_PI / 18.0;  // Occupancy histogram resolution for heading angle (rad per cell)
static const double HIST_ESTIMATE_POS_RES = 0.25;         // Estimate histogram resolution for x and y position (meters per cell)
static const double HIST_ESTIMATE_TH_RES = L_PI / 4.0;    // Estimate histogram resolution for heading angle (rad per cell)

using namespace localize;

// ========== ParticleOccupancyHistogram ========== //
ParticleOccupancyHistogram::ParticleOccupancyHistogram(const Map& map) :
  x_size_(std::round(map.width * map.scale / HIST_OCCUPANCY_POS_RES)),
  y_size_(std::round(map.height * map.scale / HIST_OCCUPANCY_POS_RES)),
  th_size_(std::round(L_2PI / HIST_OCCUPANCY_TH_RES)),
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
  long x_i = (particle.x_ - x_origin_) / HIST_OCCUPANCY_POS_RES;
  long y_i = (particle.y_ - y_origin_) / HIST_OCCUPANCY_POS_RES;
  long th_i = particle.th_ >= th_origin_ ? (particle.th_ - th_origin_) / HIST_OCCUPANCY_TH_RES
                                         : (particle.th_ - th_origin_ + L_2PI) / HIST_OCCUPANCY_TH_RES;

  // Ignore particles out of bounds
  if(   0 <= x_i  && x_i  < x_size_
     && 0 <= y_i  && y_i  < y_size_
     && 0 <= th_i && th_i < th_size_
     && !cell(x_i, y_i, th_i)
    ) {
    // Add particle to histogram in the corresponding cell
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
// ========== End ParticleOccupancyHistogram ========== //

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
    th_bot_sum_ += particle.th_;
  }
  else {
    th_top_sum_ += particle.th_;
    ++th_top_count_;
  }
  weight_normed_sum_ += particle.weight_normed_;
  ++count_;
}

size_t ParticleEstimateHistogramCell::count() const
{
  return count_;
}

namespace localize
{
  Particle particle(const ParticleEstimateHistogramCell& cell)
  {
    Particle particle;

    if (cell.count_ > 0) {
      // Calculate average x and y
      double x_avg = cell.x_sum_ / cell.count_;
      double y_avg = cell.y_sum_ / cell.count_;

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

      // Update particle state
      particle.x_ = x_avg;
      particle.y_ = y_avg;
      particle.th_ = th_avg;
      particle.weight_normed_ = cell.weight_normed_sum_;
    }
    return particle;
  }
} // namespace localize

bool localize::cellEstimateGreater(const ParticleEstimateHistogramCell* lhs,
                                   const ParticleEstimateHistogramCell* rhs
                                  )
{
  return *lhs > *rhs;
}

// ========== ParticleEstimateHistogram ========== //
ParticleEstimateHistogram::ParticleEstimateHistogram(const Map& map) :
  x_size_(std::round(map.width * map.scale / HIST_ESTIMATE_POS_RES)),
  y_size_(std::round(map.height * map.scale / HIST_ESTIMATE_POS_RES)),
  th_size_(std::round(L_2PI / HIST_ESTIMATE_TH_RES)),
  x_origin_(map.x_origin),
  y_origin_(map.y_origin),
  th_origin_(map.th_origin),
  hist_(x_size_ * y_size_ * th_size_),
  estimates_(NUM_ESTIMATES),
  update_estimates_(false),
  count_(0)
{}

void ParticleEstimateHistogram::add(const Particle& particle)
{
  // Calculate index
  long x_i = (particle.x_ - x_origin_) / HIST_ESTIMATE_POS_RES;
  long y_i = (particle.y_ - y_origin_) / HIST_ESTIMATE_POS_RES;
  long th_i = particle.th_ >= th_origin_ ? (particle.th_ - th_origin_) / HIST_ESTIMATE_TH_RES
                                         : (particle.th_ - th_origin_ + L_2PI) / HIST_ESTIMATE_TH_RES;

  // Ignore particles out of bounds
  if(   0 <= x_i  && x_i  < x_size_
     && 0 <= y_i  && y_i  < y_size_
     && 0 <= th_i && th_i < th_size_
    ) {
    // Add particle to histogram in the corresponding cell
    ParticleEstimateHistogramCell& cell_curr = cell(x_i, y_i, th_i);
    cell_curr.add(particle);

    // If this is the cell's first particle, increment the histogram's count
    if (cell_curr.count() == 1) {
      cells_.push_back(&cell_curr);
      ++count_;
    }
  }
  // Flag as modified since this impacts local averaging used to determine estimates
  update_estimates_ = count_ > 0;

  return;
}

// TBD remove print statements
void ParticleEstimateHistogram::calcEstimates()
{
  printf("\n===== Calculating estimates =====\n");
  printf("Estimate histogram count = %lu\n", cells_.size());

  // Sort all populated histogram cells by weight, largest (best) first
  std::sort(cells_.begin(), cells_.end(), cellEstimateGreater);

  // Convert histogram cells to particles by averaging all particles in each cell
  estimates_.resize(std::min(cells_.size(), NUM_ESTIMATES));

  for (size_t i = 0; i < estimates_.size(); ++i) {
    estimates_[i] = particle(*cells_[i]);
  }
  // Make another pass and average estimates if they are close to each other
  // This smoothes discretization effects that occur when a cluster of particles moves cross cell boundaries
  ParticleEstimateHistogramCell cell_default;
  Particle particle_default;

  for (size_t i = 0; i < estimates_.size(); ++i) {
    bool update_estimate = false;

    if (cells_[i]->count() > 0) {
      for (size_t j = i + 1; j < estimates_.size(); ++j) {
        // Compute deltas and determine if estimates are sufficiently close to each other to be merged
        if (   cells_[j]->count() > 0
            && std::abs(estimates_[i].x_ - estimates_[j].x_) < ESTIMATE_MERGE_DXY_MAX
            && std::abs(estimates_[i].y_ - estimates_[j].y_) < ESTIMATE_MERGE_DXY_MAX
            && std::abs(angleDelta(estimates_[i].th_, estimates_[j].th_)) < ESTIMATE_MERGE_DTH_MAX
           ) {
          // Sum histogram cells so the correct weighted average can be determined for the particle estimate later
          *cells_[i] += *cells_[j];
          update_estimate = true;

          // Reset histogram cell and estimate so they are not reused
          *cells_[j] = cell_default;
          estimates_[j] = particle_default;
        }
      }
      // Regenerate particle estimate by averaging the updated histogram cell
      if (update_estimate) {
        estimates_[i] = particle(*cells_[i]);
      }
    }
  }
  // Ensure proper ordering with best estimates first in case it changed during reaveraging
  std::sort(estimates_.begin(), estimates_.end(), Greater());

  // Ignore zero weight estimates, but always generate at least one estimate
  for (size_t i = 0; i < estimates_.size(); ++i) {
    if (   estimates_[i].weight_normed_ <= 0.0
        && i > 0
       ) {
      estimates_.resize(i);
      break;
    }
    printEstimate(i);
  }
  update_estimates_ = false;
}

const ParticleVector& ParticleEstimateHistogram::estimates()
{
  if (update_estimates_) {
    calcEstimates();
  }
  return estimates_;
}

void ParticleEstimateHistogram::reset()
{
  if (count_ > 0) {
    std::fill(hist_.begin(), hist_.end(), ParticleEstimateHistogramCell());
    cells_.resize(0);
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

size_t ParticleEstimateHistogram::count() const
{
  return count_;
}

void ParticleEstimateHistogram::printEstimate(const size_t e) const
{
  printf("Estimate %lu = %.3f, %.3f, %.3f (weight = %.2e)\n",
         e + 1,
         estimates_[e].x_,
         estimates_[e].y_,
         estimates_[e].th_ * 180.0 / L_PI,
         estimates_[e].weight_normed_
        );
}
// ========== ParticleEstimateHistogram ========== //