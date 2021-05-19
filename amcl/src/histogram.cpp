#include "amcl/histogram.h"

static const size_t NUM_ESTIMATES = 5;                    // Number of pose estimates to provide
static const double ESTIMATE_MERGE_DXY_MAX = 0.10;        // Maximum x or y delta for two estimates to be combined
static const double ESTIMATE_MERGE_DTH_MAX = L_PI / 12.0; // Maximum angular delta for two estimates to be combined
static const double HIST_OCCUPANCY_XY_RES = 0.05;         // Occupancy histogram resolution for x and y position (meters per cell)
static const double HIST_OCCUPANCY_TH_RES = L_PI / 120.0;  // Occupancy histogram resolution for heading angle (rad per cell)
static const double HIST_ESTIMATE_XY_RES = 0.10;          // Estimate histogram resolution for x and y position (meters per cell)
static const double HIST_ESTIMATE_TH_RES = L_PI / 12.0;   // Estimate histogram resolution for heading angle (rad per cell)

using namespace localize;

// ========== PoseOccupancyHistogram ========== //
PoseOccupancyHistogram::PoseOccupancyHistogram(const Map& map) :
  map_(map),
  xy_scale_(map.scaleWorld() / HIST_OCCUPANCY_XY_RES),
  th_scale_(1.0 / HIST_OCCUPANCY_TH_RES),
  x_size_(std::round(map.xSize() * xy_scale_)),
  y_size_(std::round(map.ySize() * xy_scale_)),
  th_size_(std::round(L_2PI * th_scale_)),
  hist_(x_size_ * y_size_ * th_size_, false),
  count_(0)
{}

bool PoseOccupancyHistogram::add(const Pose& pose)
{
  bool count_increased = false;

  // Calculate index
  // Convert pose from world to map frame
  Pose pose_map = worldToMap(map_, pose);

  // Rescale into histogram resolution
  // Since x and y scale must be equal for an occupancy grid, order of rotating and scaling doesn't matter
  long x_i = pose_map.x_ * xy_scale_;
  long y_i = pose_map.y_ * xy_scale_;
  long th_i = (pose_map.th_ + L_PI) * th_scale_;

  // Ignore poses out of bounds
  if(   0 <= x_i  && x_i  < x_size_
     && 0 <= y_i  && y_i  < y_size_
     && 0 <= th_i && th_i < th_size_
     && !cell(x_i, y_i, th_i)
    ) {
    // Add pose to histogram in the corresponding cell
    cell(x_i, y_i, th_i) = true;
    count_increased = true;
    ++count_;
  }
  return count_increased;
}

void PoseOccupancyHistogram::reset()
{
  if (count_ > 0) {
    std::fill(hist_.begin(), hist_.end(), false);
    count_ = 0;
  }
}

size_t PoseOccupancyHistogram::count() const
{ return count_; }

std::vector<bool>::reference PoseOccupancyHistogram::cell(const size_t x_i,
                                                          const size_t y_i,
                                                          const size_t th_i
                                                         )
{ return hist_[x_i * y_size_ * th_size_ + y_i * th_size_ + th_i]; }

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
{ return count_; }

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
{ return *lhs > *rhs; }

// ========== ParticleEstimateHistogram ========== //
ParticleEstimateHistogram::ParticleEstimateHistogram(const Map& map) :
  map_(map),
  xy_scale_(map.scaleWorld() / HIST_ESTIMATE_XY_RES),
  th_scale_(1.0 / HIST_ESTIMATE_TH_RES),
  x_size_(std::round(map.xSize() * xy_scale_)),
  y_size_(std::round(map.ySize() * xy_scale_)),
  th_size_(std::round(L_2PI * th_scale_)),
  hist_(x_size_ * y_size_ * th_size_),
  estimates_(NUM_ESTIMATES),
  update_estimates_(false),
  count_(0)
{}

void ParticleEstimateHistogram::add(const Particle& particle)
{
  // Calculate index
  // Convert pose from world to map frame
  Pose pose_map = worldToMap(map_, particle);

  // Rescale into histogram resolution
  // Since x and y scale must be equal for an occupancy grid, order of rotating and scaling doesn't matter
  long x_i = pose_map.x_ * xy_scale_;
  long y_i = pose_map.y_ * xy_scale_;
  long th_i = (pose_map.th_ + L_PI) * th_scale_;

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
    // Flag as modified since this impacts local averaging used to determine estimates
    update_estimates_ = count_ > 0;
  }
  return;
}

void ParticleEstimateHistogram::calcEstimates()
{
  // Sort all populated histogram cells by weight, largest (best) first
  std::sort(cells_.begin(), cells_.end(), cellEstimateGreater);

  // Convert histogram cells to particle estimates by averaging all particles in each cell
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

  // Ignore zero weight estimates
  for (size_t i = 0; i < estimates_.size(); ++i) {
    if (estimates_[i].weight_normed_ <= 0.0) {
      estimates_.resize(i);
      break;
    }
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
{ return hist_[x_i * y_size_ * th_size_ + y_i * th_size_ + th_i]; }

size_t ParticleEstimateHistogram::count() const
{ return count_; }
// ========== ParticleEstimateHistogram ========== //
