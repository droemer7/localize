#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include "mcl/common.h"

namespace localize
{
  // Histogram representing occupancy of pose space (x, y, th)
  class ParticleOccupancyHistogram
  {
  public:
    // Constructors
    ParticleOccupancyHistogram(const Map& map);  // Map

    // Update histogram occupancy with the particle's pose
    // Returns true if the particle fell into a new (unoccupied) cell, increasing the occupancy count
    bool add(const Particle& particle);

    // Histogram occupancy count
    size_t count() const;

    // Reset cell states and occupancy count
    void reset();

  private:
    // Reference a cell by index
    std::vector<bool>::reference cell(const size_t x_i,
                                      const size_t y_i,
                                      const size_t th_i
                                     );
  private:
    const size_t x_size_;     // Size of x dimension (number of elements)
    const size_t y_size_;     // Size of y dimension (number of elements)
    const size_t th_size_;    // Size of angular dimension (number of elements)
    const double x_origin_;   // X translation of origin (cell 0,0) relative to world frame (meters)
    const double y_origin_;   // Y translation of origin (cell 0,0) relative to world frame (meters)
    const double th_origin_;  // Angle relative to world frame (rad)

    std::vector<bool> hist_;  // Histogram
    size_t count_;            // Histogram occupancy count
  };

  struct ParticleEstimateHistogramCell
  {
    // Constructors
    ParticleEstimateHistogramCell();

    // Compare by normalized weight sums
    int compare(const ParticleEstimateHistogramCell& lhs,
                const ParticleEstimateHistogramCell& rhs
               ) const;

    // Operators
    void operator+=(const Particle& particle);

    bool operator==(const ParticleEstimateHistogramCell& rhs) const
    { return compare(*this, rhs) == 0; }

    bool operator!=(const ParticleEstimateHistogramCell& rhs) const
    { return compare(*this, rhs) != 0; }

    bool operator<(const ParticleEstimateHistogramCell& rhs) const
    { return compare(*this, rhs) < 0; }

    bool operator>(const ParticleEstimateHistogramCell& rhs) const
    { return compare(*this, rhs) > 0; }

    bool operator<=(const ParticleEstimateHistogramCell& rhs) const
    { return compare(*this, rhs) <= 0; }

    bool operator>=(const ParticleEstimateHistogramCell& rhs) const
    { return compare(*this, rhs) >= 0; }

    double x_sum_;              // Sum of particle x positions
    double y_sum_;              // Sum of particle y positions
    double th_top_sum_;         // Sum of particle angles in the top half plane of (-pi, pi]
    double th_bot_sum_;         // Sum of particle angles in the bottom half plane of (-pi, pi]
    double weight_normed_sum_;  // Sum of particle normalized weights for this cell
    size_t th_top_count_;       // Count of angles in the top half plane (-pi, 0.0) of the (-pi, pi] space
    size_t count_;              // Count of particles in this cell
  };

  // Convert cell to a particle by computing the average of all particles added to the cell
  Particle particle(const ParticleEstimateHistogramCell& cell);

  // Histogram to estimate a multimodal distribution of poses (x, y, th) by weight
  class ParticleEstimateHistogram
  {
  public:
    // Constructors
    ParticleEstimateHistogram(const Map& map);  // Map

    // Update histogram with the particle
    void add(const Particle& particle);

    // Update the estimates by sorting the histogram and selecting the best local averages
    // If no changes have been made to the histogram since this was last called, no calculations are performed
    void updateEstimates();

    // Return the top particle estimates - lower indexes are better estimates
    ParticleVector estimates();

    // Return the particle estimate from the list by index - lower indexes are better estimates
    Particle estimate(const size_t e = 0);

    // Number of estimates
    size_t count() const;

    // Reset histogram and estimates
    void reset();

  private:
    // Reference a cell by index
    ParticleEstimateHistogramCell& cell(const size_t x_i,
                                        const size_t y_i,
                                        const size_t th_i
                                       );
  private:
    const size_t x_size_;     // Size of x dimension (number of elements)
    const size_t y_size_;     // Size of y dimension (number of elements)
    const size_t th_size_;    // Size of angular dimension (number of elements)
    const double x_origin_;   // X translation of origin (cell 0,0) relative to world frame (meters)
    const double y_origin_;   // Y translation of origin (cell 0,0) relative to world frame (meters)
    const double th_origin_;  // Angle relative to world frame (rad)

    std::vector<ParticleEstimateHistogramCell> hist_;         // Histogram
    std::vector<ParticleEstimateHistogramCell> hist_sorted_;  // Histogram sorted for estimate generation

    ParticleVector estimates_;  // Averaged estimates

    bool modified_;  // Histogram was modified so estimates need to be regenerated on next request
    size_t count_;   // Histogram occupancy count
  };

} // namespace localize

#endif // HISTOGRAM_H