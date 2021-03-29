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

  class ParticleEstimateHistogramCell
  {
  public:
    // Constructors
    ParticleEstimateHistogramCell();

    // Compare by normalized weight sums
    int compare(const ParticleEstimateHistogramCell& lhs,
                const ParticleEstimateHistogramCell& rhs
               ) const;

    // Operators
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

    // Element wise addition
    ParticleEstimateHistogramCell& operator+=(const ParticleEstimateHistogramCell& rhs);

    // Element wise addition
    const ParticleEstimateHistogramCell operator+(const ParticleEstimateHistogramCell& rhs) const;

    // Add particle to cell
    void add(const Particle& particle);

    // Convert cell to a particle by computing the average of all particles added to the cell
    friend Particle particle(const ParticleEstimateHistogramCell& cell);

    // Cell particle count
    size_t count() const;

  private:
    double x_sum_;              // Sum of particle x positions
    double y_sum_;              // Sum of particle y positions
    double th_top_sum_;         // Sum of particle angles in the top half plane of (-pi, pi]
    double th_bot_sum_;         // Sum of particle angles in the bottom half plane of (-pi, pi]
    size_t th_top_count_;       // Count of angles in the top half plane (-pi, 0.0) of the (-pi, pi] space
    double weight_normed_sum_;  // Sum of particle normalized weights for this cell
    size_t count_;              // Count of particles in this cell
  };

  Particle particle(const ParticleEstimateHistogramCell& cell);

  // Histogram to estimate a multimodal distribution of poses (x, y, th) by weight
  class ParticleEstimateHistogram
  {
  public:
    // Constructors
    ParticleEstimateHistogram(const Map& map);  // Map

    // Update histogram with the particle
    void add(const Particle& particle);

    // Calculates the estimates by sorting the histogram and selecting the best local averages
    void calcEstimates();

    // Return the top particle estimates - lower indexes are better estimates
    const ParticleVector& estimates();

    // Reset histogram and estimates
    void reset();

    // Histogram occupancy count (not the number of estimates)
    size_t count() const;

  private:
    void printEstimate(const size_t e) const;

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

    ParticleVector estimates_;  // Best estimates
    bool update_estimates_;     // Estimates need to be regenerated on next request because the histogram was modified
    size_t count_;              // Histogram occupancy count
  };

} // namespace localize

#endif // HISTOGRAM_H