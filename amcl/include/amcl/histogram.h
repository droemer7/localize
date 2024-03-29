// Copyright (c) 2021-2023 Dane Roemer droemer7@gmail.com
// Distributed under the terms of the MIT License

#ifndef HISTOGRAM_H
#define HISTOGRAM_H

#include "amcl/common.h"

namespace localize
{
  // Histogram representing occupancy of pose space (x, y, th)
  class PoseOccupancyHistogram
  {
  public:
    // Constructors
    PoseOccupancyHistogram(const Map& map);  // Map

    // Update histogram occupancy with the particle's pose
    // Returns true if the particle fell into a new (unoccupied) cell, increasing the occupancy count
    bool add(const Pose& pose);

    // Reset cell states and occupancy count
    void reset();

    // Histogram occupancy count
    int count() const;

  private:
    // Reference a cell by index
    std::vector<bool>::reference cell(const int x_i,
                                      const int y_i,
                                      const int th_i
                                     );
  private:
    const Map& map_;          // Map the histogram represents

    const double xy_scale_;   // Scale of histogram x and y dimensions relative to map frame (cells per map pixel)
    const double th_scale_;   // Scale of histogram angular dimension (cells per rad)
    const int x_size_;        // Size of x dimension (number of elements)
    const int y_size_;        // Size of y dimension (number of elements)
    const int th_size_;       // Size of angular dimension (number of elements)

    std::vector<bool> hist_;  // Histogram
    int count_;               // Histogram occupancy count
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

    // Cell particle count
    int count() const;

    // Convert cell to a particle by computing the average of all particles added to the cell
    friend Particle particle(const ParticleEstimateHistogramCell& cell);

  private:
    double x_sum_;              // Sum of particle x positions
    double y_sum_;              // Sum of particle y positions
    double th_top_sum_;         // Sum of particle angles in the top half plane of (-pi, pi]
    double th_bot_sum_;         // Sum of particle angles in the bottom half plane of (-pi, pi]
    int th_top_count_;          // Count of angles in the top half plane (-pi, 0.0) of the (-pi, pi] space
    double weight_normed_sum_;  // Sum of particle normalized weights for this cell
    int count_;                 // Count of particles in this cell
  };

  // Compare estimate histogram cells by weight, descending
  bool cellEstimateGreater(const ParticleEstimateHistogramCell* lhs,
                           const ParticleEstimateHistogramCell* rhs
                          );

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
    int count() const;

  private:
    // Reference a cell by index
    ParticleEstimateHistogramCell& cell(const int x_i,
                                        const int y_i,
                                        const int th_i
                                       );
  private:
    const Map& map_;          // Map the histogram represents

    const double xy_scale_;   // Scale of histogram x and y dimensions relative to map frame (cells per map pixel)
    const double th_scale_;   // Scale of histogram angular dimension (cells per rad)
    const int x_size_;        // Size of x dimension (number of elements)
    const int y_size_;        // Size of y dimension (number of elements)
    const int th_size_;       // Size of angular dimension (number of elements)

    std::vector<ParticleEstimateHistogramCell> hist_;       // Histogram
    std::vector<ParticleEstimateHistogramCell*> cell_ptrs_; // Pointers to populated cells

    ParticleVector estimates_;  // Best estimates
    bool update_estimates_;     // Estimates need to be regenerated on next request because the histogram was modified
    int count_;                 // Histogram occupancy count
  };

} // namespace localize

#endif // HISTOGRAM_H