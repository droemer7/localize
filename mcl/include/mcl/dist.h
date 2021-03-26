#ifndef DIST_H
#define DIST_H

#include "mcl/common.h"

namespace localize
{
  struct ParticleEstimateHistogramCell
  {
    // Constructors
    ParticleEstimateHistogramCell();

    // Compare cells by weight, cell 1 weight > cell 2 weight
    static bool compWeightGreater(const ParticleEstimateHistogramCell& cell_1,
                                  const ParticleEstimateHistogramCell& cell_2
                                 );

    double x_sum_;
    double y_sum_;
    double th_top_sum_;   // Sum of angles in the top half plane (-pi, 0.0) of the (-pi, pi] space
    double th_bot_sum_;   // Sum of angles in the bottom half plane (0.0, pi] of the (-pi, pi] space
    size_t th_top_count_; // Count of angles in the top half plane (-pi, 0.0) of the (-pi, pi] space
    size_t th_bot_count_; // Count of angles in the bottom half plane (0.0, pi] of the (-pi, pi] space
    double weight_normed_sum_;
    size_t count_;
  };

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

  class ParticleDistribution
  {
  public:
    // Constructors
    ParticleDistribution(const size_t max_count, // Number of particles for which to reserve space
                         const Map& map          // Map
                        );

    // Copies new particles to the distribution and updates size, does not do anything else
    void copy(const ParticleVector& particles,
              const size_t count = 0
             );

    // Update statistics and reset sampler state
    void update();

    // Update particles and statistics and reset sampler state
    void update(const ParticleVector& particles,
                const size_t count = 0
               );

    // Return the top particle estimates - lower indexes are better estimates
    ParticleVector estimates();

    // Return the particle estimate from the list by index - lower indexes are better estimates
    Particle estimate(const size_t e = 0);

    // Reference to a particle in the distribution
    Particle& particle(const size_t p);

    // Samples a particle (with replacement) from the distribution with probability proportional to its weight
    Particle& sample();

    // Number of particles in use - in general less than particle vector size
    size_t count() const;

    // Average particle weight (smoothed over several updates)
    double weightAvg() const;

    // Particle weight variance
    double weightVar() const;

    // Particle weight standard deviation
    double weightStdDev() const;

    // Particle weight relative standard deviation (i.e., standard deviation / average)
    double weightRelativeStdDev() const;

    // Outputs a value [0.0, 1.0] indicating how the distribution's overall confidence is changing
    // Values decrease from 1.0 if recent confidence is worse than in the past
    double weightAvgRatio() const;

  private:
    // Recalculates distribution statistics
    void calcWeightStats();

    // Resets the particle sampler state to start from the beginning of the distribution
    void resetSampler();

  private:
    ParticleVector particles_;  // Particles in distribution
    size_t count_;              // Number of particles in use - in general less than particle vector size

    double weight_sum_;               // Sum of particle weights
    SmoothedWeight weight_avg_creep_; // Smoothed average particle weight, creep rate
    SmoothedWeight weight_avg_slow_;  // Smoothed average particle weight, slow rate
    SmoothedWeight weight_avg_fast_;  // Smoothed average particle weight, fast rate
    double weight_var_;               // Particle weight variance
    double weight_std_dev_;           // Particle weight standard deviation
    double weight_relative_std_dev_;  // Particle weight relative standard deviation

    size_t sample_s_;          // Sample index
    double sample_step_;       // Sample step size, this is 1 / size(distribution)
    double sample_sum_;        // Current weight sum
    double sample_sum_target_; // Target for weight sum

    ParticleEstimateHistogram hist_;  // Histogram for generating locally averaged pose estimates

    RNG rng_; // Random number generator
    std::uniform_real_distribution<double> prob_; // Distribution to generate a random probabilities (reals in [0, 1])
  };

} // namespace localize

#endif // DIST_H