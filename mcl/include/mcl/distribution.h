#ifndef DISTRIBUTION_H
#define DISTRIBUTION_H

#include "mcl/common.h"
#include "mcl/histogram.h"

namespace localize
{
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
    const ParticleVector& estimates();

    // Reference to a particle in the distribution
    Particle& particle(const size_t p);

    // Samples a particle (with replacement) from the distribution with probability proportional to its weight
    const Particle& sample();

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

    void printWeightStats() const;

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

#endif // DISTRIBUTION_H