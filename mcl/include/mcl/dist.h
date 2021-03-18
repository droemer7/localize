#ifndef DIST_H
#define DIST_H

#include "mcl/util.h"

namespace localize
{
  class ParticleDistribution
  {
  public:
    // Constructors
    ParticleDistribution();

    ParticleDistribution(const size_t max_count);  // Number of particles for which to reserve space

    ParticleDistribution(const ParticleVector& particles, // Particles
                         const size_t count               // Number of particles in use
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

    // Reference to a particle in the distribution
    Particle& particle(size_t p);

    // Samples a particle (with replacement) from the distribution with probability proportional to its weight
    Particle& sample();

    // Number of particles in use - in general less than particle vector size
    size_t count() const;

    // Average particle weight
    double weightAvg() const;

    // Particle weight variance
    double weightVar() const;

    // Particle weight standard deviation
    double weightStdDev() const;

    // Particle weight relative standard deviation (i.e., standard deviation / average)
    double weightRelativeStdDev() const;

    // Outputs a value [0.0, 1.0] indicating how the distribution's overall confidence is changing
    // A value of 1.0 indicates the distribution's average confidence is better now compared to the past
    // Values less than 1.0 indicate the distribution's average confidence is worse now compared to the past
    double weightAvgRatio() const;

  private:
    // Recalculates distribution statistics
    void calcWeightStats();

    // Resets the particle sampler state to start from the beginning of the distribution
    void resetSampler();

  private:
    ParticleVector particles_;              // Particles in distribution
    size_t count_;                          // Number of particles in use - in general less than particle vector size
    double weight_sum_;                     // Sum of particle weights
    double weight_avg_;                     // Average particle weight
    SmoothedValue<double> weight_avg_slow_; // Smoothed average particle weight, slow rate
    SmoothedValue<double> weight_avg_fast_; // Smoothed average particle weight, fast rate
    double weight_var_;                     // Particle weight variance
    double weight_std_dev_;                 // Particle weight standard deviation
    double weight_relative_std_dev_;        // Particle weight relative standard deviation

    size_t sample_s_;          // Sample index
    double sample_step_;       // Sample step size, this is 1 / size(distribution)
    double sample_sum_;        // Current weight sum
    double sample_sum_target_; // Target for weight sum

    RNG rng_; // Random number generator
    std::uniform_real_distribution<double> prob_; // Distribution to generate a random probabilities (reals in [0, 1])
  };

} // namespace localize

#endif // DIST_H