// Copyright (c) 2021-2023 Dane Roemer droemer7@gmail.com
// Distributed under the terms of the MIT License

#ifndef DISTRIBUTION_H
#define DISTRIBUTION_H

#include "amcl/common.h"
#include "amcl/histogram.h"

namespace localize
{
  class ParticleDistribution
  {
  public:
    // Constructors
    ParticleDistribution(const int max_count,   // Number of particles for which to reserve space
                         const Map& map         // Map
                        );

    // Populate distribution with new particle set
    void populate(const ParticleVector& particles, const int count = 0);

    // Populate distribution with new particle set and update statistics and reset sampler
    void update(const ParticleVector& particles, const int count = 0);

    // Update statistics and reset sampler
    void update();

    // Return the top particle estimates - lower indexes are better estimates
    const ParticleVector& estimates();

    // Reference to a particle in the distribution
    Particle& particle(const int i);

    // Number of particles in use - in general less than particle vector size
    int count() const;

    // Samples a particle (with replacement) from the distribution with probability proportional to its weight
    const Particle& sample();

    // Reset the particle sampler state to start from the beginning of the distribution
    void resetSampler();

    // Recalculate distribution statistics
    void calcWeightStats();

    // Reset distribution weight average history by setting all time-smoothed averages to the current actual value
    // This does not impact any estimates
    void resetWeightAvgHistory();

    // Average particle weight, current
    double weightAvgCurr() const;

    // Average particle weight, fast rate
    double weightAvgFast() const;

    // Average particle weight, slow rate
    double weightAvgSlow() const;

    // Average particle weight, creep rate
    double weightAvgCreep() const;

    // Outputs a value [0.0, 1.0] reflecting the ratio of <weight average slow> / <weight average creep>
    // Values less than 1.0 indicate recent confidence is worse than the past
    // A value of 1.0 indicates recent confidence is at least as good as the past
    double weightAvgRatio() const;

    // Particle weight variance
    double weightVar() const;

    // Particle weight standard deviation
    double weightStdDev() const;

    // Particle weight relative standard deviation (i.e., standard deviation / average)
    double weightRelStdDev() const;

    // Print weight statistics
    void printWeightStats() const;

  private:
    ParticleVector particles_;  // Particles in distribution
    int count_;                 // Number of particles in use - in general less than particle vector size

    double weight_sum_;               // Sum of particle weights
    double weight_avg_curr_;          // Current weight average, no smoothing
    SmoothedWeight weight_avg_creep_; // Smoothed average particle weight, creep rate
    SmoothedWeight weight_avg_slow_;  // Smoothed average particle weight, slow rate
    SmoothedWeight weight_avg_fast_;  // Smoothed average particle weight, fast rate
    double weight_var_;               // Particle weight variance
    double weight_std_dev_;           // Particle weight standard deviation
    double weight_relative_std_dev_;  // Particle weight relative standard deviation

    int sample_i_;                          // Sample index
    double sample_weight_with_correction_;  // Sample weight with correction from last sum iteration (for compensated summation)
    double sample_weight_sum_correction_;   // Sample weight sum correction term (for compensated summation)
    double sample_weight_sum_;              // Current sample weight sum
    double sample_weight_step_;             // Sample weight step size - this is 1.0 / count()
    double sample_weight_sum_target_;       // Target weight sum for next sample

    ParticleEstimateHistogram hist_;  // Histogram for generating locally averaged pose estimates

    RNG rng_; // Random number generator
    std::uniform_real_distribution<double> prob_; // Distribution to generate a random probabilities in [0, 1)
  };

} // namespace localize

#endif // DISTRIBUTION_H