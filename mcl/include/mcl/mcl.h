#ifndef MCL_H
#define MCL_H

#include "mcl/common.h"
#include "mcl/distribution.h"
#include "mcl/histogram.h"
#include "mcl/motion.h"
#include "mcl/sensor.h"

namespace localize
{
  // Monte-Carlo Localization
  // Localization technique to approximate the possible distribution of poses
  // by a set of random samples drawn from its probability distribution.
  class MCL
  {
  public:
    // Constructors
    MCL(const unsigned int num_particles_min, // Minimum number of particles
        const unsigned int num_particles_max, // Maximum number of particles
        const double car_length,              // Car length
        const float sensor_range_min,         // Sensor min range in meters
        const float sensor_range_max,         // Sensor max range in meters
        const float sensor_range_no_obj,      // Sensor range reported when nothing is detected
        const unsigned int map_width,         // Map number of pixels along x axis
        const unsigned int map_height,        // Map number of pixels along y axis
        const float map_x_origin,             // Map x translation of origin (cell 0,0) relative to world frame (meters)
        const float map_y_origin,             // Map y translation of origin (cell 0,0) relative to world frame (meters)
        const float map_th,                   // Map angle relative to world frame (read)
        const float map_scale,                // Map scale relative to world frame (meters per pixel)
        const std::vector<int8_t> map_data    // Map occupancy data in 1D vector, -1: Unknown, 0: Free, 100: Occupied
       );

    // Apply the motion model to update particle locations using p(x[t] | u[t], x[t-1])
    void update(const double vel,
                const double steering_angle,
                const double dt
               );

    // Apply the sensor model to update particle weights using p(obs[t] | pose[t], map)
    void update(const RayScan& obs);

    // Return the top particle estimates - lower indexes are better estimates
    ParticleVector estimates();

    // Return the particle estimate from the list by index - lower indexes are better estimates
    Particle estimate(const size_t e = 0);

    // Indicates if the robot velocity is within the stopped threshold based on the last saved value
    bool stopped();

    // Save particle distribution to file in CSV format
    void save(const std::string filename,
              const bool sort = true,
              const bool overwrite = true
             );

  private:
    // Update particle distribution by performing resampling (based on particle weights) and/or random sampling
    // No changes are made if distribution is ok
    // Source: KLD-Sampling: Adaptive Particle Filters (Fox 2001)
    void update();

    // Return true if distribution should be resampled
    bool resampleRequired();

    // Return true if random samples should be added to the distribution
    bool randomSampleRequired();

    // Updates the robot velocity with the input value and returns true if it is within the stopped threshold
    bool stopped(const double vel);

  private:
    std::recursive_mutex dist_mtx_; // Particle distribution mutex
    std::recursive_mutex vel_mtx_;  // Velocity mutex

    const size_t num_particles_min_;  // Minimum number of particles
    double vel_;                      // Robot linear velocity
    const Map map_;                   // Map
    VelModel motion_model_;           // Motion model
    BeamModel sensor_model_;          // Sensor model
    ParticleVector particle_data_;    // Particle data saved for offline tuning / analysis
    RayScanVector sensor_data_;       // Sensor data saved for offline tuning / analysis

    ParticleDistribution dist_;           // Particle distribution
    ParticleVector samples_;              // Sampled particles (temporary storage)
    ParticleOccupancyHistogram hist_;     // Histogram for estimating probability distribution complexity
    ParticleRandomSampler random_sample_; // Random particle sampler, generates samples in free space based on the map

    RNG rng_; // Random number generator
    std::uniform_real_distribution<double> prob_; // Distribution to generate a random probabilities (reals in [0, 1])
  };

} // namespace localize

#endif // MCL_H
