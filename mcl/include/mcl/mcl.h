#ifndef MCL_H
#define MCL_H

#include <mutex>
#include <vector>
#include <random>

#include "mcl/dist.h"
#include "mcl/motion.h"
#include "mcl/sensor.h"
#include "mcl/common.h"

namespace localize
{
  // A sampler which chooses a random particle in free space
  class ParticleRandomSampler
  {
  public:
    // Constructors
    explicit ParticleRandomSampler(const Map& map);

    // Generates a random particle in free space
    Particle operator()();

  private:
    const Map& map_;  // Occupancy map

    RNG rng_; // Random number generator

    std::uniform_real_distribution<double> x_dist_;     // Distribution of map x locations relative to world frame
    std::uniform_real_distribution<double> y_dist_;     // Distribution of map y locations relative to world frame
    std::uniform_real_distribution<double> th_dist_;    // Distribution of theta [-pi, +pi) relative to world frame
  };

  // 3D boolean histogram representing occupancy of pose space (x, y, th)
  class ParticleHistogram
  {
  public:
    // Constructors
    ParticleHistogram(const Map& map);  // Map

    // Update histogram occupancy with the particle's location
    // Returns true if the particle fell into a new (unoccupied) cell, increasing the occupancy count
    bool update(const Particle& particle);

    // Histogram occupancy count
    size_t count() const;

    // Clear histogram and reset occupancy count
    void clear();

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

  // Monte-Carlo Localization
  // Localization technique to approximate the possible distribution of poses
  // by a set of random samples drawn from its probability distribution.
  class MCL
  {
  public:
    // Constructors
    MCL(const unsigned int mcl_num_particles_min,   // Minimum number of particles
        const unsigned int mcl_num_particles_max,   // Maximum number of particles
        const double car_length,                    // Car length
        const float sensor_range_min,               // Sensor min range in meters
        const float sensor_range_max,               // Sensor max range in meters
        const float sensor_range_no_obj,            // Sensor range reported when nothing is detected
        const unsigned int map_width,               // Map number of pixels along x axis
        const unsigned int map_height,              // Map number of pixels along y axis
        const float map_x_origin,                   // Map x translation of origin (cell 0,0) relative to world frame (meters)
        const float map_y_origin,                   // Map y translation of origin (cell 0,0) relative to world frame (meters)
        const float map_th,                         // Map angle relative to world frame (read)
        const float map_scale,                      // Map scale relative to world frame (meters per pixel)
        const std::vector<int8_t> map_data          // Map occupancy data in 1D vector, -1: Unknown, 0: Free, 100: Occupied
       );

    // Apply the motion model to update particle locations using p(x[t] | u[t], x[t-1])
    void update(const double vel,
                const double steering_angle,
                const double dt
               );

    // Apply the sensor model to update particle weights using p(obs[t] | pose[t], map)
    void update(const RayScan& obs);

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

    // Indicates if the robot velocity is within the stopped threshold based on the last saved value
    bool stopped();

    // Updates the robot velocity with the input value and returns true if it is within the stopped threshold
    bool stopped(const double vel);

  private:
    size_t update_num_; // TBD remove
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
    ParticleHistogram hist_;              // Histogram for estimating probability distribution complexity
    ParticleRandomSampler random_sample_; // Random particle sampler, generates samples in free space based on the map

    RNG rng_; // Random number generator
    std::uniform_real_distribution<double> prob_; // Distribution to generate a random probabilities (reals in [0, 1])
  };

} // namespace localize

#endif // MCL_H
