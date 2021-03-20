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
    ParticleHistogram(const double x_res,   // Resolution for x position (meters per cell)
                      const double y_res,   // Resolution for y position (meters per cell)
                      const double th_res,  // Resolution for angle (rad per cell)
                      const Map& map        // Map
                     );

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
    const double x_res_;      // Resolution for x position (meters per cell)
    const double y_res_;      // Resolution for y position (meters per cell)
    const double th_res_;     // Resolution for heading angle (rad per cell)
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
  // The probability distribution is calculated by modeling the robot's motion
  // and sensor.
  class MCL
  {
  public:
    // Constructors
    MCL(const unsigned int mcl_num_particles_min,   // Minimum number of particles
        const unsigned int mcl_num_particles_max,   // Maximum number of particles
        const double mcl_kld_eps,                   // KL distance threshold
        const double mcl_hist_pos_res,              // Histogram resolution for x and y position (meters per cell)
        const double mcl_hist_th_res,               // Histogram resolution for heading angle (rad per cell)
        const double car_length,                    // Car length
        const double motion_lin_vel_n1,             // Motion model linear velocity noise coefficient 1
        const double motion_lin_vel_n2,             // Motion model linear velocity noise coefficient 2
        const double motion_ang_vel_n1,             // Motion model angular velocity noise coefficient 1
        const double motion_ang_vel_n2,             // Motion model angular velocity noise coefficient 2
        const double motion_th_n1,                  // Motion model final rotation noise coefficient 1
        const double motion_th_n2,                  // Motion model final rotation noise coefficient 2
        const float sensor_range_min,               // Sensor min range in meters
        const float sensor_range_max,               // Sensor max range in meters
        const float sensor_range_no_obj,            // Sensor range reported when nothing is detected
        const float sensor_range_std_dev,           // Sensor range standard deviation
        const float sensor_new_obj_decay_rate,      // Sensor model decay rate for new (unexpected) object probability
        const double sensor_weight_no_obj,          // Sensor model weight for no object detected probability
        const double sensor_weight_new_obj,         // Sensor model weight for new (unexpected) object probability
        const double sensor_weight_map_obj,         // Sensor model weight for map (expected) object probability
        const double sensor_weight_rand_effect,     // Sensor model weight for random effect probability
        const double sensor_uncertainty_factor,     // Sensor model uncertainty factor - extra noise added to calculation
        const unsigned int sensor_th_sample_count,  // Number of sampled sensor observations to use (count per revolution)
        const unsigned int sensor_th_raycast_count, // Number of angles for raycast (count per revolution)
        const double sensor_table_res,              // Sensor model lookup table resolution (meters per cell)
        const unsigned int map_width,               // Map number of pixels along x axis
        const unsigned int map_height,              // Map number of pixels along y axis
        const float map_x,                          // Map x translation of origin (cell 0,0) relative to world frame (meters)
        const float map_y,                          // Map y translation of origin (cell 0,0) relative to world frame (meters)
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
    void update(const RayScan&& obs);

    // Save particle distribution to file in CSV format
    void save(const std::string filename,
              const bool sort = true,
              const bool overwrite = true
             );
  private:
    // Generates an updated particle distribution by performing a combination of resampling (based on particle
    // weights) and random sampling
    // Source: KLD-Sampling: Adaptive Particle Filters (Fox 2001)
    void sample();

    // Indicates if the robot velocity is within the stopped threshold based on the last saved value
    bool stopped();

    // Updates the robot velocity with the input value and returns true if it is within the stopped threshold
    bool stopped(const double vel);

  private:
    size_t update_num_; // TBD remove
    std::recursive_mutex dist_mtx_; // Particle distribution mutex
    std::recursive_mutex vel_mtx_;  // Velocity mutex

    const size_t num_particles_min_;  // Minimum number of particles
    const double kld_eps_;            // KL distance threshold
    double vel_;                      // Robot linear velocity

    const Map map_;           // Map
    VelModel motion_model_;   // Motion model
    BeamModel sensor_model_;  // Sensor model

    ParticleDistribution dist_;           // Particle distribution
    ParticleVector samples_;              // Sampled particles (temporary storage)
    ParticleHistogram hist_;              // Histogram for estimating probability distribution complexity
    ParticleRandomSampler random_sample_; // Random particle sampler, generates samples in free space based on the map

    RNG rng_; // Random number generator
    std::uniform_real_distribution<double> prob_; // Distribution to generate a random probabilities (reals in [0, 1])
  };

} // namespace localize

#endif // MCL_H
