#ifndef MCL_H
#define MCL_H

#include <mutex>
#include <vector>
#include <random>

#include "mcl/motion.h"
#include "mcl/sensor.h"
#include "mcl/util.h"

namespace localize
{
  // Monte-Carlo Localization
  // Localization technique to approximate the possible distribution of poses
  // by a set of random samples drawn from its probability distribution.
  // The probability distribution is calculated by modeling the robot's motion
  // and sensor.
  class MCL
  {
  public:
    // Constructors
    MCL(const unsigned int mcl_num_particles_min, // Minimum number of particles
        const unsigned int mcl_num_particles_max, // Maximum number of particles
        const double mcl_kld_eps,                 // KL distance threshold
        const double mcl_hist_pos_res,            // Histogram resolution for x and y position (meters per cell)
        const double mcl_hist_th_res,             // Histogram resolution for heading angle (rad per cell)
        const double car_length,                  // Car length
        const double motion_lin_vel_n1,           // Motion model linear velocity noise coefficient 1
        const double motion_lin_vel_n2,           // Motion model linear velocity noise coefficient 2
        const double motion_ang_vel_n1,           // Motion model angular velocity noise coefficient 1
        const double motion_ang_vel_n2,           // Motion model angular velocity noise coefficient 2
        const double motion_th_n1,                // Motion model final rotation noise coefficient 1
        const double motion_th_n2,                // Motion model final rotation noise coefficient 2
        const float sensor_range_min,             // Sensor min range in meters
        const float sensor_range_max,             // Sensor max range in meters
        const float sensor_range_no_obj,          // Sensor range reported when nothing is detected
        const float sensor_range_std_dev,         // Sensor range standard deviation
        const float sensor_th_sample_res,         // Sensor angle resolution at which to sample observations (rad per sample)
        const float sensor_th_raycast_res,        // Sensor angle resolution for raycast (rad per increment)
        const float sensor_new_obj_decay_rate,    // Sensor model decay rate for new (unexpected) object probability
        const double sensor_weight_no_obj,        // Sensor model weight for no object detected probability
        const double sensor_weight_new_obj,       // Sensor model weight for new (unexpected) object probability
        const double sensor_weight_map_obj,       // Sensor model weight for map (expected) object probability
        const double sensor_weight_rand_effect,   // Sensor model weight for random effect probability
        const double sensor_uncertainty_factor,   // Sensor model uncertainty factor - extra noise added to calculation
        const double sensor_table_res,            // Sensor model lookup table resolution (meters per cell)
        const unsigned int map_width,             // Map number of pixels along x axis
        const unsigned int map_height,            // Map number of pixels along y axis
        const float map_x,                        // Map x translation of origin (cell 0,0) relative to world frame (meters)
        const float map_y,                        // Map y translation of origin (cell 0,0) relative to world frame (meters)
        const float map_th,                       // Map angle relative to world frame (read)
        const float map_scale,                    // Map scale relative to world frame (meters per pixel)
        const std::vector<int8_t> map_data        // Map occupancy data in 1D vector, -1: Unknown, 0: Free, 100: Occupied
       );

    // Apply the motion model to update particle locations using
    // p(x[t] | u[t], x[t-1])
    void update(const double vel,
                const double steering_angle,
                const double dt
               );

    // Apply the sensor model to update particle weights using
    // p(obs[t] | pose[t], map)
    void update(const RayScan&& obs);

  private:
    // Update particle distribution
    // Samples from the current distribution using a low variance technique
    // until the relative entropy between the current distribution and the
    // reference distribution (estimated by sampling and counting the number
    // of histogram bins with support) is reduced to acceptable bounds
    // Source: KLD-Sampling: Adaptive Particle Filters (Fox 2001)
    void update(ParticleDistribution& dist);

    // Generate a random particle in free space
    Particle random();

    // Normalize particle weights
    void normalize(ParticleDistribution& dist,
                   double particles_weight_sum
                  );

    // Normalize particle weights
    void normalize(ParticleDistribution& dist);

    // Indicates if the robot velocity is within the stopped threshold based on
    // the last saved value
    bool stopped();

    // Updates the robot velocity with the input value and returns true if it
    // is within the stopped threshold
    bool stopped(const double vel);

    // Save particle distribution to file in CSV format
    void save(const std::string filename,
              const bool sort = true,
              const unsigned int precision = 0,
              const bool overwrite = true
             );

  private:
    size_t iteration; // TBD remove
    std::recursive_mutex particles_mtx_;  // Particle distribution mutex
    std::recursive_mutex vel_mtx_;        // Velocity mutex

    ParticleDistribution dist_;       // Particle distribution
    const size_t num_particles_min_;  // Minimum number of particles
    const size_t num_particles_max_;  // Maximum number of particles
    const double kld_eps_;            // KL distance threshold
    double vel_;                      // Robot velocity

    const Map map_;             // Map
    VelModel motion_model_;     // Motion model
    BeamModel sensor_model_;    // Sensor model
    ParticleHistogram hist_;    // Histogram for estimating relative entropy

    RNG rng_;  // Random number engine
    std::uniform_real_distribution<double> sample_dist_;  // Distribution [0, 1) for sampling
    std::uniform_real_distribution<double> x_dist_;       // Distribution of map x locations relative to world frame
    std::uniform_real_distribution<double> y_dist_;       // Distribution of map y locations relative to world frame
    std::uniform_real_distribution<double> th_dist_;      // Distribution of theta [-pi, +pi) relative to world frame
  };

} // namespace localize

#endif // MCL_H
