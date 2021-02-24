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
    MCL(const unsigned int num_particles,       // Number of particles
        const double car_length,                // Car length
        const double motion_lin_vel_n1,         // Motion model linear velocity noise coefficient 1
        const double motion_lin_vel_n2,         // Motion model linear velocity noise coefficient 2
        const double motion_ang_vel_n1,         // Motion model angular velocity noise coefficient 1
        const double motion_ang_vel_n2,         // Motion model angular velocity noise coefficient 2
        const double motion_th_n1,              // Motion model final rotation noise coefficient 1
        const double motion_th_n2,              // Motion model final rotation noise coefficient 2
        const double sensor_range_min,          // Sensor min range in meters
        const double sensor_range_max,          // Sensor max range in meters
        const double sensor_range_no_obj,       // Sensor range reported when nothing is detected
        const double sensor_range_std_dev,      // Sensor range standard deviation
        const double sensor_th_sample_inc,      // Sensor angle increment at which to sample observations (rad / sample)
        const double sensor_new_obj_decay_rate, // Sensor model decay rate for unexpected object probability
        const double sensor_weight_no_obj,      // Sensor model weight for no object detected probability
        const double sensor_weight_new_obj,     // Sensor model weight for new (unexpected) object probability
        const double sensor_weight_map_obj,     // Sensor model weight for map (expected) object probability
        const double sensor_weight_rand_effect, // Sensor model weight for random effect probability
        const double sensor_uncertainty_factor, // Sensor model uncertainty factor - extra noise added to calculation
        const unsigned int sensor_table_size,   // Sensor model lookup table size
        const unsigned int map_width,           // Map number of pixels along x axis
        const unsigned int map_height,          // Map number of pixels along y axis
        const float map_x,                      // Map x translation of origin (cell 0,0) relative to world frame
        const float map_y,                      // Map y translation of origin (cell 0,0) relative to world frame
        const float map_th,                     // Map angle relative to world frame
        const float map_scale,                  // Map scale relative to world frame (meters per pixel)
        const std::vector<int8_t> map_data      // Map occupancy data in 1D vector, -1: Unknown, 0: Free, 100: Occupied
       );

    // Applies the motion model to generate new samples of particles from
    // p(x[t] | u[t], x[t-1])
    void motionUpdate(const double vel,
                      const double steering_angle,
                      const double dt
                     );

    // Applies the sensor model to calculate particle importance weights from
    // p(ranges[t] | pose[t], map)
    void sensorUpdate(const std::vector<Ray>& rays);

    // Resets particle distribution, randomizing each particle's pose using a
    // uniform distribution across free space
    void reset();

    // Saves particle distribution to file in CSV format
    void save(const std::string filename,
              const unsigned int precision = 0,
              const bool overwrite = true
             );

  private:
    std::vector<PoseWithWeight> particles_; // Particle distribution
    const Map map_;                         // Map
    VelModel motion_model_;                 // Motion model
    BeamModel sensor_model_;                // Sensor model

    RNG rng_;  // Random number engine

    std::uniform_real_distribution<double> x_uni_dist_;   // Uniform real distribution [0, map width]
    std::uniform_real_distribution<double> y_uni_dist_;   // Uniform real distribution [0, map height]
    std::uniform_real_distribution<double> th_uni_dist_;  // Uniform real distribution [-pi, +pi)

    std::mutex mtx_; // Particle mutex
  };

} // namespace localize

#endif // MCL_H
