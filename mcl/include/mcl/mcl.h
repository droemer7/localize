#ifndef MCL_H
#define MCL_H

#include <vector>

#include "mcl/motion.h"
#include "mcl/sensor.h"
#include "mcl/util.h"

#include "includes/RangeLib.h"

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
    MCL(const unsigned int num_particles,
        const double car_length,
        const double motion_lin_vel_n1,
        const double motion_lin_vel_n2,
        const double motion_ang_vel_n1,
        const double motion_ang_vel_n2,
        const double motion_th_n1,
        const double motion_th_n2,
        const double sensor_range_min,
        const double sensor_range_max,
        const double sensor_range_no_obj,
        const double sensor_range_std_dev,
        const double sensor_new_obj_decay_rate,
        const double sensor_weight_no_obj,
        const double sensor_weight_new_obj,
        const double sensor_weight_map_obj,
        const double sensor_weight_rand_effect,
        const size_t sensor_table_size,
        const uint32_t map_height,
        const uint32_t map_width,
        const float map_m_per_pxl,
        const double map_th,
        const double map_origin_x,
        const double map_origin_y,
        const std::vector<int8_t> map_occ_data
       );

    // Applies the motion model to generate new samples of particles from
    // p(x[t] | u[t], x[t-1])
    void motionUpdate(const double vel,
                      const double steering_angle,
                      const double dt
                     );

    // Applies the sensor model to calculate particle importance weights from
    // p(ranges[t] | pose[t], map)
    void sensorUpdate(const std::vector<float>& ranges_obs);

    // TBD void reset();

  private:
    VelModel motion_model_;   // Motion model
    BeamModel sensor_model_;  // Sensor model
    Map map_;                 // Map (converted from ROS coordinate space)

    std::vector<Pose> particles_; // Particle distribution
    std::vector<double> weights_; // Particle importance weights

    RNG rng; // Random number engine

    std::mutex particle_mtx_; // Particle mutex
  };

} // namespace localize

#endif // MCL_H
