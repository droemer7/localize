#include <float.h>
#include <math.h>
#include <random>

#include "mcl/mcl.h"

using namespace localize;

MCL::MCL(const unsigned int num_particles,
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
        ) :
  motion_model_(car_length,
                motion_lin_vel_n1,
                motion_lin_vel_n2,
                motion_ang_vel_n1,
                motion_ang_vel_n2,
                motion_th_n1,
                motion_th_n2
               ),
  sensor_model_(sensor_range_min,
                sensor_range_max,
                sensor_range_no_obj,
                sensor_range_std_dev,
                sensor_new_obj_decay_rate,
                sensor_weight_no_obj,
                sensor_weight_new_obj,
                sensor_weight_map_obj,
                sensor_weight_rand_effect,
                sensor_table_size
               ),
  map_(map_height,
       map_width,
       map_m_per_pxl,
       map_th,
       map_origin_x,
       map_origin_y,
       map_occ_data
      ),
  particles_(num_particles),
  weights_(num_particles)
{}

void MCL::motionUpdate(const double vel,
                       const double steering_angle,
                       const double dt
                      )
{
  std::lock_guard<std::mutex> lock(particle_mtx_);
  motion_model_.apply(vel,
                      steering_angle,
                      dt,
                      particles_
                     );
}

void MCL::sensorUpdate(const std::vector<float>& ranges_obs)
{
  std::lock_guard<std::mutex> lock(particle_mtx_);
  sensor_model_.apply(ranges_obs,
                      particles_,
                      map_,
                      weights_
                     );
  // TBD resample after sensor model when appropriate
}

// void MCL::reset()
// {
//   // Generate a uniform distribution within the range [-sigma, sigma]
//   std::uniform_real_distribution<double> xy_dist(0, std::nextafter(, DBL_MAX));
//   std::uniform_real_distribution<double> th_dist(-M_PI)
//   double val = 0.0;

//   for (Pose & particle : particles_) {
//     particle.x_ = int_dist()
//   }

// }