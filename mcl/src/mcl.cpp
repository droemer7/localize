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
         const double sensor_angle_sample_inc,
         const double sensor_new_obj_decay_rate,
         const double sensor_weight_no_obj,
         const double sensor_weight_new_obj,
         const double sensor_weight_map_obj,
         const double sensor_weight_rand_effect,
         const double sensor_uncertainty_factor,
         const unsigned int sensor_table_size,
         const unsigned int map_width,
         const unsigned int map_height,
         const float map_m_per_pxl,
         const double map_th,
         const double map_origin_x,
         const double map_origin_y,
         const std::vector<int8_t> map_occ_data
        ) :
  particles_(num_particles),
  x_uni_dist_(0.0, std::nextafter(map_width, UINT32_MAX)),
  y_uni_dist_(0.0, std::nextafter(map_height, UINT32_MAX)),
  th_uni_dist_(-M_PI, M_PI),
  map_(map_width,
       map_height,
       map_m_per_pxl,
       map_th,
       map_origin_x,
       map_origin_y,
       map_occ_data
      ),
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
                sensor_angle_sample_inc,
                sensor_new_obj_decay_rate,
                sensor_weight_no_obj,
                sensor_weight_new_obj,
                sensor_weight_map_obj,
                sensor_weight_rand_effect,
                sensor_uncertainty_factor,
                sensor_table_size,
                map_
               )
{
  reset();
}

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

void MCL::sensorUpdate(const std::vector<Ray>& rays)
{
  std::lock_guard<std::mutex> lock(particle_mtx_);
  sensor_model_.applyLookup(rays,
                            particles_
                           );
  throw std::runtime_error("Sample complete");
}

void MCL::reset()
{
  bool occupied = true;
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  for (PoseWithWeight & particle : particles_) {
    occupied = true;

    // Regenerate x & y until free space is found
    while (occupied) {
      particle.x_ = x_uni_dist_(rng_.engine());
      particle.y_ = y_uni_dist_(rng_.engine());
      occupied = map_.isOccupied(particle.x_,
                                 particle.y_
                                );
    }
    particle.th_ = th_uni_dist_(rng_.engine());
    particle.weight_ = 0.0;
  }
}