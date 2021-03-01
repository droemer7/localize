#include <cmath>
#include <float.h>
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
         const double sensor_th_sample_res,
         const double sensor_th_raycast_res,
         const double sensor_new_obj_decay_rate,
         const double sensor_weight_no_obj,
         const double sensor_weight_new_obj,
         const double sensor_weight_map_obj,
         const double sensor_weight_rand_effect,
         const double sensor_uncertainty_factor,
         const double sensor_table_res,
         const unsigned int map_width,
         const unsigned int map_height,
         const float map_x,
         const float map_y,
         const float map_th,
         const float map_scale,
         const std::vector<int8_t> map_data
        ) :
  particles_(num_particles),
  vel_(0.0),
  map_(map_width,
       map_height,
       map_x,
       map_y,
       map_th,
       map_scale,
       map_data
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
                sensor_th_sample_res,
                sensor_th_raycast_res,
                sensor_new_obj_decay_rate,
                sensor_weight_no_obj,
                sensor_weight_new_obj,
                sensor_weight_map_obj,
                sensor_weight_rand_effect,
                sensor_uncertainty_factor,
                sensor_table_res,
                map_
               ),
  x_uni_dist_(map_.x, std::nextafter(map_.width * map_.scale + map_.x, DBL_MAX)),
  y_uni_dist_(map_.y, std::nextafter(map_.height * map_.scale + map_.y, DBL_MAX)),
  th_uni_dist_(-M_PI, M_PI)
{
  reset();
}

void MCL::motionUpdate(const double vel,
                       const double steering_angle,
                       const double dt
                      )
{
  // Save speed so we can skip motion & sensor updates if stopped
  {
    std::lock_guard<std::mutex> lock(vel_mtx_);
    vel_ = vel;
  }
  // Only do motion update if moving
  if (!stopped()) {
    std::lock_guard<std::mutex> lock(particles_mtx_);
    motion_model_.apply(vel,
                        steering_angle,
                        dt,
                        particles_
                       );
  }
}

void MCL::sensorUpdate(const std::vector<Ray>& rays)
{
  // Only do sensor update if moving
  //if (!stopped()) {
    std::lock_guard<std::mutex> lock(particles_mtx_);
    sensor_model_.applyLookup(rays,
                              particles_
                             );
  //}
}

void MCL::reset()
{
  bool occupied = true;
  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  std::lock_guard<std::mutex> lock(particles_mtx_);

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
    // Any theta is allowed
    particle.th_ = th_uni_dist_(rng_.engine());
    particle.weight_ = 0.0;
  }
  // TBD remove
  particles_[0].x_ = -0.035;
  particles_[0].y_ = 0.0;
  particles_[0].th_ = 0.0;
}

void MCL::save(const std::string filename,
               const unsigned int precision,
               const bool overwrite
              )
{
  std::lock_guard<std::mutex> lock(particles_mtx_);
  sort(particles_, compWeight);
  localize::save(particles_,
                 filename,
                 precision,
                 overwrite
                );
}

bool MCL::stopped()
{
  std::lock_guard<std::mutex> lock(vel_mtx_);

  return std::abs(vel_) < FLT_EPSILON;
}