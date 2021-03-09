#include <cmath>
#include <float.h>
#include <random>

#include "mcl/mcl.h"

static const double STOPPED_THRESHOLD = 1e-5; // Threshold for considering robot stopped (defers updates)
static const double Z_P_99 = 2.3263478740;    // Z score for P(0.99) of Normal(0,1) distribution
static const double F_2_9 = 2 / 9;            // Fraction 2/9

using namespace localize;

MCL::MCL(const unsigned int mcl_num_particles_min,
         const unsigned int mcl_num_particles_max,
         const double mcl_kld_eps,
         const double mcl_hist_occ_weight_min,
         const double mcl_hist_pos_res,
         const double mcl_hist_th_res,
         const double car_length,
         const double motion_lin_vel_n1,
         const double motion_lin_vel_n2,
         const double motion_ang_vel_n1,
         const double motion_ang_vel_n2,
         const double motion_th_n1,
         const double motion_th_n2,
         const float sensor_range_min,
         const float sensor_range_max,
         const float sensor_range_no_obj,
         const float sensor_range_std_dev,
         const float sensor_th_sample_res,
         const float sensor_th_raycast_res,
         const float sensor_new_obj_decay_rate,
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
  iteration(0),
  particles_(mcl_num_particles_max),
  num_particles_min_(mcl_num_particles_min == 0 ? 1 : mcl_num_particles_min),
  num_particles_max_(mcl_num_particles_max),
  num_particles_curr_(0),
  kld_eps_(mcl_kld_eps),
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
  hist_(mcl_hist_pos_res,
        mcl_hist_pos_res,
        mcl_hist_th_res,
        mcl_hist_occ_weight_min,
        map_
       ),
  sample_dist_(0.0, 1.0),
  x_dist_(map_.x, std::nextafter(map_.width * map_.scale + map_.x, DBL_MAX)),
  y_dist_(map_.y, std::nextafter(map_.height * map_.scale + map_.y, DBL_MAX)),
  th_dist_(-M_PI, M_PI)
{
  if (num_particles_min_ > num_particles_max_) {
    throw std::runtime_error("MCL: num_particles_min > num_particles_max\n");
  }
  for (size_t i = 0; i < num_particles_max_; ++i) {
    particles_[i] = random();
  }
}

void MCL::update(const double vel,
                 const double steering_angle,
                 const double dt
                )
{
  if (!stopped(vel)) {
    motion_model_.update(particles_, vel, steering_angle, dt);
  }
}

void MCL::update(const RayScan&& obs)
{
  sensor_model_.update(obs);
  //if (!stopped()) {
    update(particles_);
  //}
  if (iteration++ >= 2) {
    save("particles.csv");
    throw std::runtime_error("Finished");
  }
}

void MCL::update(ParticleVector& particles)
{
  RecursiveLock lock(particles_mtx_);

  double sample_width = 0.0;
  double sum_target = 0.0;
  double sum_curr = 0.0;
  double weight_sum = 0.0;
  double num_particles_target = 0.0;
  double chi_sq_term_1 = 0.0;
  double chi_sq_term_2 = 0.0;
  bool repeat = false;
  size_t s = 0;
  size_t p = 0;
  size_t k = 0;

  // Clear histogram
  hist_.reset();

  // Initialize target and current weight sums
  if (num_particles_curr_ > 0) {
    sample_width = 1.0 / num_particles_curr_;
    sum_target = sample_dist_(rng_.engine()) * sample_width;
    sum_curr = particles[0].weight_;
  }
  // Generate samples until we exceed both the minimum and target number of
  // samples, or reach the maximum allowed
  while (   (   s < num_particles_target
             || s < num_particles_min_
            )
         && s < num_particles_max_
        ) {
    // Draw a particle from the current distribution with probability
    // proportional to its weight
    if (s < num_particles_curr_) {
      repeat = (s > 0);

      // Sum weights until we reach the target sum
      while (sum_curr < sum_target) {
        sum_curr += particles[++p].weight_;
        repeat = false;
      }
      // Add to sample set and increase target sum
      // Note that s <= p always given the above while loop
      particles[s] = particles[p];
      sum_target += sample_width;
    }
    // Exhausted current distribution so generate a new random particle
    else {
      repeat = false;
      particles[s] = random();
    }
    // Calculate particle importance weight
    // If it's a duplicate, copy the weight from the previous sample
    if (repeat) {
      particles[s].weight_ = particles[s - 1].weight_;
    }
    // If it's a new sample, calculate the weight using the sensor model
    else {
      sensor_model_.update(particles[s]);
    }
    weight_sum += particles[s].weight_;

    // Update histogram, incrementing k if the number of occupied histogram
    // cells increases
    if (hist_.update(particles[s])) {
      ++k;
      // Update target number of samples based on k and error bounds
      // Wilson-Hilferty transformation of chi-square distribution
      if (k > 1) {
        chi_sq_term_1 = (k - 1.0) / (2.0 * kld_eps_);
        chi_sq_term_2 = 1.0 - F_2_9 / (k - 1.0) + Z_P_99 * std::sqrt(F_2_9 / (k - 1.0));
        num_particles_target = chi_sq_term_1 * chi_sq_term_2 * chi_sq_term_2 * chi_sq_term_2;
      }
    }
    ++s;
  }
  // Update the number of particles we're using
  num_particles_curr_ = s;
  printf("Samples used = %lu\n", s);
  printf("Samples target = %lu\n", static_cast<size_t>(num_particles_target));
  printf("Histogram count = %lu\n", k);

  // Normalize weights
  normalize(particles, weight_sum);

  return;
}

Particle MCL::random()
{
  Particle particle;

  bool occupied = true;

  // Regenerate x & y until free space is found
  while (occupied) {
    particle.x_ = x_dist_(rng_.engine());
    particle.y_ = y_dist_(rng_.engine());
    occupied = map_.occupied(particle.x_, particle.y_);
  }
  // Any theta is allowed
  particle.th_ = th_dist_(rng_.engine());

  // Particle weight is zero (unknown) until calculated by the sensor model
  return particle;
}

void MCL::normalize(ParticleVector& particles,
                    double weight_sum
                   )
{
  RecursiveLock lock(particles_mtx_);

  double normalizer = 0.0;

  if (weight_sum > 0.0) {
    normalizer = 1 / weight_sum;
  }
  for (size_t i = 0; i < num_particles_curr_; ++i) {
    particles[i].weight_ *= normalizer;
  }
  return;
}

void MCL::normalize(ParticleVector& particles)
{
  RecursiveLock lock(particles_mtx_);

  double weight_sum = 0.0;

  for (size_t i = 0; i < num_particles_curr_; ++i) {
    weight_sum += particles[i].weight_;
  }
  normalize(particles, weight_sum);

  return;
}

bool MCL::stopped()
{
  RecursiveLock lock(vel_mtx_);

  return std::abs(vel_) < STOPPED_THRESHOLD;
}

bool MCL::stopped(const double vel)
{
  RecursiveLock lock(vel_mtx_);
  vel_ = vel;

  return stopped();
}

void MCL::save(const std::string filename,
               const bool sort,
               const unsigned int precision,
               const bool overwrite
              )
{
  RecursiveLock lock(particles_mtx_);

  ParticleVector particles = particles_;
  particles.resize(num_particles_curr_);

  if (sort) {
    localize::sort(particles);
  }
  localize::save(particles,
                 filename,
                 precision,
                 overwrite
                );
}