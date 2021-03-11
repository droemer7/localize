#include <cmath>
#include <float.h>
#include <random>

#include "mcl/mcl.h"

static const double STOPPED_THRESHOLD = 1e-10; // Threshold for considering robot stopped (defers updates)
static const double Z_P_99 = 2.3263478740;     // Z score for P(0.99) of Normal(0,1) distribution
static const double F_2_9 = 2 / 9;             // Fraction 2/9

// TBD remove
static const int LAST_ITERATION = 1;

using namespace localize;

MCL::MCL(const unsigned int mcl_num_particles_min,
         const unsigned int mcl_num_particles_max,
         const double mcl_kld_eps,
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
  dist_(ParticleVector(mcl_num_particles_max),
        0.01,
        0.9
       ),
  num_particles_min_(mcl_num_particles_min == 0 ? 1 : mcl_num_particles_min),
  num_particles_max_(mcl_num_particles_max),
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
        map_
       ),
  sample_dist_(0.0, 1.0),
  x_dist_(map_.x_origin, std::nextafter(map_.width * map_.scale + map_.x_origin, DBL_MAX)),
  y_dist_(map_.y_origin, std::nextafter(map_.height * map_.scale + map_.y_origin, DBL_MAX)),
  th_dist_(-M_PI, M_PI)
{
  if (num_particles_min_ > num_particles_max_) {
    throw std::runtime_error("MCL: num_particles_min > num_particles_max\n");
  }
  for (size_t i = 0; i < num_particles_max_; ++i) {
    dist_.particles_[i] = random();
  }
}

void MCL::update(const double vel,
                 const double steering_angle,
                 const double dt
                )
{
  if (!stopped(vel)) {
    RecursiveLock lock(particles_mtx_);
    motion_model_.update(dist_.particles_, dist_.num_particles_, vel, steering_angle, dt);
  }
}

void MCL::update(const RayScan&& obs)
{
  sensor_model_.update(obs);

  // TBD remove true
  if (true || !stopped()) {
    RecursiveLock lock(particles_mtx_);
    update(dist_);
    ++iteration;
  }
  if (iteration > LAST_ITERATION) {
    save("particles.csv");
    throw std::runtime_error("Finished");
  }
}

void MCL::update(ParticleDistribution& dist)
{
  RecursiveLock lock(particles_mtx_);
  Particle particle_p;
  bool repeat = false;

  // Move target into ParticleDistribution
  // Create a second ParticleDistribution for sample
  double sample_weight_width = 0.0;
  double sample_weight_sum_target = 0.0;
  double sample_weight_sum = 0.0;
  double particles_weight_sum = 0.0;
  double num_particles_target = num_particles_min_;
  double chi_sq_term_1 = 0.0;
  double chi_sq_term_2 = 0.0;
  size_t hist_count = 0;
  size_t s = 0;
  size_t p = 0;
  size_t p_prev = (size_t)-1;

  // Reset histogram
  hist_.reset();

  // Initialize target and current weight sums
  if (dist.num_particles_ > 0) {
    sample_weight_width = 1.0 / dist.num_particles_;
    sample_weight_sum_target = sample_dist_(rng_.engine()) * sample_weight_width;
    sample_weight_sum = dist.particles_[0].weight_;
  }
  // Generate samples until we exceed both the minimum and target number of samples, or reach the maximum allowed
  while (   (   s < num_particles_target
             || s < num_particles_min_
            )
         && s < num_particles_max_
        ) {
    // Draw a particle from the current distribution with probability proportional to its weight
    // TBD **********
    // Try draw without low variance technique? without replacement?
    // Implement slow and fast weight average to draw random samples
    if (   s < dist.num_particles_
        && p < dist.num_particles_
       ) {
      // Sum weights until we reach the target
      while (   sample_weight_sum < sample_weight_sum_target
             && p + 1 < dist.num_particles_
            ) {
        sample_weight_sum += dist.particles_[++p].weight_;
      }
      // Make sure we actually did reach the target weight sum
      if (sample_weight_sum >= sample_weight_sum_target) {
        // If it's not a repeat particle, update its weight and save to avoid recalculating the weight repeatedly for
        // the same particle
        if (p != p_prev) {
          particle_p = dist.particles_[p];
          sensor_model_.update(particle_p);
          p_prev = p;
        }
        // Add to sample set and increase target sum
        dist.particles_[s] = particle_p;
        sample_weight_sum_target += sample_weight_width;
      }
    }
    // Exhausted current distribution so generate a new random particle and update its weight
    else {
      repeat = false;
      dist.particles_[s] = random();
      sensor_model_.update(dist.particles_[s]);
    }
    // Update weight terms
    particles_weight_sum += dist.particles_[s].weight_;


    // Update histogram, incrementing k if the number of occupied histogram cells increases
    if (hist_.update(dist.particles_[s])) {
      ++hist_count;
      // Update target number of samples based on k and error bounds
      // Wilson-Hilferty transformation of chi-square distribution
      if (hist_count > 1) {
        chi_sq_term_1 = (hist_count - 1.0) / (2.0 * kld_eps_);
        chi_sq_term_2 = 1.0 - F_2_9 / (hist_count - 1.0) + Z_P_99 * std::sqrt(F_2_9 / (hist_count - 1.0));
        num_particles_target = chi_sq_term_1 * chi_sq_term_2 * chi_sq_term_2 * chi_sq_term_2;
      }
    }
    ++s;
  }
  // Update the number of particles we're using
  dist.num_particles_ = s;
  printf("*** Iteration %lu ***\n", iteration);
  printf("Samples used = %lu\n", s);
  printf("Samples target = %lu\n", static_cast<size_t>(num_particles_target));
  printf("Histogram count = %lu\n", hist_count);
  printf("---------------------------------\n");

  // Normalize weights
  if (iteration == LAST_ITERATION) {
    save("particles_prenorm_last_iteration.csv");
  }
  normalize(dist);

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

void MCL::normalize(ParticleDistribution& dist)
{
  RecursiveLock lock(particles_mtx_);
  double normalizer = 0.0;

  if (dist.weight_sum_ > 0.0) {
    normalizer = 1 / dist.weight_sum_;
  }
  else {
    for (size_t i = 0; i < dist.num_particles_; ++i) {
      dist.weight_sum_ += dist.particles_[i].weight_;
    }
  }
  for (size_t i = 0; i < dist.num_particles_; ++i) {
    dist.particles_[i].weight_ *= normalizer;
  }
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
  ParticleVector particles = dist_.particles_;
  particles.resize(dist_.num_particles_);

  if (sort) {
    localize::sort(particles);
  }
  localize::save(particles,
                 filename,
                 precision,
                 overwrite
                );
}