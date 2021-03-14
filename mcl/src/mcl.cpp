#include <cmath>
#include <float.h>
#include <random>

#include "mcl/mcl.h"

static const double STOPPED_THRESHOLD = 1e-10; // Threshold for considering robot stopped (defers updates)
static const double Z_P_01 = 2.3263478740;    // Z score for P(0.01) of Normal(0,1) distribution
static const double F_2_9 = 2 / 9;             // Fraction 2/9

// TBD remove
static const int LAST_ITERATION = 50;

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
  num_particles_min_(mcl_num_particles_min == 0 ? 1 : mcl_num_particles_min),
  num_particles_max_(mcl_num_particles_max),
  kld_eps_(mcl_kld_eps),
  vel_(0.0),
  dist_(num_particles_max_),
  samples_(num_particles_max_),
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
  prob_dist_(0.0, 1.0),
  x_dist_(map_.x_origin, std::nextafter(map_.width * map_.scale + map_.x_origin, DBL_MAX)),
  y_dist_(map_.y_origin, std::nextafter(map_.height * map_.scale + map_.y_origin, DBL_MAX)),
  th_dist_(-M_PI, M_PI)
{
  if (num_particles_min_ > num_particles_max_) {
    throw std::runtime_error("MCL: num_particles_min > num_particles_max\n");
  }
  // Initialize distribution with random samples in the map's free space
  for (size_t i = 0; i < samples_.size(); ++i) {
    samples_[i] = random();
  }
  // Update the distribution with the new samples
  dist_.update(samples_, samples_.size());
}

void MCL::update(const double vel,
                 const double steering_angle,
                 const double dt
                )
{
  if (!stopped(vel)) {
    RecursiveLock lock(dist_mtx_);
    motion_model_.apply(dist_, vel, steering_angle, dt);
  }
}

void MCL::update(const RayScan&& obs)
{
  sensor_model_.update(obs);

  if (!stopped()) {
    RecursiveLock lock(dist_mtx_);
    sensor_model_.apply(dist_, obs);
    dist_.update();

    if (dist_.confidenceAvg() < 1e-4) {
      sample();
    }
    iteration++;
  }
  // TBD remove
  if (iteration > LAST_ITERATION && stopped()) {
    throw std::runtime_error("Finished");
  }
}

void MCL::sample()
{
  RecursiveLock lock(dist_mtx_);
  Particle particle_p;
  double prob_sample_random = 1.0;
  double sample_weight_width = 0.0;
  double sample_weight_sum_target = 0.0;
  double sample_weight_sum = 0.0;
  double num_particles_target = num_particles_min_;
  double chi_sq_term_1 = 0.0;
  double chi_sq_term_2 = 0.0;
  size_t hist_count = 0;
  size_t s = 0;
  size_t p = 0;
  size_t p_prev = (size_t)-1;

  // Clear histogram
  hist_.clear();

  // Calculate the probability to draw random samples
  prob_sample_random = 1.0 - dist_.confidenceRatio();

  // Initialize target and current weight sums
  if (dist_.size() > 0) {
    sample_weight_width = 1.0 / dist_.size();
    sample_weight_sum_target = prob_dist_(rng_.engine()) * sample_weight_width;
    sample_weight_sum = dist_.particle(0).weight_normed_;
  }
  // Generate samples until we exceed the target number of samples, or reach the maximum allowed
  while (   s < num_particles_target
         && s < num_particles_max_
        ) {
    // Draw a particle from the current distribution with probability proportional to its weight
    if (   prob_dist_(rng_.engine()) > prob_sample_random
        && s < dist_.size()
        && p < dist_.size()
       ) {
      // Sum weights until we reach the target
      while (   sample_weight_sum < sample_weight_sum_target
             && p + 1 < dist_.size()
            ) {
        sample_weight_sum += dist_.particle(++p).weight_normed_;
      }
      // Make sure we actually did reach the target weight sum
      if (sample_weight_sum >= sample_weight_sum_target) {
        // If it's not a repeat particle, update its weight and save it in case this particle is sampled again
        if (p != p_prev) {
          particle_p = dist_.particle(p);
          //sensor_model_.update(particle_p); // TBD probably remove
          p_prev = p;
        }
        // Add to sample set and increase target sum
        samples_[s] = particle_p;
        sample_weight_sum_target += sample_weight_width;
      }
    }
    // Exhausted current distribution so generate a new random particle and update its weight
    else {
      samples_[s] = random();
      sensor_model_.apply(samples_[s]);
    }
    // Update histogram with the sampled particle
    hist_.update(samples_[s]);

    // If the histogram occupancy count increased with the sampled particle, update the target number of samples
    if (hist_.count() > hist_count) {
      hist_count = hist_.count();

      if (hist_count > 1) {
        // Wilson-Hilferty transformation of chi-square distribution
        chi_sq_term_1 = (hist_count - 1.0) / (2.0 * kld_eps_);
        chi_sq_term_2 = 1.0 - F_2_9 / (hist_count - 1.0) + Z_P_01 * std::sqrt(F_2_9 / (hist_count - 1.0));
        num_particles_target = chi_sq_term_1 * chi_sq_term_2 * chi_sq_term_2 * chi_sq_term_2;
        num_particles_target = num_particles_target > num_particles_min_?
                               num_particles_target : num_particles_min_;
      }
    }
    ++s;
  }
  // Update distribution with new samples
  dist_.update(samples_, s);

  printf("\n*** Iteration %lu ***\n", iteration + 1);
  printf("Samples used = %lu\n", s);
  printf("Samples target = %lu\n", static_cast<size_t>(num_particles_target));
  printf("Histogram count = %lu\n", hist_count);
  printf("---------------------------------\n");

  return;
}

void MCL::save(const std::string filename,
               const bool sort,
               const bool overwrite
              )
{
  RecursiveLock lock(dist_mtx_);
  ParticleVector particles(dist_.size());

  for (size_t i = 0; i < dist_.size(); ++i) {
    particles[i] = dist_.particle(i);
  }
  if (sort) {
    localize::sort(particles);
  }
  localize::save(particles, filename, overwrite);
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

  // Particle weight is 1.0 until updated by the sensor model
  return particle;
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