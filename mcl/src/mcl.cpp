#include <cmath>
#include <float.h>
#include <random>

#include "mcl/mcl.h"

static const double Z_P_99 = 2.3263478740;    // Z score for P(0.99) of Normal(0,1) distribution
static const double F_2_9 = 2 / 9;            // Fraction 2/9
static const double HIST_OCC_PROB_MIN = 1e-4; // Probability required to consider a histogram cell occupied

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
  iteration(0),
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
  hist_(map_.width * map_.scale,
        map_.height * map_.scale,
        M_2PI,
        mcl_hist_pos_res,
        mcl_hist_pos_res,
        mcl_hist_th_res,
        mcl_hist_occ_weight_min
       ),
  num_particles_min_(mcl_num_particles_min),
  num_particles_max_(mcl_num_particles_max),
  kld_eps_(mcl_kld_eps),
  vel_(0.0),
  sample_dist_(0.0, 1.0),
  x_dist_(map_.x, std::nextafter(map_.width * map_.scale + map_.x, DBL_MAX)),
  y_dist_(map_.y, std::nextafter(map_.height * map_.scale + map_.y, DBL_MAX)),
  th_dist_(-M_PI, M_PI)
{
  if (num_particles_min_ > num_particles_max_) {
    throw std::runtime_error("MCL: num_particles_min > num_particles_max\n");
  }
  // Reserve enough space up front since these can be large
  particles_.reserve(num_particles_max_);
  samples_.reserve(num_particles_max_);
  reset();
}

void MCL::motionUpdate(const double vel,
                       const double steering_angle,
                       const double dt
                      )
{
  {
    std::lock_guard<std::mutex> lock(vel_mtx_);
    vel_ = vel;
  }
  std::lock_guard<std::mutex> lock(particles_mtx_);

  // Only do motion update if moving
  if (!stopped()) {
    motion_model_.update(vel,
                         steering_angle,
                         dt,
                         particles_
                        );
  }
}

void MCL::sensorUpdate(const std::vector<Ray>& rays)
{
  std::lock_guard<std::mutex> lock(particles_mtx_);

  // Only do sensor update if moving
  if (!stopped()) {
    sensor_model_.update(rays, particles_);
    sample();
  }
  // TBD remove
  /*
  if (iteration == 0) {
    save("particles_presample.csv", false);
    sample();
  }
  if (iteration++ == 1) {
    save("particles_postsample.csv");
    throw std::runtime_error("Saved resampled particles");
  }
  */
}

void MCL::save(const std::string filename,
               const bool sort,
               const unsigned int precision,
               const bool overwrite
              )
{
  if (sort) {
    localize::sort(particles_);
  }
  localize::save(particles_,
                 filename,
                 precision,
                 overwrite
                );
}

// TBD can we just use particles vector and overwrite? is samples vector really necessary
void MCL::sample()
{
  size_t num_particles = particles_.size();
  double sample_width = 0.0;
  double sum_target = 0.0;
  double sum_curr = 0.0;
  double weight_sum = 0.0;
  double num_particles_target = num_particles_max_;
  double chi_sq_term_1 = 0.0;
  double chi_sq_term_2 = 0.0;
  size_t s = 0;
  size_t p = 0;
  size_t k = 0;

  // Initialize histogram
  hist_.reset();

  // Initialize target and current weight sums
  if (num_particles > 0) {
    sample_width = 1.0 / num_particles;
    sum_target = sample_dist_(rng_.engine()) * sample_width;
    sum_curr = particles_[0].weight_;
  }
  // Generate samples until we exceed both the minimum and target number of
  // samples, or reach the maximum number allowed
  while (   (   s < num_particles_target
             || s < num_particles_min_
            )
         && s < num_particles_max_
        ) {
    // Sample from the current distribution until the sampled set
    // size is equal to the current distribution size
    if (s < num_particles) {
      // Sum weights until we reach the target sum
      while (sum_curr < sum_target) {
        sum_curr += particles_[++p].weight_;
      }
      // Add to sample set and increase target sum
      samples_[s] = particles_[p];
      sum_target += sample_width;
    }
    // Generate a new random particle in free space
    else {
      samples_[s] = gen();
    }
    // Update weight sum
    weight_sum += samples_[s].weight_;

    // Update histogram, returns true if number of occupied bins increased
    k = hist_.update(samples_[s++]) ? k + 1 : k;

    // Compute target number of samples using the Wilson-Hilferty
    // transformation of the chi-square distribution
    if (k > 1) {
      chi_sq_term_1 = (k - 1.0) / (2.0 * kld_eps_);
      chi_sq_term_2 = 1.0 - F_2_9 / (k - 1.0) + Z_P_99 * std::sqrt(F_2_9 / (k - 1.0));
      num_particles_target = chi_sq_term_1 * chi_sq_term_2 * chi_sq_term_2 * chi_sq_term_2;
    }
  }
  // Resize to the actual number of samples used and normalize weights
  samples_.resize(s);
  double normalizer = 1 / weight_sum;
  for (size_t i = 0; i < samples_.size(); ++i) {
    samples_[i].weight_ *= normalizer;
  }
  particles_ = samples_;

  return;
}

PoseWithWeight MCL::gen()
{
  PoseWithWeight particle;
  bool occupied = true;

  // Regenerate x & y until free space is found
  while (occupied) {
    particle.x_ = x_dist_(rng_.engine());
    particle.y_ = y_dist_(rng_.engine());
    occupied = map_.isOccupied(particle.x_, particle.y_);
  }
  // Any theta is allowed
  particle.th_ = th_dist_(rng_.engine());

  // Particle weight is initialized to 1.0 in constructor
  return particle;
}

// TBD replace using gen()
void MCL::reset()
{
  bool occupied = true;

  for (PoseWithWeight & particle : particles_) {
    occupied = true;

    // Regenerate x & y until free space is found
    while (occupied) {
      particle.x_ = x_dist_(rng_.engine());
      particle.y_ = y_dist_(rng_.engine());
      occupied = map_.isOccupied(particle.x_, particle.y_);
    }
    // Any theta is allowed
    particle.th_ = th_dist_(rng_.engine());
    particle.weight_ = 0.0;
  }
  particles_[0].x_ = 0.0;
  particles_[0].y_ = 0.0;
  particles_[0].th_ = -0.2;
}

bool MCL::stopped()
{
  std::lock_guard<std::mutex> lock(vel_mtx_);

  return std::abs(vel_) < FLT_EPSILON;
}

// TBD remove
/*
std::vector<PoseWithWeight> MCL::sample(size_t num_samples)
{
  num_samples = std::min(num_samples, particles_.size());
  std::vector<PoseWithWeight> samples(num_samples);

  if (   num_samples > 0
      && particles_.size() > 0
     ) {
    double sample_width = 1.0 / num_samples;
    double sum_target = sample_dist_(rng_.engine()) * sample_width;
    double sum_curr = particles_[0].weight_;
    size_t s = 0;
    size_t p = 0;

    // Generate the number of samples desired
    while (s < num_samples) {
      // Sum particle weights until we reach the target sum
      while (sum_curr < sum_target) {
        sum_curr += particles_[++p].weight_;
      }
      // Add to sample set, reset weight to 1.0, and increase target sum
      samples[s] = particles_[p];
      samples[s].weight_ = 1.0;
      ++s;
      sum_target += sample_width;
    }
  }
  return samples;
}
*/