#include <cmath>
#include <float.h>
#include <random>

#include "mcl/mcl.h"

static const double STOPPED_THRESHOLD = 1e-10;  // Threshold for considering robot stopped (defers updates)
static const double Z_P_01 = 2.3263478740;      // Z score for P(0.01) of Normal(0,1) distribution
static const double F_2_9 = 2 / 9;              // Fraction 2/9

// TBD remove
static const int NUM_UPDATES = 1;

using namespace localize;

ParticleRandomSampler::ParticleRandomSampler(const Map& map) :
  map_(map),
  x_dist_(map_.x_origin, std::nextafter(map_.width * map_.scale + map_.x_origin, std::numeric_limits<double>::max())),
  y_dist_(map_.y_origin, std::nextafter(map_.height * map_.scale + map_.y_origin, std::numeric_limits<double>::max())),
  th_dist_(-M_PI, M_PI)
{}

Particle ParticleRandomSampler::operator()()
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

  // Particle weight is 0.0 until updated by the sensor model
  return particle;
}

ParticleHistogram::ParticleHistogram(const double x_res,
                                     const double y_res,
                                     const double th_res,
                                     const Map& map
                                    ) :
  x_res_(x_res),
  y_res_(y_res),
  th_res_(th_res),
  x_size_(std::round(map.width * map.scale / x_res_)),
  y_size_(std::round(map.height * map.scale / y_res_)),
  th_size_(std::round((M_2PI + map.th_origin) / th_res)),
  x_origin_(map.x_origin),
  y_origin_(map.y_origin),
  th_origin_(map.th_origin),
  hist_(x_size_ * y_size_ * th_size_, false),
  count_(0)
{}

bool ParticleHistogram::update(const Particle& particle)
{
  bool count_inc = false;

  // Calculate index
  size_t x_i = std::min(std::max(0.0, (particle.x_ - x_origin_) / x_res_),
                        static_cast<double>(x_size_ - 1)
                       );
  size_t y_i = std::min(std::max(0.0, (particle.y_ - y_origin_) / y_res_),
                        static_cast<double>(y_size_ - 1)
                       );
  size_t th_i = std::min(std::max(0.0, (particle.th_ - th_origin_) / th_res_),
                         static_cast<double>(th_size_ - 1)
                        );
  // Update histogram
  if (!cell(x_i, y_i, th_i)) {
    cell(x_i, y_i, th_i) = true;
    count_inc = true;
    ++count_;
  }
  return count_inc;
}

size_t ParticleHistogram::count() const
{
  return count_;
}

void ParticleHistogram::clear()
{
  if (count_ > 0) {
    std::fill(hist_.begin(), hist_.end(), false);
    count_ = 0;
  }
}

std::vector<bool>::reference ParticleHistogram::cell(const size_t x_i,
                                                     const size_t y_i,
                                                     const size_t th_i
                                                    )
{
  return hist_[x_i * y_size_ * th_size_ + y_i * th_size_ + th_i];
}

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
  update_num_(0),
  num_particles_min_(mcl_num_particles_min == 0 ? 1 : mcl_num_particles_min),
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
  dist_(mcl_num_particles_max),
  samples_(mcl_num_particles_max),
  hist_(mcl_hist_pos_res,
        mcl_hist_pos_res,
        mcl_hist_th_res,
        map_
       ),
  random_sample_(map_),
  prob_(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()))
{
  // Initialize distribution with random samples in the map's free space
  for (size_t i = 0; i < samples_.size(); ++i) {
    samples_[i] = random_sample_();
  }
  // Copy the new samples to the distribution
  dist_.copy(samples_, samples_.size());
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

  // TBD restore
  //if (!stopped()) {
    printf("\n***** Update %lu *****\n", update_num_ + 1);
    printf("\n===== Sensor model update =====\n");
    RecursiveLock lock(dist_mtx_);
    sensor_model_.apply(dist_);

    // Only sample if the average weight (confidence) is too low
    if (dist_.weightAvg() < 1e-3) {
      sample();
    }
    update_num_++;
  //}
  // TBD remove
  if (   stopped()
      && update_num_ >= NUM_UPDATES
     ) {
    throw std::runtime_error("Finished");
  }
}

void MCL::sample()
{
  printf("\n===== Sampling =====\n");
  RecursiveLock lock(dist_mtx_);
  double prob_sample_random = 1.0;
  double num_particles_target = num_particles_min_;
  double chi_sq_term_1 = 0.0;
  double chi_sq_term_2 = 0.0;
  size_t hist_count = 0;
  size_t s = 0;

  // Clear histogram
  hist_.clear();

  // Calculate the probability to draw random samples
  prob_sample_random = 1.0 - dist_.weightAvgRatio();

  // Generate samples until we reach the target or max
  while (   s < samples_.size()
         && s < num_particles_target
        ) {
    // Draw a particle from the current distribution with probability proportional to its weight
    if (prob_(rng_.engine()) > prob_sample_random) {
      samples_[s] = dist_.sample();
    }
    // Generate a new random particle and apply the sensor model to update its weight
    else {
      samples_[s] = random_sample_();
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
  // Update distribution with new sample set
  dist_.update(samples_, s);

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
  ParticleVector particles(dist_.count());

  for (size_t i = 0; i < dist_.count(); ++i) {
    particles[i] = dist_.particle(i);
  }
  if (sort) {
    localize::sort(particles);
  }
  localize::save(particles, filename, overwrite);
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