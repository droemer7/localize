#include <cmath>
#include <float.h>
#include <random>

#include "mcl/mcl.h"

static const double WEIGHT_AVG_LOST = 1e-08;      // Average weight below which we assume we are lost (required for random sampling)
static const double WEIGHT_DEV_CONSISTENT = 0.5;  // Weight sigma below which the weights are considered consistent (required for resampling)
static const double KLD_EPS = 0.02;               // KL distance epsilon
static const double Z_P_01 = 2.3263478740;        // Z score for P(0.01) of Normal(0,1) distribution
static const double F_2_9 = 2 / 9;                // Fraction 2/9
static const double HIST_POS_RES = 0.10;          // Histogram resolution for x and y position (meters per cell)
static const double HIST_TH_RES = M_PI / 18.0;    // Histogram resolution for heading angle (rad per cell)
static const double SPEED_STOPPED = 1e-10;        // Speed below which the robot is stopped (defers updates)
static const size_t NUM_SENSOR_SCANS_TUNE = 200;  // Number of sensor scans to save for tuning the model
static const int NUM_UPDATES = 10;                // TBD remove

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

ParticleHistogram::ParticleHistogram(const Map& map) :
  x_size_(std::round(map.width * map.scale / HIST_POS_RES)),
  y_size_(std::round(map.height * map.scale / HIST_POS_RES)),
  th_size_(std::round((M_2PI + map.th_origin) / HIST_TH_RES)),
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
  size_t x_i = std::min(std::max(0.0, (particle.x_ - x_origin_) / HIST_POS_RES),
                        static_cast<double>(x_size_ - 1)
                       );
  size_t y_i = std::min(std::max(0.0, (particle.y_ - y_origin_) / HIST_POS_RES),
                        static_cast<double>(y_size_ - 1)
                       );
  size_t th_i = std::min(std::max(0.0, (particle.th_ - th_origin_) / HIST_TH_RES),
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

MCL::MCL(const unsigned int num_particles_min,
         const unsigned int num_particles_max,
         const double car_length,
         const float sensor_range_min,
         const float sensor_range_max,
         const float sensor_range_no_obj,
         const unsigned int map_width,
         const unsigned int map_height,
         const float map_x_origin,
         const float map_y_origin,
         const float map_th,
         const float map_scale,
         const std::vector<int8_t> map_data
        ) :
  update_num_(0),
  num_particles_min_(num_particles_min),
  vel_(0.0),
  map_(map_width,
       map_height,
       map_x_origin,
       map_y_origin,
       map_th,
       map_scale,
       map_data
      ),
  motion_model_(car_length),
  sensor_model_(sensor_range_min,
                sensor_range_max,
                sensor_range_no_obj,
                map_
               ),
  dist_(num_particles_max),
  samples_(num_particles_max),
  hist_(map_),
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

void MCL::update(const RayScan& obs)
{
  // Cache observation before requesting lock
  sensor_model_.update(obs);

  if (!stopped()) {
    printf("\n***** Update %lu *****\n", update_num_ + 1);
    printf("\n===== Sensor model update =====\n");
    RecursiveLock lock(dist_mtx_);
    sensor_model_.apply(dist_);
    update();
    update_num_++;
  }
  // Run tuning
  // TBD find a better way to make this optional
  // Move into another function and have MCLNode call this instead
  // if (sensor_data_.size() < NUM_SENSOR_SCANS_TUNE) {
  //   sensor_data_.push_back(obs);
  // }
  // else {
  //   sensor_model_.tune(sensor_data_, Particle(0.02, 0.0, -0.01));
  //   throw std::runtime_error("Finished");
  // }
  // TBD remove
  if (   stopped()
      && update_num_ >= NUM_UPDATES
     ) {
    throw std::runtime_error("Finished");
  }
}

void MCL::update()
{
  RecursiveLock lock(dist_mtx_);
  double num_particles_target = num_particles_min_;
  double chi_sq_term_1 = 0.0;
  double chi_sq_term_2 = 0.0;
  size_t hist_count = 0;
  size_t s = 0;

  // The probability of random samples is based on the change in average confidence and decreases as the confidence
  // improves.
  double prob_sample_random = randomSampleRequired() ? 1.0 - dist_.weightAvgRatio() : 0.0;
  bool resample = resampleRequired();

  // Clear histogram
  hist_.clear();

  if (   prob_sample_random
      || resample
     ) {
    printf("\n===== Sampling =====\n");
    // Generate samples until we reach the target or max
    while (   s < samples_.size()
           && s < num_particles_target
          ) {
      // Generate a new random particle and apply the sensor model to update its weight
      if (prob_(rng_.engine()) < prob_sample_random) {
        samples_[s] = random_sample_();
        sensor_model_.apply(samples_[s]);
      }
      // Draw a particle from the current distribution with probability proportional to its weight
      else {
        samples_[s] = dist_.sample();
      }
      // Update histogram with the sampled particle, and if the count increased, update the target number of samples
      if (hist_.update(samples_[s])) {
        hist_count = hist_.count();

        if (hist_count > 1) {
          // Wilson-Hilferty transformation of chi-square distribution
          chi_sq_term_1 = (hist_count - 1.0) / (2.0 * KLD_EPS);
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

    printf("Prob(random) = %.2f\n", prob_sample_random);
    printf("Samples used = %lu\n", s);
    printf("Histogram count = %lu\n", hist_count);
    printf("---------------------------------\n");
  }
  return;
}

bool MCL::resampleRequired()
{
  RecursiveLock lock(dist_mtx_);
  return dist_.weightRelativeStdDev() > WEIGHT_DEV_CONSISTENT;
}

bool MCL::randomSampleRequired()
{
  RecursiveLock lock(dist_mtx_);
  return dist_.weightAvg() < WEIGHT_AVG_LOST;
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

  return std::abs(vel_) < SPEED_STOPPED;
}

bool MCL::stopped(const double vel)
{
  RecursiveLock lock(vel_mtx_);
  vel_ = vel;

  return stopped();
}