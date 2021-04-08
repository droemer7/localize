#include "mcl/mcl.h"

static const double WEIGHT_AVG_LOST = 1e-6;       // Average weight below which we assume we are lost (required for random sampling)
static const double WEIGHT_DEV_CONSISTENT = 0.5;  // Weight sigma below which the weights are considered consistent (required for resampling)
static const double SPEED_STOPPED = 1e-6;         // Speed below which the robot is stopped (defers updates)
static const double KLD_EPS = 0.02;               // KL distance epsilon
static const double Z_P_01 = 2.3263478740;        // Z score for P(0.01) of Normal(0,1) distribution
static const double F_2_9 = 2.0 / 9.0;            // Fraction 2/9

using namespace localize;

// ========== MCL ========== //
MCL::MCL(const unsigned int num_particles_min,
         const unsigned int num_particles_max,
         const double car_length,
         const double car_sensor_to_base_frame_x,
         const double car_sensor_to_base_frame_y,
         const double car_sensor_to_base_frame_th,
         const double car_sensor_to_back_frame_x,
         const double car_sensor_to_back_frame_y,
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
  num_particles_min_(num_particles_min),
  vel_lin_(0.0),
  car_sensor_to_base_frame_x_(car_sensor_to_base_frame_x),
  car_sensor_to_base_frame_y_(car_sensor_to_base_frame_y),
  car_sensor_to_base_frame_th_(car_sensor_to_base_frame_th),
  map_(map_width,
       map_height,
       map_x_origin,
       map_y_origin,
       map_th,
       map_scale,
       map_data
      ),
  motion_model_(car_length,
                car_sensor_to_back_frame_x,
                car_sensor_to_back_frame_y
               ),
  sensor_model_(sensor_range_min,
                sensor_range_max,
                sensor_range_no_obj,
                map_
               ),
  dist_(num_particles_max, map_),
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

void MCL::update(const double vel_lin,
                 const double steering_angle,
                 const double dt
                )
{
  if (!stopped(vel_lin)) {
    RecursiveLock lock(dist_mtx_);

    motion_model_.apply(dist_, vel_lin, steering_angle, dt);
  }
}

void MCL::update(const RayScan& obs)
{
  // Cache observation before requesting lock
  sensor_model_.update(obs);

  if (!stopped()) {
    RecursiveLock lock(dist_mtx_);

    sensor_model_.apply(dist_);
    printStats("\n===== Sensor update =====\n");
    update();
  }
}

ParticleVector MCL::estimates()
{
  ParticleVector estimates;
  {
    RecursiveLock lock(dist_mtx_);
    estimates = dist_.estimates();
  }
  // Transform from sensor frame (MCL local frame) to the car base frame
  double x = 0.0;
  double y = 0.0;

  for (size_t i = 0; i < estimates.size(); ++i) {
    x = (  car_sensor_to_base_frame_x_
         + estimates[i].x_ * std::cos(car_sensor_to_base_frame_th_)
         + estimates[i].y_ * std::sin(car_sensor_to_base_frame_th_)
        );
    y = (  car_sensor_to_base_frame_y_
         - estimates[i].x_ * std::sin(car_sensor_to_base_frame_th_)
         + estimates[i].y_ * std::cos(car_sensor_to_base_frame_th_)
        );
    estimates[i].th_ = wrapAngle(estimates[i].th_ + car_sensor_to_base_frame_th_);
  }
  return estimates;
}

Particle MCL::estimate()
{
  Particle estimate;
  {
    RecursiveLock lock(dist_mtx_);
    estimate = dist_.estimates()[0];
  }
  // Transform from sensor frame (MCL local frame) to the car base frame
  double x = (  car_sensor_to_base_frame_x_
              + estimate.x_ * std::cos(car_sensor_to_base_frame_th_)
              + estimate.y_ * std::sin(car_sensor_to_base_frame_th_)
             );
  double y = (  car_sensor_to_base_frame_y_
              - estimate.x_ * std::sin(car_sensor_to_base_frame_th_)
              + estimate.y_ * std::cos(car_sensor_to_base_frame_th_)
             );
  estimate.x_ = x;
  estimate.y_ = y;
  estimate.th_ = wrapAngle(estimate.th_ + car_sensor_to_base_frame_th_);

  return estimate;
}

bool MCL::stopped()
{
  RecursiveLock lock(vel_lin_mtx_);

  return std::abs(vel_lin_) < SPEED_STOPPED;
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
  // improves
  double prob_sample_random = randomSampleRequired() ? 1.0 - dist_.weightAvgRatio() : 0.0;
  bool resample = resampleRequired();

  hist_.reset();

  if (   prob_sample_random
      || resample
     ) {
    // Generate samples until we reach the target or max
    while (   s < samples_.size()
           && s < num_particles_target
          ) {
      // Generate a new random particle and apply the sensor model to update its weight
      if (   prob_sample_random   // Don't generate a new random number if probability is zero
          && prob_(rng_.engine()) < prob_sample_random
         ) {
        samples_[s] = random_sample_();
        sensor_model_.apply(samples_[s]);
      }
      // Draw a particle from the current distribution with probability proportional to its weight
      else {
        samples_[s] = dist_.sample();
      }
      // Update histogram with the sampled particle, and if the count increased, update the target number of samples
      if (hist_.add(samples_[s])) {
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
    printStats("\n===== Sample update =====\n");
    printf("Prob random = %.2f\n", prob_sample_random);
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
  ParticleVector estimates(dist_.estimates());
  ParticleVector particles(dist_.count());

  for (size_t i = 0; i < dist_.count(); ++i) {
    particles[i] = dist_.particle(i);
  }
  if (sort) {
    std::sort(particles.begin(), particles.end(), Greater());
  }
  localize::save(estimates, filename, overwrite);
  localize::save(particles, filename, false);
}

bool MCL::stopped(const double vel_lin)
{
  RecursiveLock lock(vel_lin_mtx_);

  vel_lin_ = vel_lin;

  return stopped();
}

void MCL::printStats(const std::string& header) const
{
  printf("%s", header.c_str());
  printf("Sample size = %lu\n", dist_.count());
  printf("Weight average = %.2e\n", dist_.weightAvg());
  printf("Weight average ratio = %.2f\n", dist_.weightAvgRatio());
}