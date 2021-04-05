#include "mcl/mcl.h"

// TBD restore to 1e-8
static const double WEIGHT_AVG_LOST = 1e-8;       // Average weight below which we assume we are lost (required for random sampling)
static const double WEIGHT_DEV_CONSISTENT = 1.0;  // Weight sigma below which the weights are considered consistent (required for resampling)
static const double SPEED_STOPPED = 1e-10;        // Speed below which the robot is stopped (defers updates)
static const double KLD_EPS = 0.02;               // KL distance epsilon
static const double Z_P_01 = 2.3263478740;        // Z score for P(0.01) of Normal(0,1) distribution
static const double F_2_9 = 2.0 / 9.0;            // Fraction 2/9
static const size_t NUM_SENSOR_SCANS_TUNE = 200;  // Number of sensor scans to save for tuning the model

using namespace localize;

// ========== MCL ========== //
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
    RecursiveLock lock(dist_mtx_);

    sensor_model_.apply(dist_);
    update();
  }
}

ParticleVector MCL::estimates()
{
  RecursiveLock lock(dist_mtx_);

  return dist_.estimates();
}

Particle MCL::estimate()
{
  return estimates()[0];
}

bool MCL::stopped()
{
  RecursiveLock lock(vel_mtx_);

  return std::abs(vel_) < SPEED_STOPPED;
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

  // Reset histogram
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
    printf("\n===== Sample update =====\n");
    dist_.update(samples_, s);

    printf("Prob(random) = %.2f\n", prob_sample_random);
    printf("Samples used = %lu\n", s);
    printf("Sample histogram count = %lu\n", hist_count);
    printf("---------------------------------\n");
  }
  dist_.estimates(); // TBD remove
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

bool MCL::stopped(const double vel)
{
  RecursiveLock lock(vel_mtx_);

  vel_ = vel;

  return stopped();
}
// ========== End MCL ========== //