#include "mcl/mcl.h"

static const double CAR_SPEED_STOPPED = 1e-3; // Speed below which the car is stopped (defers updates)
static const double KLD_EPS = 0.02;           // KL distance epsilon
static const double Z_P_01 = 2.3263478740;    // Z score for P(0.01) of Normal(0,1) distribution
static const double F_2_9 = 2.0 / 9.0;        // Fraction 2/9

using namespace localize;

// ========== MCL ========== //
MCL::MCL(const unsigned int mcl_num_particles_min,
         const unsigned int mcl_num_particles_max,
         const double mcl_weight_avg_random_sample,
         const double mcl_weight_rel_dev_resample,
         const double car_length,
         const double car_origin_to_sensor_frame_x,
         const double car_origin_to_sensor_frame_y,
         const double car_origin_to_sensor_frame_th,
         const double car_back_center_to_sensor_frame_x,
         const double car_back_center_to_sensor_frame_y,
         const double motion_vel_lin_n1,
         const double motion_vel_lin_n2,
         const double motion_vel_ang_n1,
         const double motion_vel_ang_n2,
         const double motion_th_n1,
         const double motion_th_n2,
         const double motion_vel_ang_bias_scale,
         const float sensor_range_min,
         const float sensor_range_max,
         const float sensor_range_no_obj,
         const float sensor_range_std_dev,
         const float sensor_decay_rate_new_obj,
         const double sensor_weight_no_obj,
         const double sensor_weight_new_obj,
         const double sensor_weight_map_obj,
         const double sensor_weight_rand_effect,
         const double sensor_weight_uncertainty_factor,
         const double sensor_prob_new_obj_reject,
         const unsigned int map_x_size,
         const unsigned int map_y_size,
         const double map_x_origin_world,
         const double map_y_origin_world,
         const double map_th_world,
         const double map_scale_world,
         const std::vector<int8_t>& map_data
        ) :
  num_particles_min_(mcl_num_particles_min),
  weight_avg_random_sample_(mcl_weight_avg_random_sample),
  weight_rel_dev_resample_(mcl_weight_rel_dev_resample),
  car_origin_to_sensor_frame_x_(car_origin_to_sensor_frame_x),
  car_origin_to_sensor_frame_y_(car_origin_to_sensor_frame_y),
  car_origin_to_sensor_frame_th_(car_origin_to_sensor_frame_th),
  car_vel_lin_(0.0),
  map_(map_x_size,
       map_y_size,
       map_x_origin_world,
       map_y_origin_world,
       map_th_world,
       map_scale_world,
       map_data
      ),
  motion_model_(car_length,
                car_back_center_to_sensor_frame_x,
                car_back_center_to_sensor_frame_y,
                motion_vel_lin_n1,
                motion_vel_lin_n2,
                motion_vel_ang_n1,
                motion_vel_ang_n2,
                motion_th_n1,
                motion_th_n2,
                motion_vel_ang_bias_scale,
                map_
               ),
  sensor_model_(sensor_range_min,
                sensor_range_max,
                sensor_range_no_obj,
                sensor_range_std_dev,
                sensor_decay_rate_new_obj,
                sensor_weight_no_obj,
                sensor_weight_new_obj,
                sensor_weight_map_obj,
                sensor_weight_rand_effect,
                sensor_weight_uncertainty_factor,
                sensor_prob_new_obj_reject,
                map_
               ),
  dist_(mcl_num_particles_max, map_),
  samples_(mcl_num_particles_max),
  hist_(map_),
  random_pose_(map_),
  localization_reset_(false),
  prob_(0.0, std::nextafter(1.0, std::numeric_limits<double>::max()))
{
  // Initialize distribution with random samples in the map's free space
  for (size_t i = 0; i < samples_.size(); ++i) {
    samples_[i] = Particle(random_pose_());
  }
  // Populate the distribution with the sample set
  dist_.populate(samples_, samples_.size());
}

void MCL::motionUpdate(const double car_vel_lin,
                       const double car_steer_angle,
                       const double dt
                      )
{
  if (!stopped(car_vel_lin)) {
    RecursiveLock lock(dist_mtx_);
    motion_model_.apply(dist_, car_vel_lin, car_steer_angle, dt);
  }
}

void MCL::sensorUpdate(const RayScan& obs)
{
  // Cache observation before requesting lock
  sensor_model_.update(obs);

  if (!stopped()) {
    RecursiveLock lock(dist_mtx_);
    sensor_model_.apply(dist_);
    sampleUpdate();
  }
}

ParticleVector MCL::estimates()
{
  ParticleVector estimates;
  {
    RecursiveLock lock(dist_mtx_);
    estimates = dist_.estimates();
  }
  // Transform from sensor frame (MCL local frame) to the car origin frame
  double th_car_origin_to_map = 0.0;

  for (size_t i = 0; i < estimates.size(); ++i) {
    th_car_origin_to_map = wrapAngle(estimates[i].th_ + car_origin_to_sensor_frame_th_);
    estimates[i].x_ += (  std::cos(th_car_origin_to_map) * car_origin_to_sensor_frame_x_
                        - std::sin(th_car_origin_to_map) * car_origin_to_sensor_frame_y_
                       );
    estimates[i].y_ += (  std::sin(th_car_origin_to_map) * car_origin_to_sensor_frame_x_
                        + std::cos(th_car_origin_to_map) * car_origin_to_sensor_frame_y_
                       );
    estimates[i].th_ = th_car_origin_to_map;
  }
  return estimates;
}

double MCL::confidence()
{
  RecursiveLock lock(dist_mtx_);

  return dist_.weightAvgFast();
}

void MCL::sampleUpdate()
{
  RecursiveLock lock(dist_mtx_);
  double num_particles_target = num_particles_min_;
  double chi_sq_term_1 = 0.0;
  double chi_sq_term_2 = 0.0;
  size_t hist_count = 0;
  size_t s = 0;

  // If random sampling was performed on the last update, we relocalized the car and can no longer compare the new
  // distribution with any time-smoothed weight average 'history' from the old distribution.
  // To handle this, reset the distribution's weight average history.
  if (localization_reset_) {
    dist_.resetWeightAvgHistory();
    localization_reset_ = false;
  }
  // Random sample probability is based on the short term vs. long term weight average: the worse the short term
  // is compared to the long term, the more random samples are added
  double prob_sample_random = randomSampleRequired() ? 1.0 - dist_.weightAvgRatio() : 0.0;

  if (   prob_sample_random
      || resampleRequired()
     ) {
    // Clear histogram of particles
    hist_.reset();

    // Generate samples until we reach the target or max
    while (   s < samples_.size()
           && s < num_particles_target
          ) {
      // Generate a new random particle and apply the sensor model to update its weight
      if (   prob_sample_random   // Don't generate a new random number if probability is zero
          && prob_(rng_.engine()) < prob_sample_random
         ) {
        samples_[s] = Particle(random_pose_());
        sensor_model_.apply(samples_[s]);
        localization_reset_ = true;
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
  }
}

bool MCL::resampleRequired()
{
  RecursiveLock lock(dist_mtx_);

  return dist_.weightRelStdDev() > weight_rel_dev_resample_;
}

bool MCL::randomSampleRequired()
{
  RecursiveLock lock(dist_mtx_);

  return dist_.weightAvgFast() < weight_avg_random_sample_;
}

bool MCL::stopped()
{
  RecursiveLock lock(car_vel_lin_mtx_);

  return std::abs(car_vel_lin_) < CAR_SPEED_STOPPED;
}

bool MCL::stopped(const double car_vel_lin)
{
  RecursiveLock lock(car_vel_lin_mtx_);

  car_vel_lin_ = car_vel_lin;

  return stopped();
}

void MCL::printStats(const std::string& header) const
{
  printf("%s", header.c_str());
  printf("Weight average [curr] = %.2e\n", dist_.weightAvgCurr());
  printf("Weight average [fast] = %.2e\n", dist_.weightAvgFast());
  printf("Weight average ratio = %.2e\n", dist_.weightAvgRatio());
}
