#ifndef AMCL_H
#define AMCL_H

#include "amcl/common.h"
#include "amcl/distribution.h"
#include "amcl/histogram.h"
#include "amcl/motion.h"
#include "amcl/sensor.h"

namespace localize
{
  // Adaptive Monte-Carlo Localization
  // Localization technique to approximate the possible distribution of poses by a set of random samples drawn from its
  // probability distribution. Localization is calculated in the sensor frame and is converted to the base frame when
  // the latest estimate is requested by the user.
  class AMCL
  {
  public:
    // Constructors
    AMCL(const unsigned int amcl_num_particles_min,       // AMCL minimum number of particles
         const unsigned int amcl_num_particles_max,       // AMCL maximum number of particles
         const double amcl_weight_avg_random_sample,      // AMCL weight average below which random sampling is enabled
         const double amcl_weight_rel_dev_resample,       // AMCL weight relative standard deviation above which resampling is performed
         const double car_length,                         // Car length
         const double car_origin_to_sensor_frame_x,       // Car origin to sensor frame x translation
         const double car_origin_to_sensor_frame_y,       // Car origin to sensor frame y translation
         const double car_origin_to_sensor_frame_th,      // Car origin to sensor frame rotation
         const double car_back_center_to_sensor_frame_x,  // Car back center to sensor frame x translation
         const double car_back_center_to_sensor_frame_y,  // Car back center to sensor frame y translation
         const double motion_vel_lin_n1,                  // Motion model linear velocity noise coefficient 1
         const double motion_vel_lin_n2,                  // Motion model linear velocity noise coefficient 2
         const double motion_vel_ang_n1,                  // Motion model angular velocity noise coefficient 1
         const double motion_vel_ang_n2,                  // Motion model angular velocity noise coefficient 2
         const double motion_th_n1,                       // Motion model final rotation noise coefficient 1
         const double motion_th_n2,                       // Motion model final rotation noise coefficient 2
         const float sensor_range_min,                    // Sensor min range in meters
         const float sensor_range_max,                    // Sensor max range in meters
         const float sensor_range_no_obj,                 // Sensor range reported when nothing is detected
         const float sensor_range_std_dev,                // Sensor model range measurement standard deviation
         const float sensor_decay_rate_new_obj,           // Sensor model decay rate for new / unexpected object probability
         const double sensor_weight_no_obj,               // Sensor model weight for no object detected probability
         const double sensor_weight_new_obj,              // Sensor model weight for new / unexpected object probability
         const double sensor_weight_map_obj,              // Sensor model weight for mapped / expected object probability
         const double sensor_weight_rand_effect,          // Sensor model weight for random effect probability
         const double sensor_weight_uncertainty_factor,   // Sensor model weight uncertainty factor (extra noise added to final weight)
         const double sensor_prob_new_obj_reject,         // Sensor model probability above which a ray is rejected for representing a new / unexpected object
         const unsigned int map_x_size,                   // Map length of x axis (width) (pixels)
         const unsigned int map_y_size,                   // Map length of y axis (height) (pixels)
         const double map_x_origin_world,                 // Map x translation of origin (cell 0,0) relative to world frame (meters)
         const double map_y_origin_world,                 // Map y translation of origin (cell 0,0) relative to world frame (meters)
         const double map_th_world,                       // Map angle relative to world frame (read)
         const double map_scale_world,                    // Map scale relative to world frame (meters per pixel)
         const std::vector<int8_t>& map_data              // Map occupancy data in 1D row-major order, -1: Unknown, 0: Free, 100: Occupied
        );

    // Apply the motion model to update particle locations using p(x[t] | u[t], x[t-1])
    void motionUpdate(const double car_vel_lin,
                      const double car_steer_angle,
                      const double dt
                     );

    // Apply the sensor model to update particle weights using p(obs[t] | pose[t], map)
    void sensorUpdate(const RayScan& obs);

    // Return the top particle estimates in the sensor's frame - lower indexes are better estimates
    ParticleVector estimates();

    // Return the average confidence of the distribution (not the average confidence of the best estimate)
    // This value is smoothed over a few time steps to reduce the effect of transient noise
    double confidence();

  private:
    // Update particle distribution by performing resampling (based on particle weights) and/or random sampling
    void sampleUpdate();

    // Return true if distribution should be resampled
    bool resampleRequired();

    // Return true if random samples should be added to the distribution
    bool randomSampleRequired();

    // Return true if the car velocity is within the stopped threshold based on the last saved value
    bool stopped();

    // Update the car velocity with the input value and return true if it is within the stopped threshold
    bool stopped(const double car_vel_lin);

    // Print statistics about the distribution
    void printStats(const std::string& header = "") const;

  private:
    RecursiveMutex dist_mtx_;         // Particle distribution mutex
    RecursiveMutex car_vel_lin_mtx_;  // Linear velocity mutex

    const size_t num_particles_min_;              // Minimum number of particles
    const double weight_avg_random_sample_;       // Weight average below which random sampling is enabled
    const double weight_rel_dev_resample_;        // Weight relative standard deviation above which resampling is performed
    const double car_origin_to_sensor_frame_x_;   // Car origin to sensor frame x translation
    const double car_origin_to_sensor_frame_y_;   // Car origin to sensor frame y translation
    const double car_origin_to_sensor_frame_th_;  // Car origin to sensor frame rotation
    double car_vel_lin_;                          // Linear velocity of the car

    const Map map_;           // Map
    VelModel motion_model_;   // Motion model
    BeamModel sensor_model_;  // Sensor model

    ParticleDistribution dist_;     // Particle distribution in the sensor frame
    ParticleVector samples_;        // Sampled particles (temporary storage)
    PoseOccupancyHistogram hist_;   // Histogram for estimating probability distribution complexity
    PoseRandomSampler random_pose_; // Random pose sampler, generates samples in free space based on the map
    bool localization_reset_;       // Indicates localization was reset by random sampling in the last update

    RNG rng_; // Random number generator
    std::uniform_real_distribution<double> prob_; // Distribution to generate a random probabilities (reals in [0, 1])
  };

} // namespace localize

#endif // AMCL_H
