#ifndef MCL_H
#define MCL_H

#include "mcl/common.h"
#include "mcl/distribution.h"
#include "mcl/histogram.h"
#include "mcl/motion.h"
#include "mcl/sensor.h"

namespace localize
{
  // Monte-Carlo Localization
  // Localization technique to approximate the possible distribution of poses
  // by a set of random samples drawn from its probability distribution.
  //
  // Note: MCL local frame is the sensor frame.
  class MCL
  {
  public:
    // Constructors
    MCL(const unsigned int num_particles_min,     // Minimum number of particles
        const unsigned int num_particles_max,     // Maximum number of particles
        const double car_length,                  // Car length
        const double car_base_to_sensor_frame_x,  // Car base to sensor frame x translation
        const double car_base_to_sensor_frame_y,  // Car base to sensor frame y translation
        const double car_base_to_sensor_frame_th, // Car base to sensor frame rotation
        const double car_back_to_sensor_frame_x,  // Car back (midpoint between back wheels) to sensor frame x translation
        const double car_back_to_sensor_frame_y,  // Car back (midpoint between back wheels) to sensor frame y translation
        const float sensor_range_min,             // Sensor min range in meters
        const float sensor_range_max,             // Sensor max range in meters
        const float sensor_range_no_obj,          // Sensor range reported when nothing is detected
        const unsigned int map_x_size,            // Map length of x axis (width) (pixels)
        const unsigned int map_y_size,            // Map length of y axis (height) (pixels)
        const double map_x_origin_world,          // Map x translation of origin (cell 0,0) relative to world frame (meters)
        const double map_y_origin_world,          // Map y translation of origin (cell 0,0) relative to world frame (meters)
        const double map_th_world,                // Map angle relative to world frame (read)
        const double map_scale_world,             // Map scale relative to world frame (meters per pixel)
        const std::vector<int8_t>& map_data       // Map occupancy data in 1D row-major order, -1: Unknown, 0: Free, 100: Occupied
       );

    // Apply the motion model to update particle locations using p(x[t] | u[t], x[t-1])
    void update(const double vel,
                const double steering_angle,
                const double dt
               );

    // Apply the sensor model to update particle weights using p(obs[t] | pose[t], map)
    void update(const RayScan& obs);

    // Return the top particle estimates in the sensor's frame - lower indexes are better estimates
    ParticleVector estimates();

    // Return the best particle estimate in the sensor's frame
    Particle estimate();

    // Indicates if the car velocity is within the stopped threshold based on the last saved value
    bool stopped();

    // Save particle distribution to file in CSV format
    void save(const std::string filename,
              const bool sort = true,
              const bool overwrite = true
             );

  private:
    // Update particle distribution by performing resampling (based on particle weights) and/or random sampling
    // No changes are made if distribution is ok
    // Source: KLD-Sampling: Adaptive Particle Filters (Fox 2001)
    void update();

    // Return true if distribution should be resampled
    bool resampleRequired();

    // Return true if random samples should be added to the distribution
    bool randomSampleRequired();

    // Updates the robot velocity with the input value and returns true if it is within the stopped threshold
    bool stopped(const double vel);

    // Print statistics about the distribution for debugging
    void printStats(const std::string& header) const;

  private:
    RecursiveMutex dist_mtx_;     // Particle distribution mutex
    RecursiveMutex vel_lin_mtx_;  // Linear velocity mutex

    const size_t num_particles_min_;      // Minimum number of particles
    double vel_lin_;                      // Linear velocity of the car
    double car_base_to_sensor_frame_x_;   // Car base to sensor frame x translation
    double car_base_to_sensor_frame_y_;   // Car base to sensor frame y translation
    double car_base_to_sensor_frame_th_;  // Car base to sensor frame rotation

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

#endif // MCL_H
