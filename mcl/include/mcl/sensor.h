#ifndef SENSOR_H
#define SENSOR_H

#include <vector>

#include "mcl/util.h"

#include "includes/RangeLib.h"

namespace localize
{
  // Beam-based probabilistic model for a range sensor
  //
  // The model is comprised of four superimposed probability distributions:
  // 1) A probability for detecting nothing
  // 2) A probability for detecting a new, unmapped object
  // 3) A probability for detecting a known, mapped object
  // 4) A probability for an unexplainable, random result
  // These are evaluated for the range observed vs. the range expected from the map to determine the probability
  // p(ranges[t] | pose[t], map)
  //
  // Ref: Probabilistic Robotics (Thrun 2006)
  class BeamModel
  {
  public:
    // Constructor
    BeamModel(const float range_min,            // Sensor min range in meters
              const float range_max,            // Sensor max range in meters
              const float range_no_obj,         // Sensor range reported when nothing is detected
              const float range_std_dev,        // Sensor range standard deviation
              const float th_sample_res,        // Sensor angle resolution at which to sample observations (rad per sample)
              const float th_raycast_res,       // Sensor angle resolution for raycast (rad per increment)
              const float new_obj_decay_rate,   // Model decay rate for new (unexpected) object probability
              const double weight_no_obj,       // Model weight for no object detected probability
              const double weight_new_obj,      // Model weight for new (unexpected) object probability
              const double weight_map_obj,      // Model weight for map (expected) object probability
              const double weight_rand_effect,  // Model weight for random effect probability
              const double uncertainty_factor,  // Model uncertainty factor - extra noise added to calculation
              const double table_res,           // Model table resolution (meters per cell)
              const Map& map                    // Map
             );

    // Applies the sensor model to determine particle importance weights from p(ranges[t] | pose[t], map)
    // Algorithm 6.1 from Probabilistic Robotics (Thrun 2006, page 158)
    void apply(Particle& particle,
               const bool calc_enable = false
              );

    void apply(Particle& particle,
               const RayScan& obs,
               const bool calc_enable = false
              );

    void apply(ParticleDistribution& dist,
               const bool calc_enable = false
              );

    void apply(ParticleDistribution& dist,
               const RayScan& obs,
               const bool calc_enable = false
              );

    // Updates the latest saved observation by sampling from the input observation
    void update(const RayScan& obs);

  private:
    // Generate a subset of ranges sampled from the full scan using the preset angle sample increment
    RayVector sample(const RayScan& obs);

    // Convert NaN, negative ranges and any range beyond the configured max to the range reported when nothing is
    // detected by the sensor
    float repair(const float range);

    // Precalculate weights given by the model for a discrete set of ranges and load this into the model lookup table
    // First axis is incremented by ranges observed from the sensor
    // Second axis is incremented by ranges calculated from the map
    void precalcProb();

    // Retrieve from the model lookup table the overall probability of the observed range given the 'true' range
    // determined from the map
    double lookupProb(const float range_obs,
                      const float range_map
                     );

    // Calculate the overall probability of the observed range given the 'true' range determined from the map
    double calcProb(const float range_obs,
                    const float range_map
                   );

    // Calculate the probability the observed range occurred due to the sensor failing to detect an object. This may
    // occur due to reflections or an object being too close to the sensor to be detectable.
    // Returns 1 if the observed range matches the range the sensor reports when no object is found
    double calcProbNoObj(const float range_obs);

    // Calculate the probability the observed range occured due to a new, unmapped object, such as a person walking by
    // the sensor.
    double calcProbNewObj(const float range_obs,
                          const float range_map
                         );

    // Calculate the probability the observed range occurred due to detecting a mapped object. This probability assumes
    // sensor noise is normally distributed with a mean equal to the range of the nearest mapped object.
    double calcProbMapObj(const float range_obs,
                          const float range_map
                         );

    // Calculate the probability the observed range occurred due to random effects (reflections, interferences, etc).
    // Returns a constant value if the observed range is within bounds of the sensor's min and max range.
    double calcProbRandEffect(const float range_obs);

  private:
    // Model parameters
    float range_min_;           // Sensor min range in meters
    float range_max_;           // Sensor max range in meters
    float range_no_obj_;        // Sensor range reported when nothing is detected
    float range_std_dev_;       // Sensor range standard deviation
    float th_sample_res_;       // Sensor angle resolution at which to sample observations (rad per sample)
    float new_obj_decay_rate_;  // Model decay rate for new (unexpected) object probability
    double weight_no_obj_;      // Model weight for no object detected probability
    double weight_new_obj_;     // Model weight for new (unexpected) object probability
    double weight_map_obj_;     // Model weight for map (expected) object probability
    double weight_rand_effect_; // Model weight for random effect probability
    double weights_sum_;        // Model weights sum
    double uncertainty_factor_; // Model uncertainty factor - extra noise added to calculation

    // Model lookup table
    // First axis is incremented by ranges observed from the sensor
    // Second axis is incremented by ranges calculated from the map
    // Values are weights that would be given by the model given ranges
    // observed by the sensor and ranges calculated by raycasting on the map
    const double table_res_;                  // Model table resolution (meters per cell)
    const size_t table_size_;                 // Model table size
    std::vector<std::vector<double>> table_;  // Model lookup table

    const size_t rays_obs_sample_size_; // Number of downsampled observations
    RayVector rays_obs_sample_;         // Downsampled observations

    ranges::CDDTCast raycaster_;  // Range calculator

    RNG rng_;  // Random number engine
    std::uniform_real_distribution<double> th_sample_dist_;  // Distribution of initial sample angle offsets
  };

} // namespace localize

#endif // SENSOR_H
