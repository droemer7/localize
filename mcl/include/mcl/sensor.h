#ifndef SENSOR_H
#define SENSOR_H

#include "mcl/distribution.h"
#include "mcl/common.h"

#include "includes/RangeLib.h"

namespace localize
{
  typedef std::vector<std::vector<double>> WeightTable;

  // Beam-based probabilistic model for a range sensor
  //
  // The model is comprised of four superimposed probability distributions:
  // 1) A probability for detecting nothing
  // 2) A probability for detecting a new, unmapped object
  // 3) A probability for detecting a known, mapped object
  // 4) A probability for an unexplainable, random result
  // These are evaluated for the range observed vs. the range expected from the map to determine the probability
  // p(ranges[t] | pose[t], map)
  class BeamModel
  {
  public:
    // Constructor
    BeamModel(const float range_min,                  // Sensor min range in meters
              const float range_max,                  // Sensor max range in meters
              const float range_no_obj,               // Sensor range reported when nothing is detected
              const float range_std_dev,              // Model range measurement standard deviation
              const float decay_rate_new_obj,         // Model decay rate for new / unexpected object probability
              const double weight_no_obj,             // Model weight for no object detected probability
              const double weight_new_obj,            // Model weight for new / unexpected object probability
              const double weight_map_obj,            // Model weight for mapped / expected object probability
              const double weight_rand_effect,        // Model weight for random effect probability
              const double weight_uncertainty_factor, // Model weight uncertainty factor (extra noise added to final weight)
              const double prob_new_obj_reject,       // Model probability above which a ray is rejected for representing a new / unexpected object
              const Map& map                          // Map
             );

    // Apply the sensor model to determine particle importance weights from p(ranges[t] | pose[t], map)
    void apply(Particle& particle, const bool calc_enable = false);

    void apply(ParticleDistribution& dist, const bool calc_enable = false);

    void apply(ParticleDistribution& dist,
               const RayScan& obs,
               const bool calc_enable = false
              );

    // Update the latest saved observation by sampling from the input observation
    void update(const RayScan& obs);

  private:
    // Generate a subset of ranges sampled from the full scan using the preset angle sample increment
    RaySampleVector sample(const RayScan& obs, const size_t sample_count);

    // Removes the contribution of outliers to the particle weights
    // Outlier weights are those derived from range observations which were likely detecting new / unexpected objects
    void removeOutliers(ParticleDistribution& dist);

    // Convert NaN, negative ranges and any range beyond the configured max to the range reported when nothing is
    // detected by the sensor
    float repairRange(const float range) const;

    // Converts the range value to a corresponding index in the table
    size_t tableIndex(const float range) const;

    // Retrieve from the model lookup table the weighted probability of the observed range being the result of a new
    // (unexpected) object, given the 'true' range determined from the map
    double lookupWeightedProbNewObj(const float range_obs, const float range_map) const;

    // Retrieve from the model lookup table the weighted mixed probability of the observed range given the 'true' range
    // determined from the map
    double lookupWeightedProb(const float range_obs, const float range_map) const;

    // Calculate the weighted mixed probability of the observed range given the 'true' range determined from the map
    double calcWeightedProb(const float range_obs, const float range_map) const;

    // Calculate the probability the observed range occurred due to the sensor failing to detect an object
    double calcProbNoObj(const float range_obs) const;

    // Calculate the probability the observed range occured due to a new, unmapped object
    double calcProbNewObj(const float range_obs, const float range_map) const;

    // Calculate the probability the observed range occurred due to detecting a mapped / expected object
    double calcProbMapObj(const float range_obs, const float range_map) const;

    // Calculate the probability the observed range occurred due to random effects
    double calcProbRandEffect(const float range_obs) const;

    // Calculate the weighted probability the observed range occurred due to the sensor failing to detect an object
    double calcWeightedProbNoObj(const float range_obs) const;

    // Calculate the weighted probability the observed range occured due to a new, unmapped object
    double calcWeightedProbNewObj(const float range_obs, const float range_map) const;

    // Calculate the weighted probability the observed range occurred due to detecting a mapped / expected object
    double calcWeightedProbMapObj(const float range_obs, const float range_map) const;

    // Calculate the weighted probability the observed range occurred due to random effects
    double calcWeightedProbRandEffect(const float range_obs) const;

    // Precalculate weighted probabilities a discrete set of ranges and load this into the model lookup table
    // First axis is incremented by ranges observed from the sensor
    // Second axis is incremented by ranges calculated from the map
    void precalcWeightedProbs();

    // Prints info corresponding to the rejected range
    void printRejectedRange(const Ray & ray, const double prob) const;

  private:
    // Model parameters
    const float range_min_;                   // Sensor min range in meters
    const float range_max_;                   // Sensor max range in meters
    const float range_no_obj_;                // Sensor range reported when nothing is detected
    const float range_std_dev_;               // Model range measurement standard deviation
    const float decay_rate_new_obj_;          // Model decay rate for new / unexpected object probability
    const double weights_sum_;                // Model weights sum (for normalization)
    const double weight_no_obj_;              // Model weight for no object detected probability
    const double weight_new_obj_;             // Model weight for new / unexpected object probability
    const double weight_map_obj_;             // Model weight for mapped / expected object probability
    const double weight_rand_effect_;         // Model weight for random effect probability
    const double weight_uncertainty_factor_;  // Model weight uncertainty factor (extra noise added to final weight)
    const double prob_new_obj_reject_;        // Model probability above which a ray is rejected for representing a new / unexpected object

    // Model lookup tables
    // First axis is incremented by ranges observed from the sensor
    // Second axis is incremented by ranges calculated from the map
    // Values are weights that would be given by the model given ranges
    // observed by the sensor and ranges calculated by raycasting on the map
    const double weight_table_res_;     // Model table resolution (meters per cell)
    const size_t weight_table_size_;    // Model table size
    WeightTable weight_table_new_obj_;  // Model lookup table, new / unexpected object probability
    WeightTable weight_table_;          // Model lookup table, all weight components combined

    RaySampleVector rays_obs_sample_; // Downsampled observations
    ranges::CDDTCast raycaster_;      // Range calculator

    RNG rng_;  // Random number engine
    std::uniform_real_distribution<float> th_sample_dist_;  // Distribution of initial sample angle offsets
  };

} // namespace localize

#endif // SENSOR_H
