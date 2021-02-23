#ifndef SENSOR_H
#define SENSOR_H

#include <vector>

#include "mcl/util.h"

#include "includes/RangeLib.h"

namespace localize
{
  struct Ray
  {
    explicit Ray(const float range = 0.0,
                 const float angle = 0.0
                ) :
      range_(range),
      angle_(angle)
    {}

    float range_;
    float angle_;
  };

  // Beam-based probabilistic model for a range sensor
  //
  // The model is comprised of four superimposed probability distributions:
  // 1) A probability for detecting nothing
  // 2) A probability for detecting a new, unmapped object
  // 3) A probability for detecting a known, mapped object
  // 4) A probability for an unexplainable, random result
  // These are evaluated for the range observed vs. the range expected from the
  // map to determine the probability p(ranges[t] | pose[t], map)
  //
  // Ref: Probabilistic Robotics (Thrun 2006)
  class BeamModel
  {
  public:
    // Constructor
    BeamModel(const double range_min,           // Sensor min range in meters
              const double range_max,           // Sensor max range in meters
              const double range_no_obj,        // Sensor range reported when nothing is detected
              const double range_std_dev,       // Sensor range standard deviation
              const double angle_sample_inc,    // Sensor angle increment at which to sample observations (rad / sample)
              const double new_obj_decay_rate,  // Model decay rate for unexpected object probability
              const double weight_no_obj,       // Model weight for no object detected probability
              const double weight_new_obj,      // Model weight for new (unexpected) object probability
              const double weight_map_obj,      // Model weight for map (expected) object probability
              const double weight_rand_effect,  // Model weight for random effect probability
              const double uncertainty_factor,  // Model uncertainty factor - extra noise added to calculation
              const size_t table_size,          // Model table size
              const Map map                     // Map
             );

    // Calculates the probability the observed range occurred due to the
    // sensor failing to detect an object. This may occur due to reflections
    // or an object being too close to the sensor to be detectable.
    //
    // Returns 1 if the observed range is equal to the range returned by the
    // sensor when no object is found, or 0 if not.
    double calcProbNoObj(const float range_obs);

    // Calculates the probability the observed range occured due to a new,
    // unmapped object, such as a person walking by the sensor.
    double calcProbNewObj(const float range_obs,
                          const float range_map
                         );

    // Calculates the probability the observed range occurred due to detecting
    // a mapped object. This probability assumes sensor noise is normally
    // distributed with a mean equal to the range of the nearest mapped
    // object.
    double calcProbMapObj(const float range_obs,
                          const float range_map
                         );

    // Calculates the probability the observed range occurred due to random
    // effects (reflections, interferences, etc).
    //
    // Returns a constant value if the observed range is within bounds of the
    // sensor's min and max range.
    double calcProbRandEffect(const float range_obs);

    // Calculates the overall probability of the observed range given the
    // 'true' range determined from the map
    double calcProb(const float range_obs,
                    const float range_map
                   );

    // Retrieves from the model lookup table the overall probability of the
    // observed range given the 'true' range determined from the map
    double lookupProb(const float range_obs,
                      const float range_map
                     );

    // Converts NaN, negative ranges and any range beyond the configured max
    // to the range reported when nothing is detected by the sensor
    float repairRange(const float range);

    // Generates a subset of ranges sampled from the full observation array
    // using the configured angle sample increment
    std::vector<Ray> sample(const std::vector<Ray>& rays_obs);

    // Applies the sensor model to calculate particle importance weights from
    // p(ranges[t] | pose[t], map)
    // Algorithm 6.1 from Probabilistic Robotics (Thrun 2006, page 158)
    // *** Online calculation - does NOT use lookup table ***
    void applyCalc(const std::vector<Ray>& rays_obs,
                   std::vector<PoseWithWeight>& particles
                  );

    // Applies the sensor model to look up particle importance weights from
    // p(ranges[t] | pose[t], map) using the model lookup table
    // Algorithm 6.1 from Probabilistic Robotics (Thrun 2006, page 158)
    void applyLookup(const std::vector<Ray>& rays_obs,
                     std::vector<PoseWithWeight>& particles
                    );

  private:
    // Precalculates weights given by the model for a discrete set of ranges
    // and loads this into the model lookup table
    // First axis is incremented by ranges observed from the sensor
    // Second axis is incremented by ranges calculated from the map
    void precalc();

    // Debug function to save ray data to file
    void save(const std::vector<Ray>& rays,
              const std::string filename,
              const unsigned int precision = 0,
              const bool overwrite = true
             );

  private:
    // Model parameters
    double range_min_;          // Sensor min range in meters
    double range_max_;          // Sensor max range in meters
    double range_no_obj_;       // Sensor range reported when nothing is detected
    double range_std_dev_;      // Sensor range standard deviation
    double angle_sample_inc_;   // Sensor angle increment at which to sample observations (rad / sample)
    double new_obj_decay_rate_; // Model decay rate for unexpected object probability
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
    std::vector<std::vector<double>> table_;  // Model lookup table
    const size_t table_size_;                 // Model table size
    const double table_inc_;                  // Model table increment (step size)

    ranges::CDDTCast raycaster_;  // Range calculator
  };

} // namespace localize

#endif // SENSOR_H
