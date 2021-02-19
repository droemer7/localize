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
              const double range_std_dev,       // Sensor standard deviation
              const double new_obj_decay_rate,  // Model decay rate for unexpected object probability
              const double weight_no_obj,       // Model weight for no object detected probability
              const double weight_new_obj,      // Model weight for new (unexpected) object probability
              const double weight_map_obj,      // Model weight for map (expected) object probability
              const double weight_rand_effect,  // Model weight for random effect probability
              const unsigned int table_size,    // Model table size
              const Map map                     // Map
             );

    // Applies the sensor model to calculate particle importance weights from
    // p(ranges[t] | pose[t], map)
    // Algorithm 6.1 from Probabilistic Robotics (Thrun 2006, page 158)
    void apply(const std::vector<float>& ranges_obs,
               const float ranges_angle_inc,
               const std::vector<Pose>& particles,
               std::vector<double>& weights
              );

    // Evaluates the sensor model to calculate particle importance weights from
    // p(ranges[t] | pose[t], map)
    // *** Does NOT use lookup table ***
    void eval(const std::vector<float>& ranges_obs,
              const std::vector<Pose>& particles,
              std::vector<double>& weights
             );

    // Evaluates the sensor model to calculate a single particle importance
    // weight from p(ranges[t] | pose[t], map)
    // *** Does NOT use lookup table ***
    void eval(const std::vector<float>& ranges_obs,
              const Pose particle,
              double weight
             );

    // Calculates the probability the observed range occurred due to the
    // sensor failing to detect an object. This may occur due to reflections
    // or an object being too close to the sensor to be detectable.
    //
    // Returns 1 if the observed range is equal to the range returned by the
    // sensor when no object is found, or 0 if not.
    double probNoObj(const double range_obs);

    // Calculates the probability the observed range occured due to a new,
    // unmapped object, such as a person walking by the sensor.
    double probNewObj(const double range_obs,
                      const double range_map
                     );

    // Calculates the probability the observed range occurred due to detecting
    // a mapped object. This probability assumes sensor noise is normally
    // distributed with a mean equal to the range of the nearest mapped
    // object.
    double probMapObj(const double range_obs,
                      const double range_map
                     );

    // Calculates the probability the observed range occurred due to random
    // effects (reflections, interferences, etc).
    //
    // Returns a constant value if the observed range is within bounds of the
    // sensor's min and max range.
    double probRandEffect(const double range_obs);

    // Converts NaN, negative ranges and any range beyond the configured max
    // to the range reported when nothing is detected by the sensor
    double convertToValidRange(const double range);

  private:
    // Precomputes weights given by the model for a discrete set of ranges and
    // loads this into the model lookup table
    // First axis is incremented by ranges observed from the sensor
    // Second axis is incremented by ranges calculated from the map
    void precompute();

  private:
    // Model parameters
    double range_min_;          // Sensor min range in meters
    double range_max_;          // Sensor max range in meters
    double range_no_obj_;       // Sensor range reported when nothing is detected
    double range_std_dev_;      // Sensor standard deviation
    double new_obj_decay_rate_; // Model decay rate for unexpected object probability
    double weight_no_obj_;      // Model weight for no object detected probability
    double weight_new_obj_;     // Model weight for new (unexpected) object probability
    double weight_map_obj_;     // Model weight for map (expected) object probability
    double weight_rand_effect_; // Model weight for random effect probability
    double weights_sum_;        // Model weights sum

    // Discretized model lookup table
    // First axis is incremented by ranges observed from the sensor
    // Second axis is incremented by ranges calculated from the map
    // Values are weights that would be given by the model given ranges
    // observed by the sensor and ranges calculated by raycasting on the map
    std::vector<std::vector<double>> table_;  // Discretized model lookup table
    const unsigned int table_size_;           // Model table size
    const double table_inc_;                  // Model table increment (step size)

    ranges::CDDTCast raycaster_;  // Range calculator
  };

} // namespace localize

#endif // SENSOR_H
