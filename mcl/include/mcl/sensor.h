#ifndef SENSOR_H
#define SENSOR_H

#include <vector>

#include "mcl/util.h"

namespace localize
{
  class BeamModel
  {
  public:
    // Constructor
    BeamModel(const double range_min,
              const double range_max,
              const double range_no_obj,
              const double range_std_dev,
              const double new_obj_decay_rate,
              const double weight_no_obj,
              const double weight_new_obj,
              const double weight_map_obj,
              const double weight_rand_effect,
              const size_t table_size = 1000
             );

    // Applies the sensor model to calculate p(ranges[t] | pose[t], map)
    // Algorithm 6.1 from Probabilistic Robotics (Thrun 2006, page 158)
    void apply(const std::vector<float>& ranges_obs,
               const std::vector<Particle>& particles,
               // TBD const OMap& map,
               std::vector<double>& weights
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

    // Saves the sensor model lookup table to CSV
    void save(const std::string filename);

  private:
    // Loads the model lookup table by applying the model to a discretized set
    // of ranges
    // First axis is indexed in increments of ranges observed from the sensor
    // Second axis is indexed in increments of ranges calculated from the map
    void load();

  private:
    // Model parameters
    double range_min_;          // Sensor max range in meters
    double range_max_;          // Sensor max range in meters
    double range_no_obj_;       // Range reported by the sensor when nothing is detected
    double range_std_dev_;      // Sensor standard deviation
    double new_obj_decay_rate_; // Decay rate for unexpected object probability
    double weight_no_obj_;      // Sensor weight for no object detected probability
    double weight_new_obj_;     // Sensor weight for new (unexpected) object probability
    double weight_map_obj_;     // Sensor weight for map (expected) object probability
    double weight_rand_effect_; // Sensor weight for random effect probability
    double weights_sum_;        // Sensor weights sum
    const size_t table_size_;   // Model table size

    // Discretized model lookup table
    // First axis is indexed in increments of ranges observed from the sensor
    // Second axis is indexed in increments of ranges calculated from the map
    // The weight that would be calculated by the model can be looked from this
    // table by passing the range observed and range calculated from the map
    // as indexes; e.g. model_table[<range observed>, <range calculated>]
    std::vector<std::vector<double>> table;

  }; // class BeamModel

} // namespace localize

#endif // SENSOR_H
