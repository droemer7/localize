#include <cmath>

#include "mcl/sensor.h"

using namespace localize;

BeamModel::BeamModel(const double range_min,
                     const double range_max,
                     const double range_no_obj,
                     const double range_std_dev,
                     const double new_obj_decay_rate,
                     const double weight_no_obj,
                     const double weight_new_obj,
                     const double weight_map_obj,
                     const double weight_rand_effect,
                     const unsigned int table_size,
                     const Map map
                    ) :
  range_min_(range_min),
  range_max_(range_max),
  range_no_obj_(range_no_obj),
  range_std_dev_(range_std_dev),
  new_obj_decay_rate_(new_obj_decay_rate),
  weight_no_obj_(weight_no_obj),
  weight_new_obj_(weight_new_obj),
  weight_map_obj_(weight_map_obj),
  weight_rand_effect_(weight_rand_effect),
  weights_sum_(  weight_no_obj_
               + weight_new_obj_
               + weight_map_obj_
               + weight_rand_effect_
              ),
  table_(table_size + 1, std::vector<double>(table_size + 1)),
  table_size_(table_size + 1),
  table_inc_(range_max / table_size),
  raycaster_(map,
             range_max / map.world_scale,
             656  // TBD th_discretization
            )
{
  // TBD check parameters are valid here?
  precompute();
}

void BeamModel::apply(const std::vector<float>& ranges_obs,
                      const float ranges_angle_inc,
                      const std::vector<Pose>& particles,
                      std::vector<double>& weights
                     )
{
  float range_map = 0.0;
  float range_obs = 0.0;
  int range_obs_table_idx = 0;
  int range_map_table_idx = 0;
  double prob = 1.0;
  for (size_t i = 0; i < particles.size(); ++i) {
    prob = 1.0;
    /*
    printf("MCL: particles[%d] = (%f, %f, %f)\n",
           static_cast<int>(i),
           particles[i].x_, particles[i].y_, particles[i].th_
          );
    */

    for (size_t j = 0; j < ranges_obs.size(); ++j) {
      // Compute range from the map
      range_map = raycaster_.calc_range(-0.035,// particles[i].x_,
                                        0.0,   // particles[i].y_,
                                        0.0    // particles[i].th_ + ranges_angle_inc * j
                                       );

      // Make sure ranges are valid numbers in the expected range
      range_obs = convertToValidRange(ranges_obs[j]);
      range_map = convertToValidRange(range_map);

      // Calculate the index to lookup in the sensor model table
      range_obs_table_idx = ranges_obs[j] / table_inc_;
      range_map_table_idx = range_map / table_inc_;
      range_obs_table_idx = std::min(std::max(0, range_obs_table_idx), static_cast<int>(table_size_) - 1);
      range_map_table_idx = std::min(std::max(0, range_map_table_idx), static_cast<int>(table_size_) - 1);

      /*
      printf("MCL: Angle index (j) = %lu\n", j);
      printf("MCL: range_obs = %f\n", range_obs);
      printf("MCL: range_map = %f\n", range_map);
      printf("MCL: range_obs_table_idx = %d\n", range_obs_table_idx);
      printf("MCL: range_map_table_idx = %d\n", range_map_table_idx);
      */

      // Update weight
      prob *= table_[range_obs_table_idx][range_map_table_idx];
    }
    weights[i] = prob;
  }
  printf("MCL: Sensor model lookup\n");
}

void BeamModel::eval(const std::vector<float>& ranges_obs,
                     const std::vector<Pose>& particles,
                     std::vector<double>& weights
                    )
{
  double prob = 1.0;

  for (size_t i = 0; i < particles.size(); ++i) {
    prob = 1.0;

    for (size_t j = 0; j < ranges_obs.size(); ++j) {
      // Compute range_map using raycast() function
      prob *= (  weight_no_obj_  * probNoObj(ranges_obs[j])
               + weight_new_obj_ * probNewObj(ranges_obs[j], ranges_obs[j]) // TBD range_map
               + weight_map_obj_ * probMapObj(ranges_obs[j], ranges_obs[j]) // TBD range_map
               + weight_rand_effect_ * probRandEffect(ranges_obs[j])
              );
    }
    weights[i] = prob;
  }
}

void BeamModel::eval(const std::vector<float>& ranges_obs,
                     const Pose particle,
                     double weight
                    )
{
  double prob = 1.0;

  for (size_t i = 0; i < ranges_obs.size(); ++i) {
      // Compute range_map using raycast() function
      prob *= (  weight_no_obj_  * probNoObj(ranges_obs[i])
               + weight_new_obj_ * probNewObj(ranges_obs[i], ranges_obs[i]) // TBD range_map
               + weight_map_obj_ * probMapObj(ranges_obs[i], ranges_obs[i]) // TBD range_map
               + weight_rand_effect_ * probRandEffect(ranges_obs[i])
              );
  }
  weight = prob;
}

double BeamModel::probNoObj(const double range_obs)
{
  return equal(range_obs, range_no_obj_);
}

double BeamModel::probNewObj(const double range_obs,
                             const double range_map
                            )
{
  if (   !equal(range_obs, range_no_obj_)
      && range_obs <= range_map
     ) {
    return (  new_obj_decay_rate_ * std::exp(-new_obj_decay_rate_ * range_obs)
            / (1 - new_obj_decay_rate_ * std::exp(-new_obj_decay_rate_ * range_map))
           );
  }
  else {
    return 0.0;
  }
}

double BeamModel::probMapObj(const double range_obs,
                             const double range_map
                            )
{
  if (!equal(range_obs, range_no_obj_)) {
    return std::exp(  -(range_obs - range_map) * (range_obs - range_map)
                    / (2 * range_std_dev_ * range_std_dev_)
                   );
  }
  else {
    return 0.0;
  }
}

double BeamModel::probRandEffect(const double range_obs)
{
  if (!equal(range_obs, range_no_obj_)) {
    return 1.0 / range_max_;
  }
  else {
    return 0.0;
  }
}

double BeamModel::convertToValidRange(double range)
{
  return (   range > range_max_
          || std::signbit(range)
          || std::isnan(range)
         ) ?
         range_no_obj_ : range;
}

void BeamModel::precompute()
{
  double range_obs = 0.0;
  double range_map = 0.0;
  double weighted_prob_no_obj = 0.0;
  double weighted_prob_rand_effect = 0.0;

  for (size_t i = 0; i < table_size_; ++i) {
    range_obs = table_inc_ * i;
    weighted_prob_no_obj = weight_no_obj_ * probNoObj(range_obs);
    weighted_prob_rand_effect = weight_rand_effect_ * probRandEffect(range_obs);

    for (size_t j = 0; j < table_size_; ++j) {
      range_map = table_inc_ * j;
      table_[i][j] = (  (  weighted_prob_no_obj
                         + weight_new_obj_ * probNewObj(range_obs, range_map)
                         + weight_map_obj_ * probMapObj(range_obs, range_map)
                         + weighted_prob_rand_effect
                        )
                      / weights_sum_
                     );
    }
  }
}