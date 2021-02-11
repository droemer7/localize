#include <float.h>

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
                     const double weight_rand_effect
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
              )
{}

void BeamModel::precompute()
{/* TBD */}

void BeamModel::apply(const std::vector<float>& ranges_obs,
                      const std::vector<Particle>& particles,
                      // TBD const OMap& map,
                      std::vector<double>& weights
                     )
{
  double prob = 0.0;
  for (size_t i = 0; i < particles.size(); ++i)
  {
    prob = 1.0;
    for (size_t j = 0; j < ranges_obs.size(); ++j)
    {
      // Compute range_map as raycast(ranges_obs, )
      prob *= (  weight_no_obj_  * probNoObj(ranges_obs[j])
               + weight_new_obj_ * probUnexpectedObj(ranges_obs[j],
                                                     ranges_obs[j] // TBD ranges_map_[i]
                                                    )
               + weight_map_obj_ * probExpectedObj(ranges_obs[j],
                                                   ranges_obs[j] // TBD ranges_map_[i]
                                                  )
               + weight_rand_effect_ * probRandEffect(ranges_obs[j])
              );
    }
    weights[i] = prob;
  }
}

inline double BeamModel::probNoObj(const double range_obs)
{
  return std::abs(range_obs - range_no_obj_) <= FLT_EPSILON;
}

inline double BeamModel::probUnexpectedObj(const double range_obs,
                                           const double range_map
                                          )
{
  return (  new_obj_decay_rate_ * std::exp(-new_obj_decay_rate_ * range_obs)
          / (1 - new_obj_decay_rate_ * std::exp(-new_obj_decay_rate_ * range_map))
         );
}

inline double BeamModel::probExpectedObj(const double range_obs,
                                         const double range_map
                                        )
{
  return std::exp(  -(range_obs - range_map) * (range_obs - range_map)
                  / (2 * range_std_dev_ * range_std_dev_)
                 );
}

inline double BeamModel::probRandEffect(const double range_obs)
{
  if (   range_obs >= range_min_
      && range_obs <= range_max_
     )
  {
    return 1.0 / range_max_;
  }
  else
  {
    return 0.0;
  }
}