#include <iomanip>
#include <float.h>
#include <fstream>

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
                     const size_t table_size
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
  table_size_(table_size + 1),
  table(table_size + 1, std::vector<double>(table_size + 1))
{
  load();
}

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
      // Compute range_map using raycast() function
      prob *= (  weight_no_obj_  * probNoObj(ranges_obs[j])
               + weight_new_obj_ * probNewObj(ranges_obs[j],
                                                     ranges_obs[j] // TBD ranges_map_[i]
                                                    )
               + weight_map_obj_ * probMapObj(ranges_obs[j],
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

inline double BeamModel::probNewObj(const double range_obs,
                                    const double range_map
                                   )
{
  if (   range_obs >= range_min_
      && range_obs <= range_map
     )
  {
    return (  new_obj_decay_rate_ * std::exp(-new_obj_decay_rate_ * range_obs)
            / (1 - new_obj_decay_rate_ * std::exp(-new_obj_decay_rate_ * range_map))
           );
  }
  else
  {
    return 0.0;
  }
}

inline double BeamModel::probMapObj(const double range_obs,
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

void BeamModel::save(const std::string filename)
{
  std::ofstream output(filename, std::ofstream::trunc);
  for (std::vector<double> row : table)
  {
    for (double val : row)
    {
      output << std::fixed << std::setprecision(16) << val << ",";
    }
    output << "\n";
  }
}

void BeamModel::load()
{
  double range_obs = 0.0;
  double range_map = 0.0;
  double weighted_prob_no_obj = 0.0;
  double weighted_prob_rand_effect = 0.0;
  double step = range_max_ / table_size_;

  for (size_t i = 0; i < table_size_; ++i)
  {
    range_obs = i * step;
    weighted_prob_no_obj = weight_no_obj_ * probNoObj(range_obs);
    weighted_prob_rand_effect = weight_rand_effect_ * probRandEffect(range_obs);

    for (size_t j = 0; j < table_size_; ++j)
    {
      range_map = j * step;
      table[i][j] = (  (  weighted_prob_no_obj
                        + weight_new_obj_ * probNewObj(range_obs, range_map)
                        + weight_map_obj_ * probMapObj(range_obs, range_map)
                        + weighted_prob_rand_effect
                       )
                     / weights_sum_
                    );
    }
  }
}