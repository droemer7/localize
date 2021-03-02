#include <cmath>

#include "mcl/sensor.h"

using namespace localize;

BeamModel::BeamModel(const double range_min,
                     const double range_max,
                     const double range_no_obj,
                     const double range_std_dev,
                     const double th_sample_res,
                     const double th_raycast_res,
                     const double new_obj_decay_rate,
                     const double weight_no_obj,
                     const double weight_new_obj,
                     const double weight_map_obj,
                     const double weight_rand_effect,
                     const double uncertainty_factor,
                     const double table_res,
                     const Map map
                    ) :
  range_min_(range_min),
  range_max_(range_max),
  range_no_obj_(range_no_obj),
  range_std_dev_(range_std_dev),
  th_sample_res_(th_sample_res),
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
  uncertainty_factor_(uncertainty_factor),
  table_res_(table_res),
  table_size_(range_max / table_res + 1),
  table_(table_size_, std::vector<double>(table_size_)),
  raycaster_(map,
             range_max / map.scale,
             std::round(M_2PI / th_raycast_res)
            ),
  sample_dist_(0.0, th_sample_res_)
{
  if (range_min_ > range_max_) {
    double prev_range_max = range_max_;
    range_max_ = range_min_;
    range_min_ = prev_range_max;
    printf("BeamModel: Warning - range min > range max, swapping.\n");
  }
  if (th_sample_res_ > M_2PI)
  {
    printf("BeamModel: Warning - angle sample resolution greater than 2pi, using 2pi. \
            Units are in radians, not degrees.\n"
          );
    th_sample_res_ = M_2PI;
  }
  if (uncertainty_factor_ < 1.0)
  {
    printf("BeamModel: Warning - model uncertainty factor less than 1, using 1 (no uncertainty).\n");
    uncertainty_factor_ = 1.0;
  }
  if (equal(uncertainty_factor_, 0.0))
  {
    printf("BeamModel: Warning - model uncertainty factor negative or too small (%f), \
           using 1 (no additional uncertainty).                                       \
           Model uncertainty should be in the range (0, 1].\n",
           uncertainty_factor_
          );
    uncertainty_factor_ = 1.0;
  }
  if (   range_min_ < 0.0
      || range_max_ < 0.0
      || range_no_obj_ < 0.0
      || range_std_dev_ < 0.0
      || th_sample_res_ < 0.0
      || new_obj_decay_rate_ < 0.0
      || weight_no_obj_ < 0.0
      || weight_new_obj_ < 0.0
      || weight_map_obj_ < 0.0
      || weight_rand_effect_ < 0.0
      || uncertainty_factor_ < 0.0
     ) {
    throw std::runtime_error("BeamModel: Check configuration - parameters must be >= zero");
  }
  precalcProb();
}

void BeamModel::apply(const std::vector<Ray>& rays_obs,
                      std::vector<PoseWithWeight>& particles,
                      const bool calc_enable
                     )
{
  double weight = 1.0;
  double weight_sum = 0.0;
  float range_obs = 0.0;
  float range_map = 0.0;

  // Downsample the full ray list according to the angle sample increment
  std::vector<Ray> rays_obs_sample = sample(rays_obs);

  for (size_t i = 0; i < particles.size(); ++i) {
    weight = 1.0;

    for (size_t j = 0; j < rays_obs_sample.size(); ++j) {
      // Compute range from the map
      range_map = raycaster_.calc_range(particles[i].x_,
                                        particles[i].y_,
                                        particles[i].th_ + rays_obs_sample[j].th_
                                       );
      // Make sure ranges are valid and convert if necessary
      // (NaNs, negative values, etc)
      range_obs = repair(rays_obs_sample[j].range_);
      range_map = repair(range_map);

      // Update partial weight with this measurement's probability
      weight *= calc_enable ? calcProb(range_obs, range_map) :
                              lookupProb(range_obs, range_map);
    }
    // Update full weight, applying overall model uncertainty
    weight = std::pow(weight, uncertainty_factor_);
    weight_sum += weight;
    particles[i].weight_ = weight;
  }
  // Normalize weights with the sum
  normalize(particles, weight_sum);
}

std::vector<Ray> BeamModel::sample(const std::vector<Ray>& rays_obs)
{
  size_t rays_obs_size = rays_obs.size();
  size_t rays_obs_sample_size = static_cast<size_t>(std::round(M_2PI / th_sample_res_));
  std::vector<Ray> rays_obs_sample(rays_obs_sample_size);

  // More than one observation
  if (   rays_obs_size > 1
      && rays_obs_sample_size > 0
     ) {
    // Generate a random offset for the sampled set to start from
    double th_obs_min = rays_obs[0].th_;
    double th_obs_inc = std::abs(rays_obs[1].th_ - th_obs_min);
    size_t o = sample_dist_(rng_.engine()) / th_obs_inc;
    size_t s = 0;

    // Iterate through both arrays until we've either gone through all of the
    // observations, or we've collected the desired amount of samples
    while (   o < rays_obs_size
           && s < rays_obs_sample_size
          ) {
      rays_obs_sample[s++] = rays_obs[o];
      o += static_cast<size_t>(th_sample_res_ / th_obs_inc);
    }
    rays_obs_sample.resize(s);
  }
  // Only one observation
  else if (   rays_obs_size == 1
           && rays_obs_sample_size > 0
          ) {
    rays_obs_sample[0] = rays_obs[0];
    rays_obs_sample.resize(1);
  }
  return rays_obs_sample;
}

float BeamModel::repair(float range)
{
  return (   range > range_max_
          || std::signbit(range)
          || std::isnan(range)
         ) ?
         range_no_obj_ : range;
}

void BeamModel::normalize(std::vector<PoseWithWeight>& particles,
                          const double weight_sum
                         )
{
  if (weight_sum > 0.0) {
    double normalizer = 1 / weight_sum;
    for (size_t i = 0; i < particles.size(); ++i) {
      particles[i].weight_ *= normalizer;
    }
  }
}

void BeamModel::precalcProb()
{
  float range_obs = 0.0;
  float range_map = 0.0;

  for (size_t i = 0; i < table_size_; ++i) {
    range_obs = table_res_ * i;

    for (size_t j = 0; j < table_size_; ++j) {
      range_map = table_res_ * j;
      table_[i][j] = calcProb(range_obs, range_map);
    }
  }
}

double BeamModel::lookupProb(const float range_obs,
                             const float range_map
                            )
{
  // Calculate sensor model table indexes for lookup
  size_t range_obs_table_i = std::min(std::max(0.0, range_obs / table_res_),
                                      static_cast<double>(table_size_ - 1));
  size_t range_map_table_i = std::min(std::max(0.0, range_map / table_res_),
                                      static_cast<double>(table_size_ - 1));

  // Lookup observed range probability
  return table_[range_obs_table_i][range_map_table_i];
}

double BeamModel::calcProb(const float range_obs,
                           const float range_map
                          )
{
  return (  (  weight_no_obj_ * calcProbNoObj(range_obs)
             + weight_new_obj_ * calcProbNewObj(range_obs, range_map)
             + weight_map_obj_ * calcProbMapObj(range_obs, range_map)
             + weight_rand_effect_ * calcProbRandEffect(range_obs)
            )
          / weights_sum_
         );
}

double BeamModel::calcProbNoObj(const float range_obs)
{
  return equal(range_obs, range_no_obj_);
}

double BeamModel::calcProbNewObj(const float range_obs,
                                 const float range_map
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

double BeamModel::calcProbMapObj(const float range_obs,
                                 const float range_map
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

double BeamModel::calcProbRandEffect(const float range_obs)
{
  if (!equal(range_obs, range_no_obj_)) {
    return 1.0 / range_max_;
  }
  else {
    return 0.0;
  }
}
