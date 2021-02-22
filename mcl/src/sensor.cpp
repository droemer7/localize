#include <cmath>

#include "mcl/sensor.h"

static const size_t ZERO = 0;

using namespace localize;

BeamModel::BeamModel(const double range_min,
                     const double range_max,
                     const double range_no_obj,
                     const double range_std_dev,
                     const double angle_sample_inc,
                     const double new_obj_decay_rate,
                     const double weight_no_obj,
                     const double weight_new_obj,
                     const double weight_map_obj,
                     const double weight_rand_effect,
                     const double uncertainty_factor,
                     const size_t table_size,
                     const Map map
                    ) :
  range_min_(range_min),
  range_max_(range_max),
  range_no_obj_(range_no_obj),
  range_std_dev_(range_std_dev),
  angle_sample_inc_(angle_sample_inc),
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
  table_(table_size + 1, std::vector<double>(table_size + 1)),
  table_size_(table_size + 1),
  table_inc_(range_max / table_size),
  raycaster_(map,
             range_max / map.world_scale,
             656 // TBD th_discretization
            )
{
  if (range_min_ >= range_max_) {
    double prev_range_max = range_max_;
    range_max_ = range_min_;
    range_min_ = prev_range_max;
  }
  if (angle_sample_inc_ > M_2PI)
  {
    printf("BeamModel: Warning - angle sample increment greater than 2pi, using 2pi. \
            Units are in radians, not degrees.\n"
          );
    angle_sample_inc_ = M_2PI;
  }
  if (uncertainty_factor_ > 1.0)
  {
    printf("BeamModel: Warning - model uncertainty factor greater than 1, using 1 (no uncertainty). \
           Model uncertainty should be in the range (0, 1].\n"
          );
    uncertainty_factor_ = 1.0;
  }
  if (equal(uncertainty_factor_, 0.0))
  {
    printf("BeamModel: Warning - model uncertainty factor negative or too small (%f), using 1 (no uncertainty). \
           Model uncertainty should be in the range (0, 1].\n",
           uncertainty_factor_
          );
    uncertainty_factor_ = 1.0;
  }
  if (   range_min_ <= 0.0
      || range_max_ <= 0.0
      || range_no_obj_ <= 0.0
      || range_std_dev_ <= 0.0
      || angle_sample_inc_ <= 0.0
      || new_obj_decay_rate_ <= 0.0
      || weight_no_obj_ <= 0.0
      || weight_new_obj_ <= 0.0
      || weight_map_obj_ <= 0.0
      || weight_rand_effect_ <= 0.0
      || uncertainty_factor_ <= 0.0
     ) {
    throw std::invalid_argument("BeamModel: Check configuration - parameters must be greater than zero");
  }
  precalc();
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

double BeamModel::lookupProb(const float range_obs,
                             const float range_map
                            )
{
  // Calculate sensor model table indexes for lookup
  size_t range_obs_table_i = 0;
  size_t range_map_table_i = 0;
  range_obs_table_i = range_obs / table_inc_;
  range_map_table_i = range_map / table_inc_;
  range_obs_table_i = std::min(std::max(ZERO, range_obs_table_i), table_size_ - 1);
  range_map_table_i = std::min(std::max(ZERO, range_map_table_i), table_size_ - 1);

  // Lookup observed range probability
  return table_[range_obs_table_i][range_map_table_i];
}

float BeamModel::repairRange(float range)
{
  return (   range > range_max_
          || std::signbit(range)
          || std::isnan(range)
         ) ?
         range_no_obj_ : range;
}

std::vector<Ray> BeamModel::sample(const std::vector<Ray>& rays_obs)
{
  std::vector<Ray> rays_obs_sample;
  size_t rays_obs_size = rays_obs.size();
  size_t rays_obs_sample_size = static_cast<size_t>(std::round(M_2PI / angle_sample_inc_));

  if (rays_obs_size > 1) {
    size_t o = 0;
    size_t s = 0;
    double angle_obs_inc = (rays_obs.end()->angle_ - rays_obs.begin()->angle_) / (rays_obs_size - 1);

    while (   o < rays_obs_size
           && s < rays_obs_sample_size
          ) {
      rays_obs_sample.push_back(rays_obs[o]);
      ++s;
      o = static_cast<size_t>(angle_sample_inc_ * s / angle_obs_inc);
    }
  }
  else if (rays_obs_size == 1) {
    rays_obs_sample.push_back(rays_obs[0]);
  }
  return rays_obs_sample;
}

void BeamModel::applyCalc(const std::vector<Ray>& rays_obs,
                          std::vector<PoseWithWeight>& particles
                         )
{
  double weight = 1.0;
  float range_obs = 0.0;
  float range_map = 0.0;

  for (size_t i = 0; i < particles.size(); ++i) {
    weight = 1.0;

    for (size_t j = 0; j < rays_obs.size(); ++j) {
      // Compute range from the map
      range_map = raycaster_.calc_range(particles[i].x_,
                                        particles[i].y_,
                                        particles[i].th_ + rays_obs[j].angle_
                                       );
      // Make sure ranges are valid and convert if necessary
      // (NaNs, negative values, etc)
      range_obs = repairRange(rays_obs[j].range_);
      range_map = repairRange(range_map);

      // Update partial weight with this measurement's probability
      weight *= calcProb(rays_obs[j].range_, range_map);
    }
    // Update full weight, applying overall model uncertainty
    particles[i].weight_ = std::pow(weight, uncertainty_factor_);
  }
}

void BeamModel::applyLookup(const std::vector<Ray>& rays_obs,
                            std::vector<PoseWithWeight>& particles
                           )
{
  size_t iterations = 1; //particles.size();  // TBD remove

  double weight = 1.0;
  float range_obs = 0.0;
  float range_map = 0.0;

  for (size_t i = 0; i < iterations; ++i) {
    weight = 1.0;
    /*
    printf("MCL: particles[%d] = (%f, %f, %f)\n",
           static_cast<int>(i),
           particles[i].x_, particles[i].y_, particles[i].th_
          );
    */
    for (size_t j = 0; j < rays_obs.size(); ++j) {
      // Compute range from the map
      range_map = raycaster_.calc_range(particles[i].x_,
                                        particles[i].y_,
                                        particles[i].th_ + rays_obs[j].angle_
                                       );
      // Make sure ranges are valid and convert if necessary
      // (NaNs, negative values, etc)
      range_obs = repairRange(rays_obs[j].range_);
      range_map = repairRange(range_map);

      // Update partial weight with this measurement's probability
      weight *= lookupProb(range_obs, range_map);
      /*
      printf("MCL: ray index (j) = %lu\n", j);
      printf("MCL: range_obs = %f\n", range_obs);
      printf("MCL: range_map = %f\n", range_map);
      printf("MCL: range_obs_table_i = %d\n", range_obs_table_i);
      printf("MCL: range_map_table_i = %d\n", range_map_table_i);
      printf("MCL: partial weight = %.20f\n", weight);
      */
    }
    // Update full weight, applying overall model uncertainty
    // printf("MCL: weight = %.20f\n", weight);
    particles[i].weight_ = std::pow(weight, uncertainty_factor_);
  }
}

void BeamModel::precalc()
{
  float range_obs = 0.0;
  float range_map = 0.0;

  for (size_t i = 0; i < table_size_; ++i) {
    range_obs = table_inc_ * i;

    for (size_t j = 0; j < table_size_; ++j) {
      range_map = table_inc_ * j;
      table_[i][j] = calcProb(range_obs, range_map);
    }
  }
}