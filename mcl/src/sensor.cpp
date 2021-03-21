#include <algorithm>
#include <cmath>

#include "mcl/sensor.h"

const float RANGE_EPSILON = 1e-5;

using namespace localize;

BeamModel::BeamModel(const float range_min,
                     const float range_max,
                     const float range_no_obj,
                     const float range_std_dev,
                     const float new_obj_decay_rate,
                     const double weight_no_obj,
                     const double weight_new_obj,
                     const double weight_map_obj,
                     const double weight_rand_effect,
                     const double uncertainty_factor,
                     const unsigned int th_sample_count,
                     const unsigned int th_raycast_count,
                     const double table_res,
                     const Map& map
                    ) :
  range_min_(range_min),
  range_max_(range_max),
  range_no_obj_(range_no_obj),
  range_std_dev_(range_std_dev),
  new_obj_decay_rate_(new_obj_decay_rate),
  weights_sum_(  weight_no_obj
               + weight_new_obj
               + weight_map_obj
               + weight_rand_effect
              ),
  weight_no_obj_(weight_no_obj / weights_sum_),
  weight_new_obj_(weight_new_obj / weights_sum_),
  weight_map_obj_(weight_map_obj / weights_sum_),
  weight_rand_effect_(weight_rand_effect / weights_sum_),
  uncertainty_factor_(uncertainty_factor),
  table_res_(table_res),
  table_size_(range_max / table_res + 1),
  weight_table_new_obj_(table_size_, std::vector<double>(table_size_)),
  weight_table_(table_size_, std::vector<double>(table_size_)),
  rays_obs_sample_(th_sample_count),
  raycaster_(map,
             range_max / map.scale,
             th_raycast_count
            ),
  th_sample_dist_(0.0, M_2PI / th_sample_count)
{
  if (uncertainty_factor_ < 1.0)
  {
    printf("BeamModel: Warning - model uncertainty factor less than 1, using 1 (none)\n");
    uncertainty_factor_ = 1.0;
  }
  // Precalculate the sensor model and create tables
  precalcProb();
}

void BeamModel::apply(Particle& particle,
                      const bool calc_enable
                     )
{
  double weight_partial_new_obj = 0.0;
  double weight_partial = 0.0;
  double weight = 1.0;
  float range_obs = 0.0;
  float range_map = 0.0;

  for (size_t i = 0; i < rays_obs_sample_.size(); ++i) {
    // Compute range from the map
    range_map = raycaster_.calc_range(particle.x_,
                                      particle.y_,
                                      particle.th_ + rays_obs_sample_[i].th_
                                     );
    // Make sure ranges are valid (NaNs, negative values, etc)
    range_obs = repair(rays_obs_sample_[i].range_);
    range_map = repair(range_map);

    // Calculate measurement probabilities for this particle
    weight_partial_new_obj = calc_enable ? calcProbNewObj(range_obs, range_map) :
                                           lookupProbNewObj(range_obs, range_map);
    weight_partial = calc_enable ? calcProb(range_obs, range_map) :
                                   lookupProb(range_obs, range_map);

    // Update weight sums for this sampled ray
    rays_obs_sample_[i].weight_new_obj_sum_ += weight_partial_new_obj;
    rays_obs_sample_[i].weight_sum_ += weight_partial;

    // Save partial weight to particle in case we determine later its an outlier and need to remove its effect
    particle.weights_[i] = weight_partial;
    weight *= weight_partial;
  }
  // Update full weight, applying overall model uncertainty
  particle.weight_ = std::pow(weight, uncertainty_factor_);
  return;
}

void BeamModel::apply(Particle& particle,
                      const RayScan& obs,
                      const bool calc_enable
                     )
{
  // Sample and update from the observation
  update(obs);

  // Update particle weight
  apply(particle, calc_enable);

  return;
}

void BeamModel::apply(ParticleDistribution& dist,
                      const bool calc_enable
                     )
{
  // Calculate particle weights
  for (size_t i = 0; i < dist.count(); ++i) {
    apply(dist.particle(i));
  }
  // Remove the contribution of outliers to the particle weights
  removeOutliers(dist);

  // Recalculate weight statistics
  dist.update();

  return;
}

void BeamModel::apply(ParticleDistribution& dist,
                      const RayScan& obs,
                      const bool calc_enable
                     )
{
  // Sample and update the observation
  update(obs);

  // Calculate particle weights
  apply(dist, calc_enable);

  return;
}

void BeamModel::update(const RayScan& obs)
{
  rays_obs_sample_ = sample(obs);
}

RaySampleVector BeamModel::sample(const RayScan& obs)
{
  RaySampleVector rays_obs_sample(rays_obs_sample_.size());

  // More than one observation
  if (   obs.rays_.size() > 1
      && rays_obs_sample.size() > 0
     ) {
    // Generate a random offset for the sampled set to start from
    size_t o_step_size = (M_2PI / rays_obs_sample.size()) / obs.th_inc_;
    size_t o = th_sample_dist_(rng_.engine()) / obs.th_inc_;
    size_t s = 0;

    // Iterate through both arrays selecting the desired amount of samples
    while (   o < obs.rays_.size()
           && s < rays_obs_sample.size()
          ) {
      rays_obs_sample[s++] = obs.rays_[o];
      o += o_step_size;
    }
    rays_obs_sample.resize(s);
  }
  // Only one observation
  else if (   obs.rays_.size() == 1
           && rays_obs_sample.size() > 0
          ) {
    rays_obs_sample[0] = obs.rays_[0];
    rays_obs_sample.resize(1);
  }
  save(rays_obs_sample, "rays_sample.csv");  // TBD remove
  return rays_obs_sample;
}

float BeamModel::repair(float range)
{
  return (   range >= range_max_
          || std::signbit(range)
          || std::isnan(range)
         ) ?
         range_no_obj_ : range;
}

void BeamModel::precalcProb()
{
  float range_obs = 0.0;
  float range_map = 0.0;

  for (size_t i = 0; i < table_size_; ++i) {
    range_obs = table_res_ * i;

    for (size_t j = 0; j < table_size_; ++j) {
      range_map = table_res_ * j;
      weight_table_new_obj_[i][j] = calcProbNewObj(range_obs, range_map);
      weight_table_[i][j] = calcProb(range_obs, range_map);
    }
  }
}

size_t BeamModel::tableIndex(const float range)
{
  return std::min(std::max(0.0, range / table_res_),
                  static_cast<double>(table_size_ - 1)
                 );
}

void BeamModel::removeOutliers(ParticleDistribution& dist)
{
  // Find outlier rays
  std::vector<size_t> rays_rejected;
  size_t reject_count = 0;

  for (size_t i = 0; i < rays_obs_sample_.size(); ++i) {
    // An outlier if this observed ray looks likely to be a new (unexpected) object
    printf("Rejection ratio[%lu] = %.2e\n", i, rays_obs_sample_[i].weight_new_obj_sum_ / rays_obs_sample_[i].weight_sum_);
    if (   (  rays_obs_sample_[i].weight_new_obj_sum_ / rays_obs_sample_[i].weight_sum_
            > SENSOR_WEIGHT_RATIO_NEW_OBJ_THRESHOLD
           )
        && rays_rejected.size() < rays_obs_sample_.size() / 2  // Only reject half at most so we aren't totally blind
       ) {
      rays_rejected.push_back(i);
      printf("Rejected ray[%lu], range = %.2f, angle = %.2f (deg)\n",
             i, rays_obs_sample_[i].range_, rays_obs_sample_[i].th_ * 180.0 / M_PI
            );
    }
  }
  // Recalculate weights, undoing the outlier contributions
  double weight_partial = 0.0;

  if (rays_rejected.size() > 0) {
    for (size_t i = 0; i < dist.count(); ++i) {
      // Reset weight for each rejected index
      for (size_t j = 0; j < rays_rejected.size(); ++j) {
        weight_partial = dist.particle(i).weights_[rays_rejected[j]];

        if (weight_partial > 0.0) {
          dist.particle(i).weight_ /= weight_partial;
        }
        else {
          dist.particle(i).weight_ = 1.0;
        }
      }
    }
  }
}

double BeamModel::lookupProbNewObj(const float range_obs,
                                   const float range_map
                                  )
{
  return weight_table_new_obj_[tableIndex(range_obs)][tableIndex(range_map)];
}

double BeamModel::lookupProb(const float range_obs,
                             const float range_map
                            )
{
  return weight_table_[tableIndex(range_obs)][tableIndex(range_map)];
}

double BeamModel::calcProb(const float range_obs,
                           const float range_map
                          )
{
  return (  calcProbNoObj(range_obs)
          + calcProbNewObj(range_obs, range_map)
          + calcProbMapObj(range_obs, range_map)
          + calcProbRandEffect(range_obs)
         );
}

double BeamModel::calcProbNoObj(const float range_obs)
{
  return weight_no_obj_ * approxEqual(range_obs,
                                      range_no_obj_,
                                      RANGE_EPSILON
                                     );
}

double BeamModel::calcProbNewObj(const float range_obs,
                                 const float range_map
                                )
{
  if (   !approxEqual(range_obs,
                      range_no_obj_,
                      RANGE_EPSILON
                     )
      && range_obs <= range_map
     ) {
    return weight_new_obj_ * (  new_obj_decay_rate_ * std::exp(-new_obj_decay_rate_ * range_obs)
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
  if (!approxEqual(range_obs,
                   range_no_obj_,
                   RANGE_EPSILON
                  )
     ) {
    return weight_map_obj_ * std::exp(  -(range_obs - range_map) * (range_obs - range_map)
                                      / (2 * range_std_dev_ * range_std_dev_)
                                     );
  }
  else {
    return 0.0;
  }
}

double BeamModel::calcProbRandEffect(const float range_obs)
{
  if (!approxEqual(range_obs,
                   range_no_obj_,
                   RANGE_EPSILON
                  )
     ) {
    return weight_rand_effect_ / range_max_;
  }
  else {
    return 0.0;
  }
}
