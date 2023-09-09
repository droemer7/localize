#include "amcl/sensor.h"

static const double WEIGHT_TABLE_RES = 0.01;      // Lookup table resolution (meters per cell)
static const int RAYCAST_TH_COUNT = 314; // Number of angles for raycast approximation (count per revolution)
static const float RANGE_EPSILON = 1e-5;          // Maximum delta between two ranges such that they are still considered 'equal'

using namespace localize;

// ========== BeamModel ========== //
BeamModel::BeamModel(const float range_min,
                     const float range_max,
                     const float range_no_obj,
                     const float range_std_dev,
                     const float decay_rate_new_obj,
                     const double weight_no_obj,
                     const double weight_new_obj,
                     const double weight_map_obj,
                     const double weight_rand_effect,
                     const double weight_uncertainty_factor,
                     const double prob_new_obj_reject,
                     const Map& map
                    ) :
  range_min_(range_min),
  range_max_(range_max),
  range_no_obj_(range_no_obj),
  range_std_dev_(range_std_dev),
  decay_rate_new_obj_(decay_rate_new_obj),
  weights_sum_(  weight_no_obj
               + weight_new_obj
               + weight_map_obj
               + weight_rand_effect
              ),
  weight_no_obj_(weight_no_obj / weights_sum_),
  weight_new_obj_(weight_new_obj / weights_sum_),
  weight_map_obj_(weight_map_obj / weights_sum_),
  weight_rand_effect_(weight_rand_effect / weights_sum_),
  weight_uncertainty_factor_(weight_uncertainty_factor),
  prob_new_obj_reject_(prob_new_obj_reject),
  weight_table_res_(WEIGHT_TABLE_RES),
  weight_table_size_(std::round(range_max / WEIGHT_TABLE_RES) + 1.0),
  weight_table_new_obj_(weight_table_size_, std::vector<double>(weight_table_size_)),
  weight_table_(weight_table_size_, std::vector<double>(weight_table_size_)),
  raycaster_(ranges::OMap(map.xSize(),
                          map.ySize(),
                          map.xOriginWorld(),
                          map.yOriginWorld(),
                          map.thWorld(),
                          map.scaleWorld(),
                          map.data()
                         ),
             range_max / map.scaleWorld(),
             RAYCAST_TH_COUNT
            ),
  ray_sample_count_(SENSOR_RAY_SAMPLE_COUNT),
  th_sample_dist_(0.0, L_2PI / SENSOR_RAY_SAMPLE_COUNT)
{
  // Reserve space since we need to start with 0 elements
  rays_obs_sample_.reserve(ray_sample_count_);

  // Precalculate the sensor model and create tables
  precalcWeightedProbs();
}

void BeamModel::apply(Particle& particle, const bool calc_enable)
{
  double weight_partial_new_obj = 0.0;
  double weight_partial = 0.0;
  double weight = 1.0;
  float range_map = 0.0;

  for (int i = 0; i < static_cast<int>(rays_obs_sample_.size()); ++i) {
    // Calculate range from the map
    range_map = raycaster_.calc_range(particle.x_,
                                      particle.y_,
                                      particle.th_ + rays_obs_sample_[i].th_
                                     );
    range_map = repairRange(range_map);

    // Calculate observed range probabilities for this particle
    weight_partial_new_obj = calc_enable ? calcWeightedProbNewObj(rays_obs_sample_[i].range_, range_map) :
                                           lookupWeightedProbNewObj(rays_obs_sample_[i].range_, range_map);
    weight_partial = calc_enable ? calcWeightedProb(rays_obs_sample_[i].range_, range_map) :
                                   lookupWeightedProb(rays_obs_sample_[i].range_, range_map);

    // Update weight sums for this sampled ray
    rays_obs_sample_[i].weight_new_obj_sum_ += weight_partial_new_obj;
    rays_obs_sample_[i].weight_sum_ += weight_partial;

    // Save partial weight to particle in case we determine later its an outlier and need to remove its effect
    particle.weights_[i] = weight_partial;
    weight *= weight_partial;
  }
  // Update full weight, applying overall model uncertainty
  particle.weight_ = std::pow(weight, weight_uncertainty_factor_);
}

void BeamModel::apply(ParticleDistribution& dist, const bool calc_enable)
{
  // Calculate particle weights
  for (int i = 0; i < dist.count(); ++i) {
    apply(dist.particle(i));
  }
  // Remove the contribution of outliers to the particle weights
  removeOutliers(dist);

  // Recalculate weight statistics
  dist.update();
}

void BeamModel::apply(ParticleDistribution& dist,
                      const RayScan& obs,
                      const bool calc_enable
                     )
{
  // Sample from the observation
  sample(obs);

  // Calculate particle weights
  apply(dist, calc_enable);
}

void BeamModel::sample(const RayScan& obs)
{
  rays_obs_sample_.resize(ray_sample_count_);
  int s = 0;

  if (obs.rays_.size() > 0) {
    // Generate a random index to start from so we don't repeat the same direction every time
    // This helps maintain some variability from scan to scan to uphold the Markov assumption of independence
    int o_step_size = std::max(static_cast<int>((L_2PI / ray_sample_count_) / obs.th_inc_), 1);
    int o_offset = std::min(static_cast<int>(th_sample_dist_(rng_.engine()) / obs.th_inc_), o_step_size - 1);
    int o = o_offset;
    int o_start = o;
    float range_o = 0.0;

    // Iterate through observations selecting the desired amount of samples
    while (   s < ray_sample_count_
           && s < static_cast<int>(obs.rays_.size())
          ) {
      o_start = o;
      range_o = repairRange(obs.rays_[o].range_);

      // Cycle through this sample interval looking for an observation that hit something
      while (approxEqual(range_o, range_no_obj_, RANGE_EPSILON)) {
        // Wrap around if we reached the end of the sample interval
        if (++o >= o_step_size * (s + 1)) {
          o = o_step_size * s;
        }
        // Get the next observation
        range_o = repairRange(obs.rays_[o].range_);

        // If we get back to the start index, all observations in this sample interval were misses
        // In this case we accept the miss and use it, since it's more likely to not be erroneous
        // Additionally, we don't want to reject misses _all_ the time (e.g., in larger open spaces)
        if (o == o_start) {
          break;
        }
      }
      rays_obs_sample_[s].range_ = range_o;
      rays_obs_sample_[s].th_ = obs.rays_[o].th_;
      o = o_step_size * ++s + o_offset;
    }
  }
  // Resize in case for some reason the observation had fewer samples than we wanted
  rays_obs_sample_.resize(s);
}

void BeamModel::removeOutliers(ParticleDistribution& dist)
{
  // Calculate probabilities and pair them with their index
  IndexedWeightVector outlier_probs(rays_obs_sample_.size());

  for (int i = 0; i < static_cast<int>(rays_obs_sample_.size()); ++i) {
    outlier_probs[i].index_ = i;

    if (rays_obs_sample_[i].weight_sum_ > 0.0) {
      outlier_probs[i].val_ = rays_obs_sample_[i].weight_new_obj_sum_ / rays_obs_sample_[i].weight_sum_;
    }
    else {
      outlier_probs[i].val_ = 0.0;
    }
  }
  // Sort probabilities according to largest outliers first, carrying along the corresponding weight index
  std::sort(outlier_probs.begin(), outlier_probs.end(), Greater());

  // Evaluate the sorted probabilities and remove the largest outliers, up to half at most
  // Observed ranges and their corresponding weights are considered outliers if they appear likely to be due to a
  // new / unexpected object
  int i = 0;
  int reject_count = 0;

  while (   i < static_cast<int>(outlier_probs.size())
         && reject_count < static_cast<int>(outlier_probs.size()) / 2  // Only reject half at most
        ) {
    if (outlier_probs[i].val_ > prob_new_obj_reject_) {
      ++reject_count;
    }
    else {
      break;  // Once one is accepted, remaining ones will also be acceptable from sorting
    }
    ++i;
  }
  // Recalculate weights, undoing the outlier contributions
  if (outlier_probs.size() > 0) {
    for (int i = 0; i < dist.count(); ++i) {

      // Remove weight for each rejected index
      for (int j = 0; j < reject_count; ++j) {
        double& weight_partial = dist.particle(i).weights_[outlier_probs[j].index_];
        weight_partial = std::pow(weight_partial, weight_uncertainty_factor_);

        dist.particle(i).weight_ = weight_partial > 0.0 ? dist.particle(i).weight_ / weight_partial
                                                        : dist.particle(i).weight_ ;
        weight_partial = 0.0;
      }
    }
  }
}

float BeamModel::repairRange(float range) const
{
  return (   range > range_max_
          || std::signbit(range)
          || !std::isfinite(range)
         ) ?
         range_no_obj_ : range;
}

int BeamModel::tableIndex(const float range) const
{
  return range / weight_table_res_;
}

double BeamModel::lookupWeightedProbNewObj(const float range_obs, const float range_map) const
{
  return weight_table_new_obj_[tableIndex(range_obs)][tableIndex(range_map)];
}

double BeamModel::lookupWeightedProb(const float range_obs, const float range_map) const
{
  return weight_table_[tableIndex(range_obs)][tableIndex(range_map)];
}

double BeamModel::calcProbNoObj(const float range_obs) const
{
  return approxEqual(range_obs, range_no_obj_, RANGE_EPSILON);
}

double BeamModel::calcProbNewObj(const float range_obs, const float range_map) const
{
  if (   !approxEqual(range_obs, range_no_obj_, RANGE_EPSILON)
      && range_obs <= range_map
     ) {
    return (  decay_rate_new_obj_ * std::exp(-decay_rate_new_obj_ * range_obs)
            / (1 - decay_rate_new_obj_ * std::exp(-decay_rate_new_obj_ * range_map))
           );
  }
  else {
    return 0.0;
  }
}

double BeamModel::calcProbMapObj(const float range_obs, const float range_map) const
{
  if (!approxEqual(range_obs, range_no_obj_, RANGE_EPSILON)) {
    return std::exp(  -(range_obs - range_map) * (range_obs - range_map)
                    / (2 * range_std_dev_ * range_std_dev_)
                   );
  }
  else {
    return 0.0;
  }
}

double BeamModel::calcProbRandEffect(const float range_obs) const
{
  if (!approxEqual(range_obs, range_no_obj_, RANGE_EPSILON)) {
    return 1.0 / range_max_;
  }
  else {
    return 0.0;
  }
}

double BeamModel::calcWeightedProbNoObj(const float range_obs) const
{
  return weight_no_obj_ * calcProbNoObj(range_obs);
}

double BeamModel::calcWeightedProbNewObj(const float range_obs, const float range_map) const
{
  return weight_new_obj_ * calcProbNewObj(range_obs, range_map);
}

double BeamModel::calcWeightedProbMapObj(const float range_obs, const float range_map) const
{
  return weight_map_obj_ * calcProbMapObj(range_obs, range_map);
}

double BeamModel::calcWeightedProbRandEffect(const float range_obs) const
{
  return weight_rand_effect_ * calcProbRandEffect(range_obs);
}

double BeamModel::calcWeightedProb(const float range_obs, const float range_map) const
{
  return (  calcWeightedProbNoObj(range_obs)
          + calcWeightedProbNewObj(range_obs, range_map)
          + calcWeightedProbMapObj(range_obs, range_map)
          + calcWeightedProbRandEffect(range_obs)
         );
}

void BeamModel::precalcWeightedProbs()
{
  double range_obs = 0.0;
  double range_map = 0.0;

  for (int i = 0; i < weight_table_size_; ++i) {
    range_obs = weight_table_res_ * i;

    for (int j = 0; j < weight_table_size_; ++j) {
      range_map = weight_table_res_ * j;
      weight_table_new_obj_[i][j] = calcWeightedProbNewObj(range_obs, range_map);
      weight_table_[i][j] = calcWeightedProb(range_obs, range_map);
    }
  }
}

void BeamModel::printRejectedRange(const Ray & ray, const double prob) const
{
  printf("Rejected range = %.2f, angle = %.2f (prob = %.3f)\n",
         ray.range_,
         ray.th_ * 180.0 / L_PI,
         prob
        );
}
