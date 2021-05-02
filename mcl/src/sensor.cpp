#include "mcl/sensor.h"

static const float RANGE_STD_DEV = 0.2;                       // Standard deviation in range measurements
static const float NEW_OBJ_DECAY_RATE = 0.5;                  // Decay rate for new / unexpected object probability
static const double WEIGHT_NO_OBJ = 0.0;                      // Weight for no object detected probability
static const double WEIGHT_NEW_OBJ = 15.0;                    // Weight for new / unexpected object probability
static const double WEIGHT_MAP_OBJ = 83.0;                    // Weight for mapped / expected object probability
static const double WEIGHT_RAND_EFFECT = 2.0;                 // Weight for random effect probability
static const double WEIGHT_UNCERTAINTY_FACTOR = 1.1;          // Weight uncertainty factor (extra noise added)
static const double WEIGHT_RATIO_REJECTION_THRESHOLD = 0.30;  // Weight ratio above which a ray is rejected for likely representing an unexpected object
static const double WEIGHT_TABLE_RES = 0.01;                  // Lookup table resolution (meters per cell)
static const unsigned int TH_RAYCAST_COUNT = 314;             // Number of angles for raycast approximation (count per revolution)
static const float RANGE_EPSILON = 1e-5;                      // Maximum delta between two ranges such that they are still considered 'equal'

using namespace localize;

// ========== BeamModel ========== //
BeamModel::BeamModel(const float range_min,
                     const float range_max,
                     const float range_no_obj,
                     const Map& map
                    ) :
  range_min_(range_min),
  range_max_(range_max),
  range_no_obj_(range_no_obj),
  range_std_dev_(RANGE_STD_DEV),
  new_obj_decay_rate_(NEW_OBJ_DECAY_RATE),
  weights_sum_(  WEIGHT_NO_OBJ
               + WEIGHT_NEW_OBJ
               + WEIGHT_MAP_OBJ
               + WEIGHT_RAND_EFFECT
              ),
  weight_no_obj_(WEIGHT_NO_OBJ / weights_sum_),
  weight_new_obj_(WEIGHT_NEW_OBJ / weights_sum_),
  weight_map_obj_(WEIGHT_MAP_OBJ / weights_sum_),
  weight_rand_effect_(WEIGHT_RAND_EFFECT / weights_sum_),
  weight_table_res_(WEIGHT_TABLE_RES),
  weight_table_size_(std::round(range_max / WEIGHT_TABLE_RES) + 1.0),
  weight_table_new_obj_(weight_table_size_, std::vector<double>(weight_table_size_)),
  weight_table_(weight_table_size_, std::vector<double>(weight_table_size_)),
  rays_obs_sample_(SENSOR_TH_SAMPLE_COUNT),
  raycaster_(ranges::OMap(map.xSize(),
                          map.ySize(),
                          map.xOriginWorld(),
                          map.yOriginWorld(),
                          map.thWorld(),
                          map.scaleWorld(),
                          map.data()
                         ),
             range_max / map.scaleWorld(),
             TH_RAYCAST_COUNT
            ),
  th_sample_dist_(0.0, L_2PI / SENSOR_TH_SAMPLE_COUNT)
{
  // Precalculate the sensor model and create tables
  precalcWeightedProbs();
}

void BeamModel::apply(Particle& particle, const bool calc_enable)
{
  double weight_partial_new_obj = 0.0;
  double weight_partial = 0.0;
  double weight = 1.0;
  float range_map = 0.0;

  for (size_t i = 0; i < rays_obs_sample_.size(); ++i) {
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
  particle.weight_ = std::pow(weight, WEIGHT_UNCERTAINTY_FACTOR);
}

void BeamModel::apply(ParticleDistribution& dist, const bool calc_enable)
{
  // Calculate particle weights
  for (size_t i = 0; i < dist.count(); ++i) {
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
  // Sample and update the observation
  update(obs);

  // Calculate particle weights
  apply(dist, calc_enable);
}

void BeamModel::update(const RayScan& obs)
{
  rays_obs_sample_ = sample(obs, SENSOR_TH_SAMPLE_COUNT);
}

RaySampleVector BeamModel::sample(const RayScan& obs, const size_t sample_count)
{
  RaySampleVector rays_obs_sample;
  std::vector<bool> sample_int_empty(sample_count, false);  // Indicates if an observation interval has no hits

  if (obs.rays_.size() > 0) {
    // Generate a random index to start from so we don't repeat samples / directions
    size_t o_step_size = std::max(static_cast<size_t>((L_2PI / sample_count) / obs.th_inc_), 1ul);
    size_t o = std::min(static_cast<size_t>(th_sample_dist_(rng_.engine()) / obs.th_inc_), o_step_size - 1ul);
    size_t s = 0;
    size_t sample_int_empty_count = 0;

    // Iterate through observations selecting the desired amount of samples
    while (   rays_obs_sample.size() < sample_count
           && rays_obs_sample.size() < obs.rays_.size()
           && sample_int_empty_count < sample_count
           && sample_int_empty_count < obs.rays_.size()
          ) {
      // printf("o = %lu\n", o);
      // Search this interval only if it hasn't been found empty yet
      if (!sample_int_empty[s]) {
        size_t o_start = o;
        float range_o = repairRange(obs.rays_[o].range_);

        // Cycle through this sample interval until we find an observation that hit something
        while (approxEqual(range_o, range_no_obj_, RANGE_EPSILON)) {
          // Wrap around if we reached the end of the sample interval
          if (++o >= o_step_size * (s + 1)) {
            o = o_step_size * s;
          }
          // If we get back to the start index, all observations in this sample interval are misses
          // Mark the interval as empty so we don't search it again later, and move on to the next
          if (o == o_start) {
            sample_int_empty[s] = true;
            ++sample_int_empty_count;
            break;
          }
          // Otherwise, get the next observed range for examination
          else {
            range_o = repairRange(obs.rays_[o].range_);
          }
        }
        // If the interval is not empty after the search, we found a hit to add to the sample set
        if (!sample_int_empty[s]) {
          rays_obs_sample.push_back(RaySample(range_o, obs.rays_[o].th_));
        }
      }
      // Increment the sample interval we're searching and reset indexes on rollover
      if (++s >= sample_count) {
        s = 0;
        o = std::min(static_cast<size_t>(th_sample_dist_(rng_.engine()) / obs.th_inc_), o_step_size - 1ul);
      }
      o = o_step_size * s;
    }
  }
  return rays_obs_sample;
}

void BeamModel::removeOutliers(ParticleDistribution& dist)
{
  // Calculate ratios and pair them with their index
  IndexedWeightVector outlier_weight_ratios(rays_obs_sample_.size());

  for (size_t i = 0; i < rays_obs_sample_.size(); ++i) {
    outlier_weight_ratios[i].index_ = i;

    if (rays_obs_sample_[i].weight_sum_ > 0.0) {
      outlier_weight_ratios[i].val_ = rays_obs_sample_[i].weight_new_obj_sum_ / rays_obs_sample_[i].weight_sum_;
    }
    else {
      outlier_weight_ratios[i].val_ = 0.0;
    }
  }
  // Sort ratios according to largest outliers first, carrying along the corresponding weight index
  std::sort(outlier_weight_ratios.begin(), outlier_weight_ratios.end(), Greater());

  // Evaluate the sorted weight ratios and remove the largest outliers, up to half at most
  // Observed ranges and their corresponding weights are considered outliers if they appear
  // likely to be due to a new / unexpected object
  size_t i = 0;
  size_t reject_count = 0;

  while (   i < outlier_weight_ratios.size()
         && reject_count < outlier_weight_ratios.size() / 2  // Only reject half at most so we aren't totally blind
        ) {
    if (outlier_weight_ratios[i].val_ > WEIGHT_RATIO_REJECTION_THRESHOLD) {
      ++reject_count;
      // printRejectedRange(rays_obs_sample_[outlier_weight_ratios[i].index_], outlier_weight_ratios[i].val_);
    }
    else {
      break;  // Once one is accepted, remaining ones will also be acceptable from sorting
    }
    ++i;
  }
  // Recalculate weights, undoing the outlier contributions
  if (outlier_weight_ratios.size() > 0) {
    for (size_t i = 0; i < dist.count(); ++i) {

      // Remove weight for each rejected index
      for (size_t j = 0; j < reject_count; ++j) {
        double& weight_partial = dist.particle(i).weights_[outlier_weight_ratios[j].index_];
        weight_partial = std::pow(weight_partial, WEIGHT_UNCERTAINTY_FACTOR);

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

size_t BeamModel::tableIndex(const float range) const
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
    return (  new_obj_decay_rate_ * std::exp(-new_obj_decay_rate_ * range_obs)
            / (1 - new_obj_decay_rate_ * std::exp(-new_obj_decay_rate_ * range_map))
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

  for (size_t i = 0; i < weight_table_size_; ++i) {
    range_obs = weight_table_res_ * i;

    for (size_t j = 0; j < weight_table_size_; ++j) {
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
