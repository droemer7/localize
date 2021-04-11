#include "mcl/sensor.h"

static const float RANGE_STD_DEV = 0.5;                       // Standard deviation in range measurements
static const float NEW_OBJ_DECAY_RATE = 0.5;                  // Decay rate for new / unexpected object probability
static const double WEIGHT_NO_OBJ = 15.0;                     // Weight for no object detected probability
static const double WEIGHT_NEW_OBJ = 5.0;                     // Weight for new / unexpected object probability
static const double WEIGHT_MAP_OBJ = 75.0;                    // Weight for mapped / expected object probability
static const double WEIGHT_RAND_EFFECT = 5.0;                 // Weight for random effect probability
static const double WEIGHT_UNCERTAINTY_FACTOR = 1.1;          // Weight uncertainty factor (extra noise added)
static const double WEIGHT_RATIO_REJECTION_THRESHOLD = 0.50;  // Weight ratio above which a ray is rejected for likely representing an unexpected object
static const double WEIGHT_TABLE_RES = 0.01;                  // Lookup table resolution (meters per cell)
static const unsigned int TH_RAYCAST_COUNT = 656;             // Number of angles for raycast approximation (count per revolution)
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
  raycaster_(map,
             range_max / map.scale,
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
    range_map = repair(range_map);

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
  return;
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

void BeamModel::tune(const RayScanVector& obs, const Particle& particle)
{
  printf("===== Sensor tuning =====\n");
  std::vector<double> ranges_obs;
  std::vector<double> ranges_map;

  for (size_t i = 0; i < obs.size(); ++i) {
    for (size_t j = 0; j < obs[i].rays_.size(); ++j) {
      double range_obs = obs[i].rays_[j].range_;
      double range_map = raycaster_.calc_range(particle.x_,
                                               particle.y_,
                                               particle.th_ + obs[i].rays_[j].th_
                                              );
      ranges_obs.push_back(repair(range_obs));
      ranges_map.push_back(repair(range_map));
    }
  }
  size_t n = 0;

  while (n < 40) { // TBD change this into a real convergence condition
    double prob_sum = 0.0;
    double prob_no_obj = 0.0;
    double prob_new_obj = 0.0;
    double prob_map_obj = 0.0;
    double prob_rand_effect = 0.0;
    std::vector<double> probs_no_obj;
    std::vector<double> probs_new_obj;
    std::vector<double> probs_map_obj;
    std::vector<double> probs_rand_effect;

    for (size_t i = 0; i < ranges_obs.size(); ++i) {
      prob_no_obj = calcWeightedProbNoObj(ranges_obs[i]);
      prob_new_obj = calcWeightedProbNewObj(ranges_obs[i], ranges_map[i]);
      prob_map_obj = calcWeightedProbMapObj(ranges_obs[i], ranges_map[i]);
      prob_rand_effect = calcWeightedProbRandEffect(ranges_obs[i]);
      prob_sum = (  prob_no_obj
                  + prob_new_obj
                  + prob_map_obj
                  + prob_rand_effect
                 );
      probs_no_obj.push_back(prob_sum > 0.0 ? prob_no_obj / prob_sum : 0.0);
      probs_new_obj.push_back(prob_sum > 0.0 ? prob_new_obj / prob_sum : 0.0);
      probs_map_obj.push_back(prob_sum > 0.0 ? prob_map_obj / prob_sum : 0.0);
      probs_rand_effect.push_back(prob_sum > 0.0 ? prob_rand_effect / prob_sum : 0.0);
    }
    if (ranges_obs.size() > 0) {
      double probs_sum_no_obj = 0.0;
      double probs_sum_new_obj = 0.0;
      double probs_sum_new_obj_range = 0.0;
      double probs_sum_map_obj = 0.0;
      double probs_sum_map_obj_err_sq = 0.0;
      double probs_sum_rand_effect = 0.0;

      for (size_t i = 0; i < ranges_obs.size(); ++i) {
        probs_sum_no_obj += probs_no_obj[i];
        probs_sum_new_obj += probs_new_obj[i];
        probs_sum_new_obj_range += probs_new_obj[i] * ranges_obs[i];
        probs_sum_map_obj += probs_map_obj[i];
        probs_sum_map_obj_err_sq += (  probs_map_obj[i]
                                     * (ranges_obs[i] - ranges_map[i])
                                     * (ranges_obs[i] - ranges_map[i])
                                    );
        probs_sum_rand_effect += probs_rand_effect[i];
      }
      weight_no_obj_ = probs_sum_no_obj / ranges_obs.size();
      weight_new_obj_ = probs_sum_new_obj / ranges_obs.size();
      weight_map_obj_ = probs_sum_map_obj / ranges_obs.size();
      weight_rand_effect_ = probs_sum_rand_effect / ranges_obs.size();
      range_std_dev_ = probs_sum_map_obj > 0.0 ? std::sqrt(probs_sum_map_obj_err_sq / probs_sum_map_obj) :
                                                 range_std_dev_;
      new_obj_decay_rate_ = probs_sum_new_obj_range > 0.0 ? probs_sum_new_obj / probs_sum_new_obj_range :
                                                            new_obj_decay_rate_;
    }
    ++n;
  }
  printf("Weight no object = %.4f\n", weight_no_obj_);
  printf("Weight new object = %.4f\n", weight_new_obj_);
  printf("Weight map object = %.4f\n", weight_map_obj_);
  printf("Weight random effect = %.4f\n", weight_rand_effect_);
  printf("Range sigma = %.4f\n", range_std_dev_);
  printf("New object decay rate = %.4f\n", new_obj_decay_rate_);
  printf("===== Sensor tuning complete =====\n");
}

RaySampleVector BeamModel::sample(const RayScan& obs)
{
  RaySampleVector rays_obs_sample(rays_obs_sample_.size());

  // More than one observation
  if (   obs.rays_.size() > 1
      && rays_obs_sample.size() > 0
     ) {
    // Generate a random offset for the sampled set to start from
    size_t o_step_size = (L_2PI / rays_obs_sample.size()) / obs.th_inc_;
    size_t o = th_sample_dist_(rng_.engine()) / obs.th_inc_;
    size_t s = 0;

    // Iterate through both arrays selecting the desired amount of samples
    while (   o < obs.rays_.size()
           && s < rays_obs_sample.size()
          ) {
      rays_obs_sample[s] = RaySample(obs.rays_[o]);
      rays_obs_sample[s].range_ = repair(rays_obs_sample[s].range_);
      o += o_step_size;
      ++s;
    }
    rays_obs_sample.resize(s);
  }
  // Only one observation
  else if (   obs.rays_.size() == 1
           && rays_obs_sample.size() > 0
          ) {
    rays_obs_sample[0] = RaySample(obs.rays_[0]);
    rays_obs_sample[0].range_ = repair(rays_obs_sample[0].range_);
    rays_obs_sample.resize(1);
  }
  return rays_obs_sample;
}

// TBD remove print statements
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

float BeamModel::repair(float range) const
{
  return (   range > range_max_
          || std::signbit(range)
          || std::isnan(range)
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