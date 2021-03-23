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
  precalcWeightedProbs();
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

    // Calculate observed range probabilities for this particle
    weight_partial_new_obj = calc_enable ? calcWeightedProbNewObj(range_obs, range_map) :
                                           lookupWeightedProbNewObj(range_obs, range_map);
    weight_partial = calc_enable ? calcWeightedProb(range_obs, range_map) :
                                   lookupWeightedProb(range_obs, range_map);

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

void BeamModel::tune(const RayScanVector& obs,
                     const Particle& particle
                    )
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

// TBD remove print statements
void BeamModel::removeOutliers(ParticleDistribution& dist)
{
  // Calculate ratios and pair them with their index
  std::vector<std::pair<size_t, double>> weight_ratios(rays_obs_sample_.size());

  for (size_t i = 0; i < rays_obs_sample_.size(); ++i) {
    weight_ratios[i].first = i;

    if (rays_obs_sample_[i].weight_sum_ > 0) {
      weight_ratios[i].second = rays_obs_sample_[i].weight_new_obj_sum_ / rays_obs_sample_[i].weight_sum_;
    }
    else {
      weight_ratios[i].second = 1.0;
    }
    // printf("Outlier ratio[%lu] = %.2e\n", weight_ratios[i].first, weight_ratios[i].second);
  }
  // Sort ratios according to worst outliers first, carrying along the corresponding weight index
  auto worstDescending = [](const std::pair<size_t, double>& ratio_1, const std::pair<size_t, double>& ratio_2)
                           { return ratio_1.second > ratio_2.second; };
  std::sort(weight_ratios.begin(), weight_ratios.end(), worstDescending);

  // Evaluate the sorted weight ratios list and remove the worst outliers, up to half at most
  // Observed ranges and their corresponding weights are considered outliers if they appear
  // likely to be due to a new / unexpected object
  std::vector<size_t> weight_indexes_rejected;
  size_t i = 0;

  while (   i < weight_ratios.size()
         && weight_indexes_rejected.size() < weight_ratios.size() / 2  // Only reject half at most so we aren't totally blind
        ) {
    // printf("Outlier ratio[%lu] = %.2e", weight_ratios[i].first, weight_ratios[i].second);

    if (weight_ratios[i].second > SENSOR_WEIGHT_RATIO_NEW_OBJ_THRESHOLD) {
      weight_indexes_rejected.push_back(weight_ratios[i].first);
      printf("rejected range = %.2f, angle = %.2f (ratio = %.3f)\n",
             rays_obs_sample_[weight_ratios[i].first].range_,
             rays_obs_sample_[weight_ratios[i].first].th_ * 180.0 / M_PI,
             weight_ratios[i].second
            );
    }
    else {
      // printf(", accepted range = %.2f, angle = %.2f\n",
      //        rays_obs_sample_[weight_ratios[i].first].range_, rays_obs_sample_[weight_ratios[i].first].th_ * 180.0 / M_PI
      //       );
      // Once one is accepted, remaining ones will also be acceptable from sorting
      break;
    }
    ++i;
  }
  // Recalculate weights, undoing the outlier contributions
  double weight_partial = 0.0;

  if (weight_indexes_rejected.size() > 0) {
    for (size_t i = 0; i < dist.count(); ++i) {

      // Reset weight for each rejected index
      for (size_t j = 0; j < weight_indexes_rejected.size(); ++j) {
        weight_partial = dist.particle(i).weights_[weight_indexes_rejected[j]];
        weight_partial = std::pow(weight_partial, SENSOR_UNCERTAINTY_FACTOR);

        if (weight_partial > 0.0) {
          // printf("Rejecting outlier angle = %.2f, range = %.2f, ",
          //        rays_obs_sample_[weight_ratios[j].first].th_ * 180.0 / M_PI,
          //        rays_obs_sample_[weight_ratios[j].first].range_
          //       );
          // printf("weight before = %.2e, ", dist.particle(i).weight_);
          dist.particle(i).weight_ /= weight_partial;
          // printf("weight after = %.2e\n", dist.particle(i).weight_);
        }
        else {
          dist.particle(i).weight_ = 1.0;
        }
      }
    }
  }
}

size_t BeamModel::tableIndex(const float range)
{
  return std::min(std::max(0.0, range / table_res_),
                  static_cast<double>(table_size_ - 1)
                 );
}

double BeamModel::lookupWeightedProbNewObj(const float range_obs,
                                           const float range_map
                                          )
{
  return weight_table_new_obj_[tableIndex(range_obs)][tableIndex(range_map)];
}

double BeamModel::lookupWeightedProb(const float range_obs,
                                     const float range_map
                                    )
{
  return weight_table_[tableIndex(range_obs)][tableIndex(range_map)];
}

double BeamModel::calcProbNoObj(const float range_obs)
{
  return approxEqual(range_obs, range_no_obj_, RANGE_EPSILON);
}

double BeamModel::calcProbNewObj(const float range_obs,
                                 const float range_map
                                )
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

double BeamModel::calcProbMapObj(const float range_obs,
                                 const float range_map
                                )
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

double BeamModel::calcProbRandEffect(const float range_obs)
{
  if (!approxEqual(range_obs, range_no_obj_, RANGE_EPSILON)) {
    return 1.0 / range_max_;
  }
  else {
    return 0.0;
  }
}

double BeamModel::calcWeightedProbNoObj(const float range_obs)
{
  return weight_no_obj_ * calcProbNoObj(range_obs);
}

double BeamModel::calcWeightedProbNewObj(const float range_obs,
                                         const float range_map
                                        )
{
  return weight_new_obj_ * calcProbNewObj(range_obs, range_map);
}

double BeamModel::calcWeightedProbMapObj(const float range_obs,
                                         const float range_map
                                        )
{
  return weight_map_obj_ * calcProbMapObj(range_obs, range_map);
}

double BeamModel::calcWeightedProbRandEffect(const float range_obs)
{
  return weight_rand_effect_ * calcProbRandEffect(range_obs);
}

double BeamModel::calcWeightedProb(const float range_obs,
                                   const float range_map
                                  )
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

  for (size_t i = 0; i < table_size_; ++i) {
    range_obs = table_res_ * i;

    for (size_t j = 0; j < table_size_; ++j) {
      range_map = table_res_ * j;
      weight_table_new_obj_[i][j] = calcWeightedProbNewObj(range_obs, range_map);
      weight_table_[i][j] = calcWeightedProb(range_obs, range_map);
    }
  }
}