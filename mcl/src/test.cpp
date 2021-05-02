#include <chrono>
#include <float.h>
#include <stdio.h>
#include <unistd.h>

#include <tf2_ros/transform_listener.h>

#include "mcl/mcl.h"
#include "mcl/sensor.h"
#include "mcl/common.h"
#include "mcl/util.h"

#include "includes/RangeLib.h"

using namespace localize;

typedef std::chrono::high_resolution_clock Clock;
std::chrono::_V2::high_resolution_clock::time_point start;
std::chrono::_V2::high_resolution_clock::time_point end;
std::chrono::duration<double> dur;

RaySampleVector rays_obs_sample_(8);
RNG rng_;
std::uniform_real_distribution<float> th_sample_dist_(0.0, L_2PI / SENSOR_TH_SAMPLE_COUNT);
std::uniform_real_distribution<float> range_dist_(0.0, 11.0);
std::uniform_real_distribution<float> prob_dist_(0.0, 1.0);
float range_no_obj_ = 0.0;
static const float RANGE_EPSILON = 1e-5;

void testAngleUtils()
{
  double top = -179.0 * L_PI / 180.0;
  double bot = 175.0 * L_PI / 180.0;
  printf("th_delta = %.4f\n", angleDelta(top, bot) * 180.0 / L_PI);

  top = 230.0 * L_PI / 180.0;
  bot = -10.0 * L_PI / 180.0;
  printf("\n");
  printf("th_delta = %.4f\n", angleDelta(top, bot) * 180.0 / L_PI);

  top = -175.0 * L_PI / 180.0;
  bot = 150.0 * L_PI / 180.0;
  printf("\n");
  printf("th_delta = %.4f\n", angleDelta(top, bot) * 180.0 / L_PI);

  top = 0.0 * L_PI / 180.0;
  bot = -10.0 * L_PI / 180.0;
  size_t top_count = 1;
  size_t bot_count = 3;

  printf("\n");
  printf("th_delta = %.4f\n", angleDelta(top, bot) * 180.0 / L_PI);
  printf("th_avg = %.4f\n", wrapAngle(top + angleDelta(top, bot) * bot_count / (bot_count + top_count)) * 180.0 / L_PI);
}

void testTransforms()
{
  printf("\n");
  Pose base_to_map(0.938, 0.377, DEG2RAD(56.507));
  Pose map_to_base;

  // Rotate
  map_to_base.x_ = -(base_to_map.x_ * std::cos(-base_to_map.th_) - base_to_map.y_ * std::sin(-base_to_map.th_));
  map_to_base.y_ = -(base_to_map.x_ * std::sin(-base_to_map.th_) + base_to_map.y_ * std::cos(-base_to_map.th_));
  map_to_base.th_ = -base_to_map.th_;

  printf("map_to_base:\n");
  printf("x = %.3f\n", map_to_base.x_);
  printf("y = %.3f\n", map_to_base.y_);
  printf("th = %.3f\n", RAD2DEG(map_to_base.th_));
  printf("==========================\n");

  Pose base_to_odom(0.960, 0.632, DEG2RAD(57.268));
  Pose odom_to_map;

  odom_to_map.x_ = (  base_to_odom.x_
                    + (  map_to_base.x_ * std::cos(base_to_odom.th_)
                       - map_to_base.y_ * std::sin(base_to_odom.th_)
                      )
                   );
  odom_to_map.y_ = (  base_to_odom.y_
                    + (  map_to_base.x_ * std::sin(base_to_odom.th_)
                       + map_to_base.y_ * std::cos(base_to_odom.th_)
                      )
                   );
  odom_to_map.th_ = base_to_odom.th_ + map_to_base.th_;

  printf("odom_to_map:\n");
  printf("x = %.3f\n", odom_to_map.x_);
  printf("y = %.3f\n", odom_to_map.y_);
  printf("th = %.3f\n", RAD2DEG(odom_to_map.th_));
  printf("==========================\n");

  Pose map_to_odom;

  // Rotate
  map_to_odom.x_ = -(odom_to_map.x_ * std::cos(-odom_to_map.th_) - odom_to_map.y_ * std::sin(-odom_to_map.th_));
  map_to_odom.y_ = -(odom_to_map.x_ * std::sin(-odom_to_map.th_) + odom_to_map.y_ * std::cos(-odom_to_map.th_));
  map_to_odom.th_ = -odom_to_map.th_;

  printf("map_to_odom:\n");
  printf("x = %.3f\n", map_to_odom.x_);
  printf("y = %.3f\n", map_to_odom.y_);
  printf("th = %.3f\n", RAD2DEG(map_to_odom.th_));
  printf("==========================\n");
}

RayScan generateScan(const size_t num_rays, const size_t num_consec_miss, float prob_miss)
{
  RayScan scan(num_rays);
  size_t i = 0;
  float th = 0.0;
  scan.th_inc_ = num_rays > 0 ? L_2PI / num_rays : 0.0;

  while (i < scan.rays_.size()) {
    if (prob_dist_(rng_.engine()) > prob_miss) {
      for (size_t j = i; j < (num_consec_miss + i) && j < scan.rays_.size(); ++j) {
        scan.rays_[j].range_ = 0.0;
        scan.rays_[j].th_ = th;
        th += scan.th_inc_;
      }
      i += num_consec_miss;
    }
    else {
      scan.rays_[i].range_ = range_dist_(rng_.engine());
      scan.rays_[i].th_ = th;
      th += scan.th_inc_;
      ++i;
    }
  }
  return scan;
}

float repairRange(float range)
{
  return (   range > 11.0
          || std::signbit(range)
          || !std::isfinite(range)
         ) ?
         0.0 : range;
}

RaySampleVector sample(const RayScan& obs, const size_t sample_count)
{
  size_t iterations = 0;
  RaySampleVector rays_obs_sample;
  std::vector<bool> sample_int_empty(sample_count, false);  // Indicates if an observation interval has no hits

  if (obs.rays_.size() > 0) {
    // Generate a random index to start from so we don't repeat samples / directions
    size_t o_step_size = std::max(static_cast<size_t>((L_2PI / sample_count) / obs.th_inc_), 1ul);
    size_t o = std::min(static_cast<size_t>(th_sample_dist_(rng_.engine()) / obs.th_inc_), o_step_size - 1ul);
    size_t s = 0;
    size_t sample_int_empty_count = 0;

    printf("o_step_size = %lu\n", o_step_size);

    // Iterate through observations selecting the desired amount of samples
    while (   rays_obs_sample.size() < sample_count
           && rays_obs_sample.size() < obs.rays_.size()
           && sample_int_empty_count < sample_count
           && sample_int_empty_count < obs.rays_.size()
          ) {
      // Search this interval only if it hasn't been found empty yet
      if (!sample_int_empty[s]) {
        ++iterations;
        size_t o_start = o;
        float range_o = repairRange(obs.rays_[o].range_);

        // Cycle through this sample interval until we find an observation that hit something
        while (approxEqual(range_o, range_no_obj_, RANGE_EPSILON)) {
          ++iterations;
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
          printf("Selected ray[%lu] = %.2f\n", o, range_o);
        }
      }
      // Increment the sample interval we're searching and reset indexes on rollover
      if (++s >= sample_count) {
        s = 0;
        o = std::min(static_cast<size_t>(th_sample_dist_(rng_.engine()) / obs.th_inc_), o_step_size - 1ul);
      }
      o = o_step_size * s;
    }
    printf("Iterations = %lu\n", iterations);
  }
  return rays_obs_sample;
}

void testRaySampling()
{
  RayScan scan = generateScan(360, 1, 0.5);

  for (size_t i = 0; i < scan.rays_.size(); ++i) {
    printf("ray[%lu] = %.2f, %.2f\n", i, scan.rays_[i].range_, RAD2DEG(scan.rays_[i].th_));
  }
  printf("=========================\n");

  start = Clock::now();
  RaySampleVector samples = sample(scan, 8);
  end = Clock::now();
  dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);

  for (size_t i = 0; i < samples.size(); ++i) {
    printf("sample[%lu] = %.2f, %.2f\n", i, samples[i].range_, RAD2DEG(samples[i].th_));
  }
  printf("Duration = %.4f\n", dur.count() * 1000.0);
  sleep(1);
}

void steeringAngleMeasurement()
{
  double car_length = 0.29;    // Distance between front and rear axles
  double wheelbase = 0.225;    // Width of axles, or distance between middle of rear wheels
  double wheel_width = 0.0445; // Width of wheel
  double car_width = wheelbase + wheel_width;
  double radius_rear_meas = car_length / std::tan(0.34) - car_width / 2.0;
  printf("diameter to measure (to outer edge of wheel) = %.4f (inches)\n",
         2.0 * (radius_rear_meas * 100.0 / 2.54)
        );
}

int main(int argc, char** argv)
{
  steeringAngleMeasurement();
  return 0;
}