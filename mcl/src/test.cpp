#include <chrono>
#include <float.h>
#include <stdio.h>

#include "mcl/mcl.h"
#include "mcl/sensor.h"
#include "mcl/util.h"

#include "includes/RangeLib.h"

static const double SENSOR_RANGE_MIN = 0.0;
static const double SENSOR_RANGE_MAX = 10.0;
static const double SENSOR_RANGE_NO_OBJ = 0.0;
static const double SENSOR_RANGE_STD_DEV = 0.01;
static const double SENSOR_ANGLE_SAMPLE_RES = 45 * M_PI / 180.0;
static const double SENSOR_NEW_OBJ_DECAY_RATE = 0.5;
static const double SENSOR_WEIGHT_NO_OBJ = 25.0;
static const double SENSOR_WEIGHT_NEW_OBJ = 15.0;
static const double SENSOR_WEIGHT_MAP_OBJ = 55.0;
static const double SENSOR_WEIGHT_RAND_EFFECT = 5.0;
static const double SENSOR_UNCERTAINTY_FACTOR = 0.95;
static const size_t SENSOR_TABLE_SIZE = 1000;

using namespace localize;

template <class T>
void testSampleNormalDist(const unsigned int samples,
                          const unsigned int repeats
                         )
{
  printf("\nTesting NormalDistributionSampler ... \n");
  NormalDistributionSampler<T> sampler;
  std::chrono::_V2::high_resolution_clock::time_point start;
  std::chrono::_V2::high_resolution_clock::time_point end;
  std::chrono::duration<double> dur;
  T mean = 0;
  T std_dev = 0;
  unsigned int count = 0;

  for (unsigned int i = 0; i < repeats; ++i) {
    mean = (i + 1) % 10;
    std_dev = (i + 1) % 10;
    count = 0;
    start = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    for (unsigned int j = 0; j < samples; ++j) {
      if (std::abs(sampler.gen(mean, std_dev) - mean) > 2.0 * std_dev)
      {
        ++count;
      }
    }
    end = std::chrono::high_resolution_clock::now();
    dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    printf("%u samples generated in %.2f ms with %.2f%% outside of 2 standard deviations\n",
           samples, dur.count() * 1000, 100.0 * count / samples
          );
  }
  printf("--- Test complete ---\n");
}

void testVectorReserve(size_t iterations,
                       const bool reserve_enable
                      )
{
  std::chrono::_V2::high_resolution_clock::time_point start;
  std::chrono::_V2::high_resolution_clock::time_point end;
  std::chrono::duration<double> dur;
  double dur_max = 0.0;
  double dur_tot = 0.0;

  PoseWithWeight pose(1.0, 2.0, 3.0, 4.0);
  std::vector<PoseWithWeight> v;

  if (reserve_enable) {
    v.resize(iterations);
  }

  printf("v.size() = %lu\n", v.size());
  printf("v.capacity() = %lu\n", v.capacity());

  std::vector<double> dur_v(iterations);
  for (size_t i = 0; i < iterations; ++i) {
    start = std::chrono::high_resolution_clock::now();
    v.push_back(pose);
    end = std::chrono::high_resolution_clock::now();
    dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
    dur_v[i] = dur.count() * 1000.0;
    dur_max = dur_v[i] > dur_max? dur_v[i] : dur_max;
    dur_tot += dur_v[i];
  }
  printf("Max = %f ms\n", dur_max);
  printf("Total = %f ms\n", dur_tot);
  printf("--- Test complete ---\n");
}

void testHist()
{
  std::vector<std::vector<std::vector<int>>> hist(5, std::vector<std::vector<int>>(10, std::vector<int>(3, 0)));
  int w = 0;
  for (size_t i = 0; i < hist.size(); ++i) {
    for (size_t j = 0; j < hist[0].size(); ++j) {
      for (size_t k = 0; k < hist[0][0].size(); ++k) {
        hist[i][j][k] = w++;
      }
    }
  }

  for (size_t i = 0; i < hist.size(); ++i) {
    for (size_t j = 0; j < hist[0].size(); ++j) {
      for (size_t k = 0; k < hist[0][0].size(); ++k) {
        printf("hist[%lu][%lu][%lu] = %d\n", i, j, k, hist[i][j][k]);
      }
    }
  }
}

void testAngleWrapping(const size_t num_angles,
                       const double angle_inc
                      )
{
  double angle = 0.0;

  for (size_t i = 0; i < num_angles; ++i) {
    angle = i * angle_inc;
    printf("Wrap(%f) = %f\n",
           angle * 180.0 / M_PI,
           wrapAngle(angle) * 180.0 / M_PI
          );
  }
}

void normalize(double weight_sum)
{
  std::vector<double> weights = {1, 2, 3, 4, 5};
  if (weight_sum <= 0.0) {
    weight_sum = 0.0;
    for (size_t i = 0; i < weights.size(); ++i) {
      weight_sum += weights[i];
    }
  }
  double normalizer = 1 / weight_sum;
  for (size_t i = 0; i < weights.size(); ++i) {
    weights[i] *= normalizer;
  }
  for (size_t i = 0; i < weights.size(); ++i) {
    printf("weight[%lu] = %f\n", i, weights[i]);
  }
}

void normalize()
{
  normalize(0.0);
}

int main(int argc, char** argv)
{
  // unsigned int iterations = 3 * 10000;
  // unsigned int repeats = 5;
  // testSampleNormalDist<float>(iterations, repeats);
  // testSampleNormalDist<double>(iterations, repeats);
  // testVectorReserve(10000, true);
  // testVectorReserve(10000, false);
  // testHist();
  // testAngleWrapping(20, 45 * M_PI / 180.0);

  normalize();

  return 0;
}