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

int main(int argc, char** argv)
{
  unsigned int iterations = 3 * 10000;
  unsigned int repeats = 5;
  testSampleNormalDist<float>(iterations, repeats);
  testSampleNormalDist<double>(iterations, repeats);

  return 0;
}