#include <stdio.h>

#include "mcl/mcl.h"
#include "mcl/sensor.h"
#include "mcl/util.h"

static const double SENSOR_RANGE_MIN = 0.0;
static const double SENSOR_RANGE_MAX = 10.0;
static const double SENSOR_RANGE_NO_OBJ = 0.0;
static const double SENSOR_RANGE_STD_DEV = 0.01;
static const double SENSOR_NEW_OBJ_DECAY_RATE = 0.5;
static const double SENSOR_WEIGHT_NO_OBJ = 25.0;
static const double SENSOR_WEIGHT_NEW_OBJ = 15.0;
static const double SENSOR_WEIGHT_MAP_OBJ = 55.0;
static const double SENSOR_WEIGHT_RAND_EFFECT = 5.0;
static const size_t SENSOR_TABLE_SIZE = 1000;

using namespace localize;

void testSampleNormalDist(const unsigned int num_samples,
                          const unsigned int num_std_devs,
                          const double std_dev
                         )
{
  printf("\nTesting NormalDistributionSampler:\n");
  NormalDistributionSampler sampler;

  double val = 0.0;
  int count = 0;
  for (int i = 0; i < num_samples; ++i)
  {
    val = sampler(std_dev);
    if (std::abs(val) > std_dev * num_std_devs)
    {
      ++count;
    }
  }
  double percent_out = 100.0 * count / num_samples;
  printf("Generated %d samples\n", num_samples);
  printf("%.20f%% samples were outside %d-sigma\n", percent_out, num_std_devs);
  printf("--- Test complete ---\n");
}

void testProbExpectedObj(const double std_dev,
                         const double mean
                        )
{
  printf("\nTesting BeamModel::probExpectedObj():\n");
  BeamModel sensorModel(SENSOR_RANGE_MIN,
                        SENSOR_RANGE_MAX,
                        SENSOR_RANGE_NO_OBJ,
                        SENSOR_RANGE_STD_DEV,
                        SENSOR_NEW_OBJ_DECAY_RATE,
                        SENSOR_WEIGHT_NO_OBJ,
                        SENSOR_WEIGHT_NEW_OBJ,
                        SENSOR_WEIGHT_MAP_OBJ,
                        SENSOR_WEIGHT_RAND_EFFECT,
                        SENSOR_TABLE_SIZE
                       );
  std::vector<double> error_levels = {0.0, 0.0001, 0.1, 1, 2, 3, 5.25, 10.98};
  for (double& error_level : error_levels)
  {
    double error = std_dev * error_level;
    printf("p(error = %f-sigma) = %f\n", error_level, sensorModel.probMapObj(error + mean, mean));
  }
  printf("--- Test complete ---\n");
}

void testProbRandEffect(std::vector<float> ranges)
{
  printf("\nTesting BeamModel::probRandEffect():\n");
  BeamModel sensorModel(SENSOR_RANGE_MIN,
                        SENSOR_RANGE_MAX,
                        SENSOR_RANGE_NO_OBJ,
                        SENSOR_RANGE_STD_DEV,
                        SENSOR_NEW_OBJ_DECAY_RATE,
                        SENSOR_WEIGHT_NO_OBJ,
                        SENSOR_WEIGHT_NEW_OBJ,
                        SENSOR_WEIGHT_MAP_OBJ,
                        SENSOR_WEIGHT_RAND_EFFECT,
                        SENSOR_TABLE_SIZE
                       );
  std::vector<double> probs = std::vector<double>(ranges.size(), 0.0);

  for (size_t i = 0; i < ranges.size(); ++i)
  {
    probs[i] = sensorModel.probRandEffect(ranges[i]);
    printf("p(range = %f) = %.20f\n", ranges[i], probs[i]);
  }
  printf("--- Test complete ---\n");
}

void testArray(float * ins, int num_ins)
{
  printf("\n");
  for(int i = 0; i < num_ins; ++i)
  {
    printf("ins[%d] = %f\n", i, ins[i]);
  }
  printf("--- Test complete ---\n");
}

int main(int argc, char** argv)
{
  // unsigned int num_std_devs = 2;
  // unsigned int num_samples = std::pow(10.0, static_cast<double>(num_std_devs + 3));
  // double std_dev = 1.0;
  // testSampleNormalDist(num_samples,
  //                      num_std_devs,
  //                      std_dev
  //                     );

  // std::vector<float> ranges = {0.0, FLT_MIN, 1.0, 2.0, 5.00023125, 9.999999, 10.0};
  // for (size_t i = 0; i < ranges.size(); ++i)
  // {
  //   testProbRandEffect(ranges);
  // }

  std::string filename = "/home/dane/doc/sensor_model.csv";
  printf("Loading sensor model ... ");
  BeamModel sensorModel(SENSOR_RANGE_MIN,
                        SENSOR_RANGE_MAX,
                        SENSOR_RANGE_NO_OBJ,
                        SENSOR_RANGE_STD_DEV,
                        SENSOR_NEW_OBJ_DECAY_RATE,
                        SENSOR_WEIGHT_NO_OBJ,
                        SENSOR_WEIGHT_NEW_OBJ,
                        SENSOR_WEIGHT_MAP_OBJ,
                        SENSOR_WEIGHT_RAND_EFFECT,
                        SENSOR_TABLE_SIZE
                       );
  printf("done\n");
  printf("Saving sensor model to '%s' ... ", filename.c_str());
  sensorModel.save(filename);
  printf("done\n");

  return 0;
}