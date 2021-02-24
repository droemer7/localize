#include <chrono>
#include <float.h>
#include <stdio.h>

#include "mcl/mcl.h"
#include "mcl/sensor.h"
#include "mcl/util.h"

#include "includes/RangeLib.h"

int a, b, c, d, i = 0;

static const double SENSOR_RANGE_MIN = 0.0;
static const double SENSOR_RANGE_MAX = 10.0;
static const double SENSOR_RANGE_NO_OBJ = 0.0;
static const double SENSOR_RANGE_STD_DEV = 0.01;
static const double SENSOR_ANGLE_SAMPLE_INC = 45 * M_PI / 180.0;
static const double SENSOR_NEW_OBJ_DECAY_RATE = 0.5;
static const double SENSOR_WEIGHT_NO_OBJ = 25.0;
static const double SENSOR_WEIGHT_NEW_OBJ = 15.0;
static const double SENSOR_WEIGHT_MAP_OBJ = 55.0;
static const double SENSOR_WEIGHT_RAND_EFFECT = 5.0;
static const double SENSOR_UNCERTAINTY_FACTOR = 0.95;
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
    val = sampler.gen(std_dev);
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
  Map map(10, 10, 0.1, 0.0, 0.0, 0.0, std::vector<int8_t>(10 * 10));
  BeamModel sensorModel(SENSOR_RANGE_MIN,
                        SENSOR_RANGE_MAX,
                        SENSOR_RANGE_NO_OBJ,
                        SENSOR_RANGE_STD_DEV,
                        SENSOR_ANGLE_SAMPLE_INC,
                        SENSOR_NEW_OBJ_DECAY_RATE,
                        SENSOR_WEIGHT_NO_OBJ,
                        SENSOR_WEIGHT_NEW_OBJ,
                        SENSOR_WEIGHT_MAP_OBJ,
                        SENSOR_WEIGHT_RAND_EFFECT,
                        SENSOR_UNCERTAINTY_FACTOR,
                        SENSOR_TABLE_SIZE,
                        map
                       );
  std::vector<double> error_levels = {0.0, 0.0001, 0.1, 1, 2, 3, 5.25, 10.98};
  for (double& error_level : error_levels)
  {
    double error = std_dev * error_level;
    printf("p(error = %f-sigma) = %f\n", error_level, sensorModel.calcProbMapObj(error + mean, mean));
  }
  printf("--- Test complete ---\n");
}

void testProbRandEffect(std::vector<float> ranges)
{
  printf("\nTesting BeamModel::probRandEffect():\n");
  Map map(10, 10, 0.1, 0.0, 0.0, 0.0, std::vector<int8_t>(10 * 10));
  BeamModel sensorModel(SENSOR_RANGE_MIN,
                        SENSOR_RANGE_MAX,
                        SENSOR_RANGE_NO_OBJ,
                        SENSOR_RANGE_STD_DEV,
                        SENSOR_ANGLE_SAMPLE_INC,
                        SENSOR_NEW_OBJ_DECAY_RATE,
                        SENSOR_WEIGHT_NO_OBJ,
                        SENSOR_WEIGHT_NEW_OBJ,
                        SENSOR_WEIGHT_MAP_OBJ,
                        SENSOR_WEIGHT_RAND_EFFECT,
                        SENSOR_UNCERTAINTY_FACTOR,
                        SENSOR_TABLE_SIZE,
                        map
                       );
  std::vector<double> probs = std::vector<double>(ranges.size(), 0.0);

  for (size_t i = 0; i < ranges.size(); ++i)
  {
    probs[i] = sensorModel.calcProbRandEffect(ranges[i]);
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

void sensorFunctionVal(std::vector<double> v1,
                       std::vector<double> v2,
                       std::vector<double> v3,
                       std::vector<double> v4
                      )
{
  a = v1[i];
  b = v2[i];
  c = v3[i];
  d = v4[i];
}

void sensorFunctionRef(std::vector<double>& v1,
                       std::vector<double>& v2,
                       std::vector<double>& v3,
                       std::vector<double>& v4
                      )
{
  a = v1[i];
  b = v2[i];
  c = v3[i];
  d = v4[i];
}

void singleCopyVal(double v1,
                   double v2,
                   double v3,
                   double v4
                  )
{
  a = v1;
  b = v2;
  c = v3;
  d = v4;
}

void singleCopyRef(double& v1,
                   double& v2,
                   double& v3,
                   double& v4
                  )
{
  a = v1;
  b = v2;
  c = v3;
  d = v4;
}

void testSensorFunctionCopyVal(size_t size, size_t iterations)
{
  std::vector<double> v1(size, a);
  std::vector<double> v2(size, b);
  std::vector<double> v3(size, c);
  std::vector<double> v4(size, d);
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; ++i) {
    sensorFunctionVal(v1, v2, v3, v4);
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  printf("sensorFunctionVal():\n");
  printf("Array size = %lu\n", size);
  printf("Iterations = %lu\n", iterations);
  printf("Duration = %.10f\n", dur.count());
  printf("------------------------------------\n");
}

void testSensorFunctionCopyRef(size_t size, size_t iterations)
{
  std::vector<double> v1(size, a);
  std::vector<double> v2(size, b);
  std::vector<double> v3(size, c);
  std::vector<double> v4(size, d);
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; ++i) {
    sensorFunctionRef(v1, v2, v3, v4);
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  printf("sensorFunctionRef():\n");
  printf("Array size = %lu\n", size);
  printf("Iterations = %lu\n", iterations);
  printf("Duration = %.10f\n", dur.count());
  printf("------------------------------------\n");
}

void testSingleCopyVal(size_t iterations)
{
  double v1 = a;
  double v2 = b;
  double v3 = c;
  double v4 = d;
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; ++i) {
    singleCopyVal(v1, v2, v3, v4);
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  printf("singleCopyVal():\n");
  printf("Iterations = %lu\n", iterations);
  printf("Duration = %.10f\n", dur.count());
  printf("------------------------------------\n");
}

void testSingleCopyRef(size_t iterations)
{
  double v1 = a;
  double v2 = b;
  double v3 = c;
  double v4 = d;
  auto start = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; ++i) {
    singleCopyRef(v1, v2, v3, v4);
  }
  auto end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  printf("singleCopyRef():\n");
  printf("Iterations = %lu\n", iterations);
  printf("Duration = %.10f\n", dur.count());
  printf("------------------------------------\n");
}

double ifElse(int x)
{
  if (a > x) {
    return a - 1;
  }
  else {
    return a + 1;
  }
}

double noIfElse(int x)
{
  return x - 1;
}

void testIfElse(size_t iterations)
{
  auto startIf = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; ++i) {
    b = ifElse(i);
  }
  auto endIf = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> durIf = std::chrono::duration_cast<std::chrono::duration<double>>(endIf - startIf);

  auto startNoIf = std::chrono::high_resolution_clock::now();
  for (int i = 0; i < iterations; ++i) {
    b = noIfElse(i);
  }
  auto endNoIf = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> durNoIf = std::chrono::duration_cast<std::chrono::duration<double>>(endNoIf - startNoIf);

  printf("ifElse():\n");
  printf("Iterations = %lu\n", iterations);
  printf("Duration ifElse()= %.10f\n", durIf.count());
  printf("Duration noIfElse()= %.10f\n", durNoIf.count());
  printf("------------------------------------\n");
}

int main(int argc, char** argv)
{
  // Test NormalDistributionSampler
  /*
  unsigned int num_std_devs = 2;
  unsigned int num_samples = std::pow(10.0, static_cast<double>(num_std_devs + 3));
  double std_dev = 1.0;
  testSampleNormalDist(num_samples,
                       num_std_devs,
                       std_dev
                      );
  */

  // Test BeamModel::probRandEffect()
  /*
  std::vector<float> ranges = {0.0, FLT_MIN, 1.0, 2.0, 5.00023125, 9.999999, 10.0};
  for (size_t i = 0; i < ranges.size(); ++i)
  {
    testProbRandEffect(ranges);
  }
  */

  // Test BeamModel::save()
  /*
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
  */

  // Test floating point
  /*
  double test_d = 980.123456789123456789;
  float test_f = 980.123456789123456789;
  printf("double = %.20f\n", test_d);
  printf("float = %.20f\n", test_f);
  printf("comparison = %d\n", std::abs((float)0.10001f - (double)0.1) <= FLT_EPSILON);
  printf("FLT_EPSILON = %.20f\n", FLT_EPSILON);
  */

  // Test profiling functions
  // testSensorFunctionCopyVal(100000, 1);
  // testSensorFunctionCopyRef(100000, 1);
  // testSingleCopyVal(100000);
  // testSingleCopyRef(100000);
  // testIfElse(1000000);

  printf("Size after rounding = %.10f (float), %d (int)\n",
         std::round(M_2PI / SENSOR_ANGLE_SAMPLE_INC),
         static_cast<int>(std::round(M_2PI / SENSOR_ANGLE_SAMPLE_INC))
        );
  printf("7 * angle increment =  %.10f", 7.0 * SENSOR_ANGLE_SAMPLE_INC * 180.0 / M_PI);

  return 0;
}