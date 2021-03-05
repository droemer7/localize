#include <chrono>
#include <float.h>
#include <stdio.h>
#include <thread>

#include "mcl/mcl.h"
#include "mcl/sensor.h"
#include "mcl/util.h"

#include "includes/RangeLib.h"

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

  Particle pose(1.0, 2.0, 3.0, 4.0);
  ParticleVector v;

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

int main(int argc, char** argv)
{
  // unsigned int iterations = 3 * 10000;
  // unsigned int repeats = 5;
  // testSampleNormalDist<float>(iterations, repeats);
  // testSampleNormalDist<double>(iterations, repeats);
  // testVectorReserve(10000, true);
  // testVectorReserve(10000, false);
  // testAngleWrapping(20, 45 * M_PI / 180.0);

  // std::shared_ptr<std::vector<PoseWithWeight>> particles_ptr(new std::vector<PoseWithWeight>(5));
  // std::mutex mtx;
  // Particles particles(mtx, particles_ptr);
  // particles[0].x_ = 55;
  // printf("x = %f\n", (*particles_ptr)[0].x_);

  std::vector<int> v = {1, 2, 3};
  {
    std::vector<int>& vref = v;
    vref[0] = 4;
    vref[1] = 5;
    vref[2] = 6;
  }
  for (size_t i = 0; i < v.size(); ++i) {
    printf("v[%lu] = %d\n", i, v[i]);
  }

  return 0;
}