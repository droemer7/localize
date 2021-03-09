#include <chrono>
#include <float.h>
#include <stdio.h>
#include <thread>

#include "mcl/mcl.h"
#include "mcl/sensor.h"
#include "mcl/util.h"

#include "includes/RangeLib.h"

using namespace localize;

double temp;
RNG rng;
std::uniform_real_distribution<double> x_dist(-10.0, std::nextafter(10.0, DBL_MAX));
std::uniform_real_distribution<double> y_dist(-10.0, std::nextafter(10.0, DBL_MAX));
std::uniform_real_distribution<double> th_dist(-M_PI, M_PI);
std::chrono::_V2::high_resolution_clock::time_point start;
std::chrono::_V2::high_resolution_clock::time_point end;

ParticleVector particles;

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

void print(ParticleVector& particles)
{
  for (size_t i = 0; i < particles.size(); ++i) {
    printf("sample[%lu] = (%f, %f, %f, %f)\n",
           i, particles[i].x_, particles[i].y_, particles[i].th_, particles[i].weight_
          );
  }
  printf("\n");
}

void gen(Particle& particle)
{
  //Particle particle;

  particle.x_ = x_dist(rng.engine());
  particle.y_ = y_dist(rng.engine());
  particle.th_ = th_dist(rng.engine());

  return;// particle;
}

void testGen(size_t num_samples)
{
  printf("\nTesting gen() ... \n");
  particles.resize(num_samples);

  start = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < particles.size(); ++i) {
    gen(particles[i]);
  }
  end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  printf("Generated %lu samples in %.2f ms\n", num_samples, dur.count() * 1000.0);
  printf("--- Test complete ---\n");
}

void testSqrt(size_t num_samples)
{
  printf("\nTesting std::sqrt() ... \n");
  start = std::chrono::high_resolution_clock::now();
  for (size_t i = 2; i < num_samples + 2; ++i) {
    temp += std::sqrt((2 / 9) * (i - 1.0));
  }
  end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  printf("Calculated %lu sqrt in %.2f ms\n", num_samples, dur.count() * 1000.0);
  printf("--- Test complete ---\n");
}

int main(int argc, char** argv)
{
  // unsigned int iterations = 3 * 10000;
  // unsigned int repeats = 5;
  // testSampleNormalDist<float>(iterations, repeats);
  // testSampleNormalDist<double>(iterations, repeats);
  // testAngleWrapping(20, 45 * M_PI / 180.0);
  // testGen(200'000);

  testSqrt(200'000);

  return 0;
}