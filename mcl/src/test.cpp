// #include <iostream>
// #include <utility>
// #include <chrono>

// using namespace std::chrono;
// //const size_t length = 23'068'672;
// const size_t length = 5'000'000;

// typedef high_resolution_clock::duration dur_t;
// std::pair<dur_t, dur_t> stack()
// {
//         auto start = high_resolution_clock::now();
//         volatile int eathalfastack[length] = {};
//         auto mid = high_resolution_clock::now();
//         for( size_t i = 0; i < length; ++i ){
//                 eathalfastack[i] = i;
//         }
//         auto end = high_resolution_clock::now();
//     return {mid-start, end-mid};
// }

// std::pair<dur_t, dur_t> heap()
// {
//         auto start = high_resolution_clock::now();
//         volatile int* heaparr = new volatile int[length]();
//         auto mid = high_resolution_clock::now();
//         for( size_t i = 0; i < length ; i++ ){
//                 heaparr[i] = i;
//         }
//         auto end = high_resolution_clock::now();
//         delete[] heaparr;
//         return make_pair(mid - start, end - mid);
// }

// int main()
// {
//     dur_t stack_alloc, stack_write, heap_alloc, heap_write;
//     for(int cnt = 0; cnt < 100; ++cnt)
//     {
//         std::pair<dur_t, dur_t> timing = stack();
//         stack_alloc += timing.first;
//         stack_write += timing.second;
//         timing = heap();
//         heap_alloc += timing.first;
//         heap_write += timing.second;
//     }

//         std::cout << "Time taken in ms:\n"
//               << "stack alloc: " << duration_cast<milliseconds>(stack_alloc).count() << "ms\n"
//               << "stack write: " << duration_cast<milliseconds>(stack_write).count() << "ms\n"
//               << "Heap  alloc: " << duration_cast<milliseconds>(heap_alloc).count() << "ms\n"
//               << "Heap  write: " << duration_cast<milliseconds>(heap_write).count() << "ms\n";
// }

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

void testParticleHistogram()
{
  printf("\nTesting ParticleHistogram ... \n");
  std::vector<std::vector<std::vector<int>>> hist_(400,
                                                    std::vector<std::vector<int>>(400,
                                                    std::vector<int>(72, false)
                                                   ));
  size_t count = 0;
  start = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < hist_.size(); ++i) {
    for (size_t j = 0; j < hist_[0].size(); ++j) {
      for (size_t k = 0; k < hist_[0][0].size(); ++k) {
        hist_[i][j][k] = false;
      }
    }
  }
  count = 0;
  end = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  printf("Reset histogram in %.2f ms\n", dur.count() * 1000.0);
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

  testParticleHistogram();

  // ParticleVector particles(10'000'000);
  // start = std::chrono::high_resolution_clock::now();
  // for (size_t i = 0; i < particles.size(); ++i) {
  //   particles[i].weight_ = 2.0;
  // }
  // end = std::chrono::high_resolution_clock::now();
  // std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  // printf("Done: %.2f ms\n", dur.count() * 1000.0);
  // printf("--- Test complete ---\n");

  return 0;
}