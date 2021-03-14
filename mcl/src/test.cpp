#include <chrono>
#include <float.h>
#include <stdio.h>
#include <thread>

#include "mcl/mcl.h"
#include "mcl/sensor.h"
#include "mcl/util.h"

#include "includes/RangeLib.h"

using namespace localize;

std::chrono::_V2::high_resolution_clock::time_point start;
std::chrono::_V2::high_resolution_clock::time_point end;
const size_t x_size = 500;
const size_t y_size = 500;
const size_t th_size = 500;

void testParticleHistogramArray()
{
  printf("\nTesting ParticleHistogram (array) ... \n");
  //typedef std::array<std::array<std::array<unsigned char, x_size>, y_size>, th_size> Histogram;
  typedef std::array<unsigned char, x_size * y_size * th_size> Histogram;
  std::unique_ptr<Histogram> hist_ptr = std::unique_ptr<Histogram>(new Histogram);

  start = std::chrono::high_resolution_clock::now();
  std::fill((*hist_ptr).begin(), (*hist_ptr).end(), false);
  end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  printf("Reset histogram in %.2f ms\n", dur.count() * 1000.0);
  printf("--- Test complete ---\n");
}

void testParticleHistogramVector3D()
{
  printf("\nTesting ParticleHistogram (3D vector) ... \n");
  std::vector<std::vector<std::vector<unsigned char>>> hist(x_size,
                                                            std::vector<std::vector<unsigned char>>(y_size,
                                                            std::vector<unsigned char>(th_size, false)
                                                           ));
  start = std::chrono::high_resolution_clock::now();
  for (size_t i = 0; i < hist.size(); ++i) {
    for (size_t j = 0; j < hist[0].size(); ++j) {
      std::fill(hist[i][j].begin(), hist[i][j].end(), false);
    }
  }
  end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  printf("Reset histogram in %.2f ms\n", dur.count() * 1000.0);
  printf("--- Test complete ---\n");
}

void testParticleHistogramArray1D()
{
  printf("\nTesting ParticleHistogram (1D array) ... \n");
  std::array<bool, x_size * y_size * th_size> hist;

  start = std::chrono::high_resolution_clock::now();
  std::fill(hist.begin(), hist.end(), true);
  end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);

  // for (size_t i = 0; i < hist.size(); ++i) {
  //   printf("%d, ", static_cast<bool>(hist[i]));
  // }
  printf("Reset histogram in %.2f ms\n", dur.count() * 1000.0);
  printf("--- Test complete ---\n");
}

void testParticleHistogramVector1D()
{
  printf("\nTesting ParticleHistogram (1D vector) ... \n");
  std::vector<bool> hist(x_size * y_size * th_size, false);

  start = std::chrono::high_resolution_clock::now();
  std::fill(hist.begin(), hist.end(), true);
  end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);

  // for (size_t i = 0; i < hist.size(); ++i) {
  //   printf("%d, ", static_cast<bool>(hist[i]));
  // }
  printf("Reset histogram in %.2f ms\n", dur.count() * 1000.0);
  printf("--- Test complete ---\n");
}

void test3Dto1DVector()
{
  printf("\nTesting 3D to 1D vector ... \n");
  size_t x_len = 4;
  size_t y_len = 6;
  size_t th_len = 3;
  std::vector<std::vector<std::vector<int>>> vector3d(x_len,
                                                      std::vector<std::vector<int>>(y_len,
                                                      std::vector<int>(th_len, false)
                                                     ));
  std::vector<bool> vector1d(x_len * y_len * th_len);

  int v = 0;
  printf("vector3d = ");
  for (size_t i = 0; i < vector3d.size(); ++i) {
    for (size_t j = 0; j < vector3d[0].size(); ++j) {
      for (size_t k = 0; k < vector3d[0][0].size(); ++k) {
        printf("%d, ", v);
        vector3d[i][j][k] = ++v % 2;
      }
    }
  }
  printf("\n");
  v = 0;
  printf("vector1d = ");
  for (size_t i = 0; i < vector1d.size(); ++i) {
    printf("%d, ", v);
    vector1d[i] = ++v % 2;
  }
  printf("\n");

  printf("vector1d = ");
  for (size_t i = 0; i < vector3d.size(); ++i) {
    for (size_t j = 0; j < vector3d[0].size(); ++j) {
      for (size_t k = 0; k < vector3d[0][0].size(); ++k) {
        printf("%d, ", static_cast<bool>(vector1d[i * y_len * th_len + j * th_len + k]));
      }
    }
  }
  printf("\n");
}

void resize(ParticleVector& particles, const size_t count)
{
  start = std::chrono::high_resolution_clock::now();
  particles.resize(count);
  end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  printf("Resized to %lu in %.2f ms\n", count, dur.count() * 1000.0);
}

void testResize(const size_t count)
{
  ParticleVector particles;
  particles.reserve(count);

  printf("\nTesting resize... \n");
  for (size_t i = 0; i < 20; ++i) {
    if (i % 2) {
      resize(particles, count);
    }
    else{
      resize(particles, 0);
    }
  }
  printf("--- Test complete ---\n");
}

struct Dist
{
  Dist(const size_t num_particles_max) :
    particles_(num_particles_max)
  {
    particles_.reserve(num_particles_max);
  }
  void update(ParticleVector& particles)
  {
    particles_.swap(particles);
  }
  void update(const ParticleVector& particles, const size_t num_particles)
  {
    for (size_t i = 0; i < num_particles; ++i) {
      particles_[i] = particles[i];
    }
  }
  ParticleVector particles_;
};

int main(int argc, char** argv)
{
  // testParticleHistogramArray();
  // testParticleHistogramVector3D();
  // testParticleHistogramArray1D();
  // testParticleHistogramVector1D();
  // test3Dto1DVector();
  testResize(100'000);

  double val = 0.0;
  const size_t num_particles_max = 100'000;
  Dist dist(num_particles_max - num_particles_max / 10);
  ParticleVector samples(num_particles_max);

  samples.reserve(num_particles_max);
  for (auto & sample : samples) {
    sample.x_ = val + 1.1;
    sample.y_ = val + 3.25;
    sample.th_ = val + 25.99;
    sample.weight_ = val + 9e-5;
    val += 1.0;
  }
  printf("Size = %lu, particles[0] (before swap): %f, %f, %f, %f\n",
         dist.particles_.size(),
         dist.particles_[0].x_,
         dist.particles_[0].y_,
         dist.particles_[0].th_,
         dist.particles_[0].weight_
        );
  start = std::chrono::high_resolution_clock::now();
  samples.resize(num_particles_max);
  //dist.update(samples, 0.9 * num_particles_max);
  end = std::chrono::high_resolution_clock::now();

  std::chrono::duration<double> dur = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
  printf("Size = %lu, particles[0] (after swap): %f, %f, %f, %f\n",
         dist.particles_.size(),
         dist.particles_[0].x_,
         dist.particles_[0].y_,
         dist.particles_[0].th_,
         dist.particles_[0].weight_
        );
  printf("Updated distribution (vector) in %.2f ms\n", dur.count() * 1000.0);

  return 0;
}