#include <chrono>
#include <float.h>
#include <stdio.h>
#include <thread>

#include "mcl/mcl.h"
#include "mcl/sensor.h"
#include "mcl/util.h"

#include "includes/RangeLib.h"

double start = 0.5;

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

void normalize(ParticleVector& particles)
{
  double weight_sum = 0.0;
  for (size_t i = 0; i < particles.size(); ++i) {
    weight_sum += particles[i].weight_;
  }
  double normalizer = 1 / weight_sum;
  for (size_t i = 0; i < particles.size(); ++i) {
    particles[i].weight_ *= normalizer;
  }
}

ParticleVector create(size_t num_particles)
{
  ParticleVector particles;
  for (size_t i = 0; i < num_particles; ++i) {
    particles.push_back(Particle(i, i+1, i+2, 1.0));
  }
  normalize(particles);
  return particles;
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

void sample(size_t init_num_particles,
            size_t num_particles_min_,
            size_t num_particles_max_
           )
{
  ParticleVector particles_ = create(init_num_particles);
  ParticleVector samples_(num_particles_max_);

  print(particles_);

  size_t num_particles = init_num_particles;
  double sample_width = 0.0;
  double sum_target = 0.0;
  double sum_curr = 0.0;
  double weight_sum = 0.0;
  double num_particles_target = num_particles_max_;
  size_t s = 0;
  size_t p = 0;

  // Initialize target and current weight sums
  if (num_particles > 0) {
    sample_width = 1.0 / num_particles;
    sum_target = start * sample_width;
    sum_curr = particles_[p].weight_;
  }
  // Generate samples until we exceed both the minimum and target number of
  // samples, or reach the maximum number allowed
  while (   (   s < num_particles_target
             || s < num_particles_min_
            )
         && s < num_particles_max_
        ) {
    // Sample from the current distribution until the sampled set
    // size is equal to the current distribution size
    if (s < num_particles) {
      // Sum weights until we reach the target sum
      while (sum_curr < sum_target) {
        sum_curr += particles_[++p].weight_;
      }
      // Add to sample set and increase target sum
      samples_[s] = particles_[p];
      sum_target += sample_width;
    }
    // Generate a new random particle in free space
    else {
      samples_[s] = Particle(99, 99, 99, 1.0);
    }
    // Update weight sum
    weight_sum += samples_[s].weight_;

    /*KLD stuff*/

    ++s;
  }
  // Resize to the actual number of samples used
  samples_.resize(s);

  // Normalize weights
  double normalizer = 1 / weight_sum;
  for (size_t i = 0; i < samples_.size(); ++i) {
    samples_[i].weight_ *= normalizer;
  }
  particles_ = samples_;

  print(particles_);

  return;
}

void sampleOverwrite(size_t init_num_particles,
                     size_t num_particles_min_,
                     size_t num_particles_max_
                    )
{
  ParticleVector particles_ = create(init_num_particles);
  particles_.resize(num_particles_max_);

  print(particles_);

  size_t num_particles = init_num_particles;
  double sample_width = 0.0;
  double sum_target = 0.0;
  double sum_curr = 0.0;
  double weight_sum = 0.0;
  double num_particles_target = num_particles_max_;
  size_t s = 0;
  size_t p = 0;

  // Initialize target and current weight sums
  if (num_particles > 0) {
    sample_width = 1.0 / num_particles;
    sum_target = start * sample_width;
    sum_curr = particles_[p].weight_;
  }
  // Generate samples until we exceed both the minimum and target number of
  // samples, or reach the maximum number allowed
  while (   (   s < num_particles_target
             || s < num_particles_min_
            )
         && s < num_particles_max_
        ) {
    // Sample from the current distribution until the sampled set
    // size is equal to the current distribution size
    if (s < num_particles) {
      // Sum weights until we reach the target sum
      while (sum_curr < sum_target) {
        sum_curr += particles_[++p].weight_;
      }
      // Add to sample set and increase target sum
      printf("s, p = %lu, %lu\n", s, p);
      printf("sum_curr = %f\n", sum_curr);
      printf("sum_target = %f\n", sum_target);
      particles_[s] = particles_[p];
      sum_target += sample_width;
    }
    // Generate a new random particle in free space
    else {
      particles_[s] = Particle(99, 99, 99);
    }
    // Update weight sum
    weight_sum += particles_[s].weight_;

    /*KLD stuff*/

    ++s;
  }
  // Resize to the actual number of samples used
  particles_.resize(s);

  // Normalize weights
  double normalizer = 1 / weight_sum;
  for (size_t i = 0; i < particles_.size(); ++i) {
    particles_[i].weight_ *= normalizer;
  }

  print(particles_);

  return;
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

  sample(10, 15, 20);
  sampleOverwrite(10, 15, 20);

  return 0;
}