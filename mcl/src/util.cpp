#include <chrono>
#include <float.h>

#include "mcl/util.h"

using namespace localize;

Pose::Pose(const double x,
           const double y,
           const double th
          ):
  x_(x),
  y_(y),
  th_(th)
{}

// Note: This was derived from RangeLib author's definition of PyOMap in RangLibc.pyx
Map::Map(const unsigned int map_width,
         const unsigned int map_height,
         const float map_m_per_pxl,
         const double map_th,
         const double map_origin_x,
         const double map_origin_y,
         const std::vector<int8_t> map_occ_data
        ) :
  ranges::OMap(map_height, map_width)
{
  for (int i = 0; i < map_height; ++i) {
    for (int j = 0; j < map_width; ++j) {
      if (map_occ_data[i * map_width + j] > 10) {
        grid[i][j] = true;
      }
    }
  }
  world_scale = map_m_per_pxl;
  world_angle = -map_th;
  world_origin_x = map_origin_x;
  world_origin_y = map_origin_y;
  world_sin_angle = std::sin(-map_th);
  world_cos_angle = std::cos(-map_th);
}

RNG::RNG()
{
  // Initialize random number generator
  // Source: https://stackoverflow.com/a/13446015
  std::random_device rng_dev;
  std::chrono::_V2::system_clock::duration time = std::chrono::_V2::system_clock::now().time_since_epoch();
  std::mt19937::result_type time_seconds = std::chrono::duration_cast<std::chrono::seconds>(time).count();
  std::mt19937::result_type time_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time).count();
  std::mt19937::result_type rng_seed = rng_dev() ^ (time_seconds + time_microseconds);
  rng_gen_.seed(rng_seed);
}

double NormalDistributionSampler::gen(const double std_dev,
                                      const double mean
                                     )
{
  // Generate a uniform distribution within the range [-sigma, sigma]
  std::uniform_real_distribution<double> uni_dist(-std_dev, std::nextafter(std_dev, DBL_MAX));
  double sum = 0.0;

  for (unsigned int i = 0; i < 12; ++i) {
    sum += uni_dist(rng_.engine());
  }
  return (sum / 2) + mean;
}