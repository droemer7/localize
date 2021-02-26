#include <chrono>
#include <float.h>

#include "mcl/util.h"

using namespace localize;

const std::string localize::DATA_PATH = "/home/dane/sw/ros/master/src/localize/mcl/data/";

Pose::Pose(const double x,
           const double y,
           const double th
          ):
  x_(x),
  y_(y),
  th_(th)
{}

// Note: This was derived from RangeLib author's definition of PyOMap in RangLibc.pyx
Map::Map(const unsigned int width,
         const unsigned int height,
         const float x,
         const float y,
         const float th,
         const float scale,
         const std::vector<int8_t> data
        ) :
  ranges::OMap(height, width)
{
  for (int i = 0; i < height; ++i) {
    for (int j = 0; j < width; ++j) {
      if (data[i * width + j] > 10) {
        grid[i][j] = true;
      }
    }
  }
  this->x = x;
  this->y = y;
  this->th = -th;
  this->sin_th = std::sin(-th);
  this->cos_th = std::cos(-th);
  this->scale = scale;
}

RNG::RNG()
{
  // Initialize random number generator
  // Source: https://stackoverflow.com/a/13446015
  std::random_device dev;
  std::chrono::_V2::system_clock::duration time = std::chrono::_V2::system_clock::now().time_since_epoch();
  std::mt19937::result_type time_seconds = std::chrono::duration_cast<std::chrono::seconds>(time).count();
  std::mt19937::result_type time_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time).count();
  std::mt19937::result_type seed = dev() ^ (time_seconds + time_microseconds);
  gen_.seed(seed);
}