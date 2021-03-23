#include <chrono>
#include <float.h>

#include "mcl/common.h"

using namespace localize;

// Misc
const std::string localize::DATA_PATH = "/home/dane/sw/ros/master/src/localize/mcl/data/";

// MCL
const unsigned int localize::MCL_NUM_PARTICLES_MIN = 1'000;
const unsigned int localize::MCL_NUM_PARTICLES_MAX = 100'000;
const double localize::MCL_KLD_EPS = 0.02;
const double localize::MCL_HIST_POS_RES = 0.10;
const double localize::MCL_HIST_TH_RES = M_PI / 18.0;

// Motion model
const double localize::MOTION_LIN_VEL_N1 = 0.01;
const double localize::MOTION_LIN_VEL_N2 = 0.01;
const double localize::MOTION_ANG_VEL_N1 = 0.01;
const double localize::MOTION_ANG_VEL_N2 = 0.01;
const double localize::MOTION_TH_N1 = 0.01;
const double localize::MOTION_TH_N2 = 0.01;

// Sensor model
const float localize::SENSOR_RANGE_NO_OBJ = 0.0;
const float localize::SENSOR_RANGE_STD_DEV = 0.5;       // 0.5
const float localize::SENSOR_NEW_OBJ_DECAY_RATE = 0.5;  // 0.5
const double localize::SENSOR_WEIGHT_NO_OBJ = 15.0;
const double localize::SENSOR_WEIGHT_NEW_OBJ = 5.0;     // 4.0
const double localize::SENSOR_WEIGHT_MAP_OBJ = 75.0;    // 80.0
const double localize::SENSOR_WEIGHT_RAND_EFFECT = 5.0; // 1.0
const double localize::SENSOR_UNCERTAINTY_FACTOR = 1.1;
const double localize::SENSOR_WEIGHT_RATIO_NEW_OBJ_THRESHOLD = 0.10;
                                                               /*(  SENSOR_WEIGHT_NEW_OBJ
                                                                / (  SENSOR_WEIGHT_NO_OBJ
                                                                   + SENSOR_WEIGHT_NEW_OBJ
                                                                   + SENSOR_WEIGHT_MAP_OBJ
                                                                   + SENSOR_WEIGHT_RAND_EFFECT
                                                                  )
                                                               );*/
const double localize::SENSOR_TABLE_RES = 0.01;
const unsigned int localize::TH_SAMPLE_COUNT = 8;
const unsigned int localize::TH_RAYCAST_COUNT = 656;

Particle::Particle(const double x,
                   const double y,
                   const double th,
                   const double weight,
                   const double weight_normed
                  ):
  x_(x),
  y_(y),
  th_(th),
  weight_(weight),
  weight_normed_(weight_normed),
  weights_(TH_SAMPLE_COUNT, 0.0)
{}

Ray::Ray(const float range,
         const float th
        ) :
  range_(range),
  th_(th)
{}

RaySample::RaySample(const float range,
                     const float th,
                     const double weight_new_obj_sum,
                     const double weight_sum
                    ) :
  Ray(range, th),
  weight_new_obj_sum_(weight_new_obj_sum),
  weight_sum_(weight_sum)
{}

RaySample::RaySample(const Ray& ray) :
  RaySample(ray.range_, ray.th_)
{}

void RaySample::operator=(const Ray& ray)
{
  range_ = ray.range_;
  th_ = ray.th_;

  return;
}

RayScan::RayScan(RayVector rays,
                 float th_inc,
                 float t_inc,
                 float t_dur
                ) :
  rays_(rays),
  th_inc_(th_inc),
  t_inc_(t_inc),
  t_dur_(t_dur)
{}

RayScan::RayScan(size_t num_rays) :
  RayScan()
{
  rays_.resize(num_rays);
}

// Note: This was derived from RangeLib author's definition of PyOMap in RangLibc.pyx
Map::Map(const unsigned int width,
         const unsigned int height,
         const float x_origin,
         const float y_origin,
         const float th_origin,
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
  this->x_origin = x_origin;
  this->y_origin = y_origin;
  this->th_origin = -th_origin;
  this->sin_th = std::sin(-th_origin);
  this->cos_th = std::cos(-th_origin);
  this->scale = scale;
}

bool Map::occupied(float x, float y) const
{
  rosWorldToGrid(x, y);

  return isOccupiedNT(x, y);
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