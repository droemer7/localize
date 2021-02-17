#ifndef UTIL_H
#define UTIL_H

#include <chrono>
#include <cmath>
#include <float.h>
#include <random>

#include <ros/ros.h>

#include "includes/RangeLib.h"

namespace localize
{
  // A particle with 2D location and heading
  struct Pose
  {
    explicit Pose(const double x = 0.0,
                  const double y = 0.0,
                  const double th = 0.0
                 ):
      x_(x),
      y_(y),
      th_(th)
    {}

    double x_;
    double y_;
    double th_;
  };

  // Map class constructed with ROS coordinate space conversion parameters
  // Used by RangeLib
  class Map : public ranges::OMap
  {
  public:
    // Constructor
    Map(const unsigned int map_width,           // Map width
        const unsigned int map_height,          // Map height
        const float map_m_per_pxl,              // Map resolution (meters per pixel)
        const double map_th,                    // Map angle
        const double map_origin_x,              // Map origin x position
        const double map_origin_y,              // Map origin y position
        const std::vector<int8_t> map_occ_data  // Map occupancy data in 1D vector, -1: Unknown, 0: Free, 100: Occupied
       ) :
      ranges::OMap(map_height, map_width) // Swap width and height because OMap uses a different coordinate space
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

    bool isOccupied(int x, int y) const
    { return ranges::OMap::isOccupiedNT(y, x); }
  };

  // RNG wrapper to seed properly
  class RNG
  {
  public:
    // Constructor
    RNG()
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

    // A reference to the random number engine
    std::mt19937& engine()
    { return rng_gen_; }

  private:
    std::mt19937 rng_gen_;  // Generator for random numbers
  };

  // Generate random samples from a normal distribution
  class NormalDistributionSampler
  {
  public:
    // Generates a random sample from a normal distribution
    // Algorithm 5.4 in Probabilistic Robotics (Thrun 2006, page 124)
    double gen(const double std_dev,
               const double mean = 0.0
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

  private:
    RNG rng_;  // Random number engine
  };

  // Retrieves the desired parameter value from the ROS parameter server
  template <class T>
  inline bool getParam(const ros::NodeHandle& nh,
                       std::string name,
                       T& value
                      )
  {
    bool result = true;

    if (!nh.getParam(name, value)) {
      ROS_FATAL("MCL: Parameter '%s' not found", name.c_str());
      result = false;
    }
    return result;
  }

  // Wrap an angle to (-pi, pi] (angle of -pi should convert to +pi)
  inline double wrapAngle(double angle)
  {
    if (angle > M_PI) {
      angle = std::fmod(angle, M_2PI);
      if (angle > M_PI) {
        angle -= M_2PI;
      }
    }
    else if (angle <= -M_PI) {
      angle = std::fmod(angle, -M_2PI);
      if (angle <= -M_PI) {
        angle += M_2PI;
      }
    }
    return angle;
  }

} // namespace localize

#endif // UTIL_H