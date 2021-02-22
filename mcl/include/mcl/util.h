#ifndef UTIL_H
#define UTIL_H

#include <cmath>
#include <float.h>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "includes/RangeLib.h"

namespace localize
{
  // A particle with 2D location and heading
  struct Pose
  {
    explicit Pose(const double x = 0.0,
                  const double y = 0.0,
                  const double th = 0.0
                 );

    double x_;
    double y_;
    double th_;
  };

  struct PoseWithWeight : Pose
  {
    explicit PoseWithWeight(const double x = 0.0,
                            const double y = 0.0,
                            const double th = 0.0,
                            const double weight = 0.0
                           ) :
      Pose(x, y, th),
      weight_(weight)
    {}

    double weight_;
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
       );

    inline bool isOccupied(int x, int y) const
    { return ranges::OMap::isOccupiedNT(y, x); }
  };

  // RNG wrapper to seed properly
  class RNG
  {
  public:
    // Constructor
    RNG();

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
              );

  private:
    RNG rng_;  // Random number engine
  };

  inline bool equal(const double a,
                    const double b
                   )
  { return std::abs(a - b) <= FLT_EPSILON; }

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

  // Saves a matrix as a CSV
  template <class T>
  void save(const std::vector<std::vector<T>>& matrix,
            const std::string filename,
            const bool overwrite = true
           )
  {
    std::ios_base::openmode mode;
    if (overwrite) {
      mode = std::ofstream::trunc;
    }
    else {
      mode = std::ofstream::ate;
    }
    std::ofstream output(filename, mode);

    for (const std::vector<T>& row : matrix) {
      for (const T& val : row) {
        output << std::fixed << std::setprecision(16) << val << ",";
      }
      output << "\n";
    }
    output.close();
  }

} // namespace localize

#endif // UTIL_H