#ifndef UTIL_H
#define UTIL_H

#include <cmath>
#include <iomanip>
#include <float.h>
#include <fstream>
#include <random>
#include <string>
#include <vector>

#include "includes/RangeLib.h"

namespace localize
{
  extern const std::string DATA_PATH;

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
                            const double weight = 1.0
                           ) :
      Pose(x, y, th),
      weight_(weight)
    {}

    double weight_;
  };

  struct Ray
  {
    explicit Ray(const float range = 0.0,
                 const float th = 0.0
                ) :
      range_(range),
      th_(th)
    {}

    float range_;
    float th_;
  };

  // Map class constructed with ROS coordinate space conversion parameters
  // Used by RangeLib
  class Map : public ranges::OMap
  {
  public:
    // Constructor
    Map(const unsigned int width,       // Number of pixels along x axis
        const unsigned int height,      // Number of pixels along y axis
        const float x,                  // X translation of origin (cell 0,0) relative to world frame
        const float y,                  // Y translation of origin (cell 0,0) relative to world frame
        const float th,                 // Angle relative to world frame
        const float scale,              // Scale relative to world frame (meters per pixel)
        const std::vector<int8_t> data  // Occupancy data in 1D vector, -1: Unknown, 0: Free, 100: Occupied
       );

    inline bool isOccupied(float x, float y) const
    {
      rosWorldToGrid(x, y);
      return isOccupiedNT(x, y);
    }
  };

  // RNG wrapper to seed properly
  class RNG
  {
  public:
    // Constructor
    RNG();

    // A reference to the random number engine
    std::mt19937& engine()
    { return gen_; }

  private:
    std::mt19937 gen_;  // Generator for random numbers
  };

  // Generate random samples from a normal distribution
  template <class T=double>
  class NormalDistributionSampler
  {
  public:
    // Generates a random sample from a normal distribution specified by the
    // mean and standard deviation
    T gen(const T mean,
          const T std_dev
         )
    {
      typename std::normal_distribution<T>::param_type params(mean, std_dev);
      return distribution_(rng_.engine(), params);
    }

  private:
    RNG rng_;                                   // Random number engine
    std::normal_distribution<T> distribution_;  // Normal distribution from which to sample
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

  // Saves a matrix to file in CSV format
  template <class T>
  void save(const std::vector<std::vector<T>>& data_matrix,
            const std::string filename,
            const unsigned int precision = 0,
            const bool overwrite = true
           )
  {
    std::ofstream output(DATA_PATH + filename,
                         overwrite ? std::ofstream::trunc :
                                     std::ofstream::ate
                        );
    if (precision > 0) {
      output << std::fixed << std::setprecision(precision);
    }
    for (const std::vector<T>& row : data_matrix) {
      for (const T& val : row) {
        output << static_cast<double>(val) << ",";
      }
      output << "\n";
    }
    output.close();
  }

  template <class T>
  void save(const std::vector<T>& data_vector,
            const std::string filename,
            size_t num_cols = 0,
            const unsigned int precision = 0,
            const bool overwrite = true
           )
  {
    size_t num_rows;

    if (num_cols > 0) {
      num_rows = std::ceil(data_vector.size() / num_cols);
    }
    else {
      num_rows = 1;
      num_cols = data_vector.size();
    }
    std::vector<std::vector<T>> data_matrix;

    for (size_t r = 0; r < num_rows; ++r) {
      std::vector<T> row;

      for (size_t c = 0; c < num_cols; ++c) {
        row.push_back(data_vector[r * num_cols + c]);
      }
      data_matrix.push_back(row);
    }
    save(data_matrix,
         filename,
         precision,
         overwrite
        );
  }

  inline void save(const std::vector<PoseWithWeight>& particles,
                   const std::string filename,
                   const unsigned int precision = 0,
                   const bool overwrite = true
                  )
  {
    std::ofstream output(DATA_PATH + filename,
                         overwrite ? std::ofstream::trunc :
                                     std::ofstream::ate
                        );
    if (precision > 0) {
      output << std::fixed << std::setprecision(precision);
    }
    output << "Particles\n";
    output << "x,y,theta (deg),weight\n";

    for (const PoseWithWeight& particle : particles) {
      output << particle.x_ << ","
             << particle.y_ << ","
             << particle.th_ * 180.0 / M_PI << ","
             << particle.weight_ << ","
             << "\n";
    }
    output.close();
  }

  inline void save(const std::vector<Ray>& rays,
                   const std::string filename,
                   const unsigned int precision = 0,
                   const bool overwrite = true
                  )
  {
    std::ofstream output(DATA_PATH + filename,
                         overwrite ? std::ofstream::trunc :
                                     std::ofstream::ate
                        );
    if (precision > 0) {
      output << std::fixed << std::setprecision(precision);
    }
    output << "Rays\n";
    output << "range,theta (deg)\n";
    for (const Ray& ray : rays) {
      output << ray.range_ << ","
             << ray.th_ * 180.0 / M_PI << ","
             << "\n";
    }
    output.close();
  }

  inline bool compX(const PoseWithWeight& p1,
                    const PoseWithWeight& p2
                   )
  { return (p1.x_ < p2.x_); };

  inline bool compY(const PoseWithWeight& p1,
                    const PoseWithWeight& p2
                   )
  { return (p1.y_ < p2.y_); };

  inline bool compTh(const PoseWithWeight& p1,
                     const PoseWithWeight& p2
                    )
  { return (p1.th_ < p2.th_); };

  inline bool compWeight(const PoseWithWeight& p1,
                         const PoseWithWeight& p2
                        )
  { return (p1.weight_ < p2.weight_); };

  inline void sort(std::vector<PoseWithWeight>& particles,
                   bool (*comp)(const PoseWithWeight& p1, const PoseWithWeight& p2)
                  )
  { std::sort(particles.rbegin(), particles.rend(), comp); }

} // namespace localize

#endif // UTIL_H