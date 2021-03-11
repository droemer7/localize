#ifndef UTIL_H
#define UTIL_H

#include <cmath>
#include <iomanip>
#include <float.h>
#include <fstream>
#include <mutex>
#include <random>
#include <string>
#include <vector>

#include "includes/RangeLib.h"

namespace localize
{
  typedef std::mutex Mutex;
  typedef std::recursive_mutex RecursiveMutex;
  typedef std::lock_guard<std::mutex> Lock;
  typedef std::lock_guard<std::recursive_mutex> RecursiveLock;

  extern const std::string DATA_PATH;

  // A particle with 2D location, heading angle and weight
  struct Particle
  {
    // Constructors
    explicit Particle(const double x = 0.0,     // X position (meters)
                      const double y = 0.0,     // Y position (meters)
                      const double th = 0.0,    // Heading angle (rad)
                      const double weight = 0.0 // Importance weight
                     );

    double x_;      // X position (meters)
    double y_;      // Y position (meters)
    double th_;     // Heading angle (rad)
    double weight_; // Importance weight
  };

  typedef std::vector<Particle> ParticleVector;

  struct ParticleDistribution
  {
    ParticleDistribution(const ParticleVector particles,    // Particles
                         const double weight_avg_slow_rate, // Particle weight average slow smoothing rate
                         const double weight_avg_fast_rate, // Particle weight average fast smoothing rate
                         const double weight_avg = 0.0,     // Particle weight average
                         const double weight_sum = 0.0      // Particle weight sum
                        );

    ParticleVector particles_;    // Particles
    double num_particles_;        // Number of particles (this is <= particles.size())
    double weight_avg_slow_rate_; // Particle weight average slow smoothing rate
    double weight_avg_fast_rate_; // Particle weight average fast smoothing rate
    double weight_avg_;           // Particle weight average
    double weight_avg_slow_;      // Particle weight average, smoothed at slow rate
    double weight_avg_fast_;      // Particle weight average, smoothed at fast rate
    double weight_sum_;           // Particle weight sum
  };

  // A range sensor ray with range and angle
  struct Ray
  {
    // Constructors
    explicit Ray(const float range = 0.0, // Range (meters)
                 const float th = 0.0     // Angle (rad)
                );

    float range_; // Range (meters)
    float th_;    // Angle (rad)
  };

  typedef std::vector<Ray> RayVector;

  // Rayscan data
  struct RayScan
  {
    // Constructors
    explicit RayScan(RayVector rays = RayVector(),  // Vector of rays data
                     float th_inc = 0.0,            // Angle increment between ray elements (rad per index)
                     float t_inc = 0.0,             // Time increment between ray elements (sec per index)
                     float t_dur = 0.0              // Scan duration (sec)
                    );

    explicit RayScan(size_t num_rays);

    RayVector rays_;  // Vector of rays data
    float th_inc_;    // Angle increment between ray elements (rad per index)
    float t_inc_;     // Time increment between ray elements (sec per index)
    float t_dur_;     // Scan duration (sec)
  };

  // Map class constructed with ROS coordinate space conversion parameters
  // Used by RangeLib
  class Map : public ranges::OMap
  {
  public:
    // Constructor
    Map(const unsigned int width,       // Number of pixels along x axis
        const unsigned int height,      // Number of pixels along y axis
        const float x_origin,           // X translation of origin (cell 0,0) relative to world frame (meters)
        const float y_origin,           // Y translation of origin (cell 0,0) relative to world frame (meters)
        const float th_origin,          // Angle relative to world frame (rad)
        const float scale,              // Scale relative to world frame (meters per pixel)
        const std::vector<int8_t> data  // Occupancy data in 1D vector, -1: Unknown, 0: Free, 100: Occupied
       );

    inline bool occupied(float x, float y) const
    {
      rosWorldToGrid(x, y);
      return isOccupiedNT(x, y);
    }
  };

  // 3D boolean histogram representing occupancy of pose space (x, y, th)
  class ParticleHistogram
  {
  public:
    // Constructors
    ParticleHistogram(const double x_res,   // Resolution for x position (meters per cell)
                      const double y_res,   // Resolution for y position (meters per cell)
                      const double th_res,  // Resolution for angle (rad per cell)
                      const Map& map        // Map
                     );

    // Update occupancy with the particle's location if its
    // weight is larger than the minimum required threshold
    bool update(const Particle& particle);

    // Resets all cell occupancy counts
    void reset();

  private:
    const double x_res_;      // Resolution for x position (meters per cell)
    const double y_res_;      // Resolution for y position (meters per cell)
    const double th_res_;     // Resolution for heading angle (rad per cell)
    const size_t x_size_;     // Size of x dimension (number of elements)
    const size_t y_size_;     // Size of y dimension (number of elements)
    const size_t th_size_;    // Size of angular dimension (number of elements)
    const double x_origin_;   // X translation of origin (cell 0,0) relative to world frame (meters)
    const double y_origin_;   // Y translation of origin (cell 0,0) relative to world frame (meters)
    const double th_origin_;  // Angle relative to world frame (rad)
    std::vector<std::vector<std::vector<bool>>> hist_;  // Histogram
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

  // Approximate equality check for floating point values
  // For MCL
  // The Art of Comptuer Programming, Volume 2, Seminumeric Algorithms (Knuth 1997)
  // Section 4.2.2. Equation 22
  inline bool approxEqual(const float a,
                          const float b,
                          const float epsilon
                         )
  { return std::abs(a - b) <= epsilon * std::max(std::abs(a), std::abs(b)); }

  inline bool approxEqual(const double a,
                          const double b,
                          const double epsilon
                         )
  { return std::abs(a - b) <= epsilon * std::max(std::abs(a), std::abs(b)); }

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

  // Saves data to file in CSV format
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

  // Saves data to file in CSV format
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

  // Save data to file in CSV format
  inline void save(const ParticleVector& particles,
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

    for (const Particle& particle : particles) {
      output << particle.x_ << ","
             << particle.y_ << ","
             << particle.th_ * 180.0 / M_PI << ","
             << particle.weight_ << ","
             << "\n";
    }
    output.close();
  }

  // Save data to file in CSV format
  inline void save(const RayVector& rays,
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

  // Compare poses by x
  inline bool compX(const Particle& p1, const Particle& p2)
  { return (p1.x_ < p2.x_); };

  // Compare poses by y
  inline bool compY(const Particle& p1, const Particle& p2)
  { return (p1.y_ < p2.y_); };

  // Compare poses by theta
  inline bool compTh(const Particle& p1, const Particle& p2)
  { return (p1.th_ < p2.th_); };

  // Compare poses by weight
  inline bool compWeight(const Particle& p1, const Particle& p2)
  { return (p1.weight_ < p2.weight_); };

  // Sort poses in descending value, default by weight
  inline void sort(ParticleVector& particles,
                   bool (*comp)(const Particle& p1, const Particle& p2) = compWeight
                  )
  { std::sort(particles.rbegin(), particles.rend(), comp); }

} // namespace localize

#endif // UTIL_H