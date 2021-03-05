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
  typedef std::lock_guard<std::mutex> Lock;
  typedef std::lock_guard<std::recursive_mutex> RecursiveLock;

  extern const std::string DATA_PATH;

  // A particle with 2D location, heading angle and weight
  struct Particle
  {
    explicit Particle(const double x = 0.0,
                      const double y = 0.0,
                      const double th = 0.0,
                      const double weight = 1.0
                     );

    double x_;
    double y_;
    double th_;
    double weight_;
  };

  typedef std::vector<Particle> ParticleVector;

  // A range sensor ray with range and angle
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

  typedef std::vector<Ray> RayVector;

  class LockedParticleVector
  {
  public:
    // Constructor
    LockedParticleVector(ParticleVector& particles,
                         std::recursive_mutex & mtx
                        ) :
      lock_(mtx),
      particles_(particles)
    {}

    // Operators
    Particle & operator[](size_t i) const
    { return particles_[i]; }

    ParticleVector & operator=(const ParticleVector & particles)
    { return particles_; }

    // Iterators
    ParticleVector::iterator begin() const
    { return particles_.begin(); }

    ParticleVector::iterator end() const
    { return particles_.end(); }

    ParticleVector::reverse_iterator rbegin() const
    { return particles_.rbegin(); }

    ParticleVector::reverse_iterator rend() const
    { return particles_.rend(); }

    // Operations / Attributes
    void reserve(size_t num) const
    { return particles_.reserve(num); }

    void resize(size_t num)
    { particles_.resize(num); }

    size_t size() const
    { return particles_.size(); }

  private:
    std::unique_lock<std::recursive_mutex> lock_;
    ParticleVector& particles_;
  };

  // 3D boolean histogram representing occupancy of pose space (x, y, th)
  // Current provides minimum
  class ParticleHistogram
  {
  public:
    // Constructors
    ParticleHistogram(const double x_len,     // Length of x dimension (meters)
                      const double y_len,     // Length of y dimension (meters)
                      const double th_len,    // Range of angular dimension (rad)
                      const double x_res,     // Resolution for x position (meters per cell)
                      const double y_res,     // Resolution for y position (meters per cell)
                      const double th_res,    // Resolution for angle (rad per cell)
                      const double weight_min // Minimum particle weight required for a histogram cell to be considered occupied
                     );

    // Update occupancy with the particle's location if its
    // weight is larger than the minimum required threshold
    bool update(const Particle& particle);

    // Resets all cell occupancy counts
    void reset();

  private:
    const double x_len_;      // Length of x dimension (meters)
    const double y_len_;      // Length of y dimension (meters)
    const double th_len_;     // Range of angular dimension (rad)
    const double x_res_;      // Resolution for x position (meters per cell)
    const double y_res_;      // Resolution for y position (meters per cell)
    const double th_res_;     // Resolution for heading angle (rad per cell)
    const double weight_min_; // Minimum particle weight required for a histogram cell to be considered occupied
    std::vector<std::vector<std::vector<bool>>> hist_;  // Histogram
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

  // Approximate equality check for floating point
  inline bool approxEqual(const double a,
                          const double b
                         )
  { return std::abs(a - b) <= FLT_EPSILON * std::abs(a); }

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
  inline bool compX(const Particle& p1,
                    const Particle& p2
                   )
  { return (p1.x_ < p2.x_); };

  // Compare poses by y
  inline bool compY(const Particle& p1,
                    const Particle& p2
                   )
  { return (p1.y_ < p2.y_); };

  // Compare poses by theta
  inline bool compTh(const Particle& p1,
                     const Particle& p2
                    )
  { return (p1.th_ < p2.th_); };

  // Compare poses by weight
  inline bool compWeight(const Particle& p1,
                         const Particle& p2
                        )
  { return (p1.weight_ < p2.weight_); };

  // Sort poses in descending value, default by weight
  inline void sort(ParticleVector& particles,
                   bool (*comp)(const Particle& p1, const Particle& p2) = compWeight
                  )
  { std::sort(particles.rbegin(), particles.rend(), comp); }

} // namespace localize

#endif // UTIL_H