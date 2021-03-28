#ifndef COMMON_H
#define COMMON_H

#include <cmath>
#include <iomanip>
#include <float.h>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include "mcl/util.h"

#include "includes/RangeLib.h"

namespace localize
{
  typedef std::mutex Mutex;
  typedef std::recursive_mutex RecursiveMutex;
  typedef std::lock_guard<std::mutex> Lock;
  typedef std::lock_guard<std::recursive_mutex> RecursiveLock;

  extern const unsigned int SENSOR_TH_SAMPLE_COUNT; // Number of samples per ray scan (count per revolution)
  extern const std::string DATA_PATH;               // Full path to directory for saving data

  // A particle with 2D location, heading angle and weight
  struct Particle
  {
    // Constructors
    explicit Particle(const double x = 0.0,             // X position (meters)
                      const double y = 0.0,             // Y position (meters)
                      const double th = 0.0,            // Heading angle (rad)
                      const double weight = 0.0,        // Importance weight
                      const double weight_normed = 0.0  // Normalized importance weight
                     );

    // Compare particles by weight
    int compare(const Particle& lhs, const Particle& rhs) const;

    // Operators
    bool operator==(const Particle& rhs) const
    { return compare(*this, rhs) == 0; }

    bool operator!=(const Particle& rhs) const
    { return compare(*this, rhs) != 0; }

    bool operator<(const Particle& rhs) const
    { return compare(*this, rhs) < 0; }

    bool operator>(const Particle& rhs) const
    { return compare(*this, rhs) > 0; }

    bool operator<=(const Particle& rhs) const
    { return compare(*this, rhs) <= 0; }

    bool operator>=(const Particle& rhs) const
    { return compare(*this, rhs) >= 0; }

    double x_;              // X position (meters)
    double y_;              // Y position (meters)
    double th_;             // Heading angle (rad)
    double weight_;         // Importance weight (not normalized)
    double weight_normed_;  // Normalized importance weight
    std::vector<double> weights_; // Partial importance weights listed by sample angle index
  };

  typedef std::vector<Particle> ParticleVector;

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

  // A ray with cumulative probabilities for p(new object) and p(all)
  // This class is used in range scanner based models for outlier rejection of short range measurements
  struct RaySample : Ray
  {
    // Constructors
    explicit RaySample(const float range = 0.0,               // Range (meters)
                       const float th = 0.0,                  // Angle (rad)
                       const double weight_new_obj_sum = 0.0, // Sum of weights across the distribution for this angle representing a new / unexpected object
                       const double weight_sum = 0.0          // Sum of weights across the distribution for this angle
                      );

    explicit RaySample(const Ray& ray);

    void operator=(const Ray& ray);

    double weight_new_obj_sum_; // Sum of weights across the distribution for this angle representing a new / unexpected object
    double weight_sum_;         // Sum of weights across the distribution for this angle
  };

  typedef std::vector<RaySample> RaySampleVector;

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

  typedef std::vector<RayScan> RayScanVector;

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

    bool occupied(float x, float y) const;
  };

  // A sampler which chooses a random particle in free space
  class ParticleRandomSampler
  {
  public:
    // Constructors
    explicit ParticleRandomSampler(const Map& map);

    // Generates a random particle in free space
    Particle operator()();

  private:
    const Map& map_;  // Occupancy map

    RNG rng_; // Random number generator

    std::uniform_real_distribution<double> x_dist_;   // Distribution of map x locations relative to world frame
    std::uniform_real_distribution<double> y_dist_;   // Distribution of map y locations relative to world frame
    std::uniform_real_distribution<double> th_dist_;  // Distribution of theta [-pi, +pi) relative to world frame
  };


  // Saves data to file in CSV format
  template <class T>
  void save(const std::vector<std::vector<T>>& data_matrix,
            const std::string filename,
            const bool overwrite = true
           )
  {
    std::ofstream output(DATA_PATH + filename,
                         overwrite ? std::ofstream::trunc :
                                     std::ofstream::app
                        );
    for (const std::vector<T>& row : data_matrix) {
      for (const T& val : row) {
        output << static_cast<double>(val) << ", ";
      }
      output << "\n";
    }
    output << "\n";
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
                   const bool overwrite = true
                  )
  {
    std::ofstream output(DATA_PATH + filename,
                         overwrite ? std::ofstream::trunc :
                                     std::ofstream::app
                        );
    output << "Particles\n";
    output << "x, y, theta (deg), weight, normalized weight\n";

    for (const Particle& particle : particles) {
      output << std::fixed << std::setprecision(3)
             << particle.x_ << ", "
             << particle.y_ << ", "
             << particle.th_ * 180.0 / L_PI << ", "
             << std::scientific << std::setprecision(4)
             << particle.weight_ << ", "
             << particle.weight_normed_ << ", " << "\n";
    }
    output << "\n";
    output.close();
  }

  // Save data to file in CSV format
  inline void save(const std::vector<RaySample>& rays,
                   const std::string filename,
                   const bool overwrite = true
                  )
  {
    std::ofstream output(DATA_PATH + filename,
                         overwrite ? std::ofstream::trunc :
                                     std::ofstream::app
                        );
    output << "Rays\n";
    output << "range, theta (deg)\n";
    for (const RaySample& ray : rays) {
      output << std::fixed << std::setprecision(3) << ray.range_ << ", " << ray.th_ * 180.0 / L_PI << ", " << "\n";
    }
    output << "\n";
    output.close();
  }

} // namespace localize

#endif // COMMON_H