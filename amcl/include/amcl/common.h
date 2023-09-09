#ifndef COMMON_H
#define COMMON_H

#include <assert.h>
#include <cmath>
#include <iomanip>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#include "amcl/util.h"

namespace localize
{
  class Point;
  class Pose;
  class Particle;
  class Ray;
  class RaySample;
  class RayScan;

  typedef std::mutex Mutex;
  typedef std::recursive_mutex RecursiveMutex;
  typedef std::lock_guard<std::mutex> Lock;
  typedef std::lock_guard<std::recursive_mutex> RecursiveLock;

  typedef std::vector<Point> PointVector;
  typedef std::vector<Pose> PoseVector;
  typedef std::vector<Particle> ParticleVector;
  typedef std::vector<Ray> RayVector;
  typedef std::vector<RaySample> RaySampleVector;
  typedef std::vector<RayScan> RayScanVector;

  extern const int NUM_ESTIMATES;             // Number of pose estimates to provide
  extern const int SENSOR_NUM_RAYS_PER_SCAN;  // Number of rays per scan expected (count per revolution)
  extern const int SENSOR_RAY_SAMPLE_COUNT;   // Number of ray samples to use per scan (count per revolution)
  extern const std::string DATA_PATH;         // Full path to directory for saving data

  // 2D point (x, y)
  struct Point
  {
    explicit Point(const double x = 0.0, // X position
                   const double y = 0.0  // Y position
                  );

    double x_;  // X position
    double y_;  // Y position
  };

  // 2D point (x, y) and angle of orientation
  struct Pose : public Point
  {
    explicit Pose(const Point& point,   // Point (x, y)
                  const double th = 0.0 // Angle
                 );

    explicit Pose(const double x = 0.0, // X position
                  const double y = 0.0, // Y position
                  const double th = 0.0 // Angle
                 );

    double th_; // Angle
  };

  // Weighted pose estimate
  struct Particle : public Pose
  {
    // Constructors
    explicit Particle(const Pose& pose,                 // Pose (x, y, angle)
                      const double weight = 0.0,        // Importance weight
                      const double weight_normed = 0.0  // Normalized importance weight
                     );

    explicit Particle(const double x = 0.0,             // X position
                      const double y = 0.0,             // Y position
                      const double th = 0.0,            // Angle
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

    double weight_;               // Importance weight (not normalized)
    double weight_normed_;        // Importance weight (normalized)
    std::vector<double> weights_; // Partial importance weights (not normalized) listed by sample angle index
  };

  // A range sensor ray with range and angle
  struct Ray
  {
    // Constructors
    explicit Ray(const float range = 0.0, // Range
                 const float th = 0.0     // Angle
                );

    float range_; // Range
    float th_;    // Angle
  };

  // A ray with cumulative probabilities for p(new object) and p(all)
  // This class is used in range scanner based models for outlier rejection of short range measurements
  struct RaySample : Ray
  {
    // Constructors
    explicit RaySample(const float range = 0.0,               // Range
                       const float th = 0.0,                  // Angle
                       const double weight_new_obj_sum = 0.0, // Sum of weights across the distribution for this angle representing a new / unexpected object
                       const double weight_sum = 0.0          // Sum of weights across the distribution for this angle
                      );

    explicit RaySample(const Ray& ray);

    double weight_new_obj_sum_; // Sum of weights across the distribution for this angle representing a new / unexpected object
    double weight_sum_;         // Sum of weights across the distribution for this angle representing the full sensor model pdf
  };

  // Rayscan data
  struct RayScan
  {
    // Constructors
    explicit RayScan(const RayVector& rays = RayVector(), // Vector of rays data
                     float th_inc = 0.0,                  // Angle increment between ray elements (rad per index)
                     float t_inc = 0.0,                   // Time increment between ray elements (sec per index)
                     float t_dur = 0.0                    // Scan duration (sec)
                    );

    explicit RayScan(int num_rays);

    RayVector rays_;  // Vector of rays data
    float th_inc_;    // Angle increment between ray elements (angle per index)
    float t_inc_;     // Time increment between ray elements (sec per index)
    float t_dur_;     // Scan duration (sec)
  };

  // Occupancy grid map
  class Map
  {
  public:
    // Constructor
    Map(const int x_size,               // Length of x axis (width) (pixels)
        const int y_size,               // Length of y axis (height) (pixels)
        const double x_origin_world,    // X translation of origin (cell 0,0) relative to world frame (meters)
        const double y_origin_world,    // Y translation of origin (cell 0,0) relative to world frame (meters)
        const double th_world,          // Angle relative to world frame (rad)
        const double scale_world,       // Scale relative to world frame (meters per pixel)
        const std::vector<int8_t>& data // Occupancy data in 1D row-major order, -1: Unknown, 0: Free, 100: Occupied
       );

    // Return true if a point (in the map frame) is in an occupied space (out of bounds is considered occupied)
    bool occupied(const Point& point) const;

    // Length of x axis (pixels)
    int xSize() const;

    // Length of y axis (pixels)
    int ySize() const;

    // X translation of origin (cell 0,0) relative to world frame (meters)
    double xOriginWorld() const;

    // Y translation of origin (cell 0,0) relative to world frame (meters)
    double yOriginWorld() const;

    // Angle relative to world frame
    double thWorld() const;

    // Sin of angle relative to world frame
    double sinThWorld() const;

    // Cos of angle relative to world frame
    double cosThWorld() const;

    // Scale relative to world frame (meters per pixel)
    double scaleWorld() const;

    // Occupancy data in 1D row-major order
    const std::vector<bool>& data() const;

  private:
    const int x_size_;              // Length of x axis (pixels)
    const int y_size_;              // Length of y axis (pixels)
    const double x_origin_world_;   // X translation of origin (cell 0,0) relative to world frame (meters)
    const double y_origin_world_;   // Y translation of origin (cell 0,0) relative to world frame (meters)
    const double th_world_;         // Angle relative to world frame
    const double sin_th_world_;     // Sin of angle relative to world frame
    const double cos_th_world_;     // Cos of angle relative to world frame
    const double scale_world_;      // Scale relative to world frame (meters per pixel)
    const std::vector<bool> data_;  // Occupancy data in 1D row-major order, 0: Free, 1: Occupied
  };

  // Transform a point in the world frame to the map frame
  Point worldToMap(const Map& map, const Point& point_world);

  // Transform a pose in the world frame to the map frame
  Pose worldToMap(const Map& map, const Pose& pose_world);

  // Transform a point in the map frame to the world frame
  Point mapToWorld(const Map& map, const Point& point_map);

  // Transform a pose in the map frame to the world frame
  Pose mapToWorld(const Map& map, const Pose& pose_map);

  // A sampler which chooses a random pose in free space
  class PoseRandomSampler
  {
  public:
    // Constructors
    explicit PoseRandomSampler(const Map& map);

    // Generate a random pose in free space
    Pose operator()();

  private:
    const Map& map_; // Occupancy map

    RNG rng_; // Random number generator

    std::uniform_real_distribution<double> x_dist_;   // Distribution of x locations in map frame [0, width)
    std::uniform_real_distribution<double> y_dist_;   // Distribution of y locations in map frame [0, height)
    std::uniform_real_distribution<double> th_dist_;  // Distribution of theta in map frame (-pi, pi]
  };

  // Print particle state
  inline void printParticle(const Particle& particle, const std::string& name = "")
  {
    if (name != "") {
      printf("%s: ", name.c_str());
    }
    else {
      printf("Particle: ");
    }
    printf("%.2f, %.2f, %.2f (weight = %.3e, normed weight = %.3e)\n",
           particle.x_,
           particle.y_,
           particle.th_ * 180.0 / L_PI,
           particle.weight_,
           particle.weight_normed_
          );
  }

  // Save particles to file in CSV format
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

  // Save rays to file in CSV format
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