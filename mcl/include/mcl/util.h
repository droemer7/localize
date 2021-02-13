#ifndef UTIL_H
#define UTIL_H

#include <chrono>
#include <cmath>
#include <float.h>
#include <random>

#include <ros/ros.h>

namespace localize
{
  // A particle with 2D location and heading
  struct Particle
  {
    explicit Particle(const double x = 0.0,
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

  }; // struct Particle

  // Generate random samples from a normal distribution
  class NormalDistributionSampler
  {
  public:
    // Constructor
    NormalDistributionSampler()
    {
      // Initialize PRNG (source: https://stackoverflow.com/a/13446015)
      std::random_device rng_dev;
      std::chrono::_V2::system_clock::duration time = std::chrono::_V2::system_clock::now().time_since_epoch();
      std::mt19937::result_type time_seconds = std::chrono::duration_cast<std::chrono::seconds>(time).count();
      std::mt19937::result_type time_microseconds = std::chrono::duration_cast<std::chrono::microseconds>(time).count();
      std::mt19937::result_type rng_seed = rng_dev() ^ (time_seconds + time_microseconds);
      rng_gen_.seed(rng_seed);
    }

    // Generates a random sample from a normal distribution
    // Algorithm 5.4 in Probabilistic Robotics (Thrun 2006, page 124)
    double gen(const double std_dev, const double mean = 0.0)
    {
      // Generate a uniform distribution within the range [-sigma, sigma]
      std::uniform_real_distribution<double> rng_dist(-std_dev, std::nextafter(std_dev, DBL_MAX));
      double sum = 0.0;

      for (unsigned int i = 0; i < 12; ++i) {
        sum += rng_dist(rng_gen_);
      }
      return (sum / 2) + mean;

    } // function gen()

  private:
    std::mt19937 rng_gen_;  // Generator for random numbers

  }; // class NormalDistributionSampler

  // Retrieves the desired parameter value from the ROS parameter server
  template <class T>
  bool getParam(const ros::NodeHandle& nh, std::string name, T& var)
  {
    bool result = true;

    if (!nh.getParam(name, var)) {
      ROS_FATAL("MCL: Parameter '%s' not found", name.c_str());
      result = false;
    }
    return result;

  }; // function getParam()

} // namespace localize

#endif // UTIL_H