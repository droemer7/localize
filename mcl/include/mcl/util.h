#ifndef UTIL_H
#define UTIL_H

#include <algorithm>
#include <chrono>
#include <cmath>
#include <float.h>
#include <limits>
#include <math.h>
#include <random>
#include <sys/resource.h>
#include <type_traits>

#define L_PI 3.14159265358979323846
#define L_2PI 6.28318530717958647693

namespace localize
{
  template <class T> struct IndexedValue;
  template <class T> struct SmoothedValue;

  typedef IndexedValue<double> IndexedWeight;
  typedef std::vector<IndexedWeight> IndexedWeightVector;
  typedef SmoothedValue<double> SmoothedWeight;

  struct Greater
  {
    template <class T>
    bool operator()(const T& lhs, const T& rhs) const
    { return lhs > rhs; }
  };

  // RNG wrapper
  class RNG
  {
  public:
    // Constructor
    RNG()
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

    // A reference to the random number engine
    std::mt19937& engine()
    { return gen_; }

  private:
    std::mt19937 gen_;  // Generator for random numbers
  };

  // Normal distribution sampler
  template <class T>
  class NormalDistributionSampler
  {
  public:
    // Generates a random sample from a normal distribution specified by the mean and standard deviation
    T gen(const T mean, const T std_dev)
    {
      typename std::normal_distribution<T>::param_type params(mean, std_dev);
      return distribution_(rng_.engine(), params);
    }

  private:
    RNG rng_;                                   // Random number engine
    std::normal_distribution<T> distribution_;  // Normal distribution from which to sample
  };

  template <class T>
  struct IndexedValue
  {
    static_assert(std::is_integral<T>::value || std::is_floating_point<T>::value,
                  "IndexedValue: T must be an integral or floating point type"
                 );
    // Constructors
    IndexedValue() :
      val_(0),
      index_(0)
    {}

    explicit IndexedValue(const T val) :
      IndexedValue()
    { val_ = val; }

    IndexedValue(const T val, const size_t index) :
      IndexedValue()
    {
      val_ = val;
      index_ = index;
    }

    int compare(const IndexedValue& lhs, const IndexedValue& rhs) const
    {
      int result;
      if (lhs.val_ < rhs.val_) {
        result = -1;
      }
      else if (lhs.val_ > rhs.val_) {
        result = 1;
      }
      else { // lhs.val_ == rhs.val_
        result = 0;
      }
      return result;
    }

    // Operators
    bool operator==(const IndexedValue& rhs) const
    { return compare(*this, rhs) == 0; }

    bool operator!=(const IndexedValue& rhs) const
    { return compare(*this, rhs) != 0; }

    bool operator<(const IndexedValue& rhs) const
    { return compare(*this, rhs) < 0; }

    bool operator>(const IndexedValue& rhs) const
    { return compare(*this, rhs) > 0; }

    bool operator<=(const IndexedValue& rhs) const
    { return compare(*this, rhs) <= 0; }

    bool operator>=(const IndexedValue& rhs) const
    { return compare(*this, rhs) >= 0; }

    operator T() const
    { return val_; }

    T val_;
    size_t index_;
  };

  // Applies exponential smoothing to a changing value
  template <class T>
  class SmoothedValue
  {
    static_assert(std::is_floating_point<T>::value, "SmoothedValue: T must be a floating point type");

  public:
    // Constructors
    explicit SmoothedValue(const double rate) :
      rate_(rate),
      val_(0.0),
      initialized_(false)
    {}

    SmoothedValue(const double rate, const T val):
      SmoothedValue(rate)
    {
      update(val);
    }

    int compare(const SmoothedValue& lhs, const SmoothedValue& rhs) const
    {
      int result;
      if (lhs.val_ < rhs.val_) {
        result = -1;
      }
      else if (lhs.val_ > rhs.val_) {
        result = 1;
      }
      else { // lhs.val_ == rhs.val_
        result = 0;
      }
      return result;
    }

    // Operators
    bool operator==(const SmoothedValue& rhs) const
    { return compare(*this, rhs) == 0; }

    bool operator!=(const SmoothedValue& rhs) const
    { return compare(*this, rhs) != 0; }

    bool operator<(const SmoothedValue& rhs) const
    { return compare(*this, rhs) < 0; }

    bool operator>(const SmoothedValue& rhs) const
    { return compare(*this, rhs) > 0; }

    bool operator<=(const SmoothedValue& rhs) const
    { return compare(*this, rhs) <= 0; }

    bool operator>=(const SmoothedValue& rhs) const
    { return compare(*this, rhs) >= 0; }

    operator T() const
    { return val_; }

    // Update the value
    void update(const T val)
    {
      val_ = initialized_ ? val_ + rate_ * (val - val_) :
                            val;
      initialized_ = true;
      return;
    }

    // Resets the internal state so the next update will set the initial value
    void reset()
    { initialized_ = false; }

    // Resets the value to the one specified
    void reset(const T val)
    {
      reset();
      update(val);
    }

  private:
    double rate_;
    T val_;
    bool initialized_;
  };

  // Approximate equality check for floating point values
  // The Art of Comptuer Programming, Volume 2, Seminumeric Algorithms (Knuth 1997)
  // Section 4.2.2. Equation 22
  inline bool approxEqual(const float a, const float b, const float epsilon)
  { return std::abs(a - b) <= epsilon * std::max(std::abs(a), std::abs(b)); }

  inline bool approxEqual(const double a, const double b, const double epsilon)
  { return std::abs(a - b) <= epsilon * std::max(std::abs(a), std::abs(b)); }

  // Wrap an angle to (-pi, pi] (angle of -pi should convert to +pi)
  inline double wrapAngle(double angle)
  {
    if (angle > L_PI) {
      angle = std::fmod(angle, L_2PI);

      if (angle > L_PI) {
        angle -= L_2PI;
      }
    }
    else if (angle <= -L_PI) {
      angle = std::fmod(angle, -L_2PI);

      if (angle <= -L_PI) {
        angle += L_2PI;
      }
    }
    return angle;
  }

  // Unwrap an angle < 0.0 or > 2pi to [0.0, 2pi) (angle of -0.0 should convert to <2pi)
  inline double unwrapAngle(double angle)
  {
    while (angle < 0.0) {
      angle += L_2PI;
    }
    while (angle > L_2PI) {
      angle -= L_2PI;
    }
    return angle;
  }

  // Calculates the delta from angle 1 to angle 2 (rad), in the direction that is shortest
  // For example (in degrees):
  //   +175, +150 => -25
  //   -160, -150 => +10
  //    170, -150 => +40
  //    230, -10  => +120
  //   -179,  175 => -6
  inline double angleDelta(double angle_1, double angle_2)
  {
    // Make sure angles are in a common coordinate space
    angle_1 = wrapAngle(angle_1);
    angle_2 = wrapAngle(angle_2);

    int angle_1_sign = std::signbit(angle_1);
    int angle_2_sign = std::signbit(angle_2);

    // If angles are the same sign, angle 2 - angle 1 is the correct delta
    double delta_1_to_2 = angle_2 - angle_1;

    // If angle 1 > 0 and angle 1 - angle 2 is more than a half circle, handle rollover
    if (   angle_1_sign == 0
        && angle_1 - angle_2 > L_PI
       ) {
      delta_1_to_2 += L_2PI;
    }
    // If angle 2 > 0 and angle 2 - angle 1 is more than a half circle, handle rollover
    else if (   angle_2_sign == 0
             && angle_2 - angle_1 > L_PI
            ) {
      delta_1_to_2 -= L_2PI;
    }
    return delta_1_to_2;
  }

  inline void printMemoryUsage()
  {
    rusage stackusage;
    getrusage(RUSAGE_SELF, &stackusage);
    printf("MCL: Memory usage = %ld MB\n", stackusage.ru_maxrss / 1024);
  }

} // namespace localize

#endif // UTIL_H