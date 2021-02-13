#ifndef MCL_H
#define MCL_H

#include <mutex>
#include <vector>

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <vesc_msgs/VescStateStamped.h>

#include "mcl/motion.h"
#include "mcl/sensor.h"
#include "mcl/util.h"

namespace localize
{
  // Monte-Carlo Localization
  // The basis of the implementation in this class is the _particle filter_,
  // a recursive Bayes filter which approximates the robot pose distribution
  // by a set of random samples drawn from its probability distribution.
  // The probability distribution is calculated by applying the motion model -
  // propagating particles forward in time based on control input(s) - and the
  // sensor model - determining the likelihood of a given particle based on the
  // sensor measurements obtained.
  class MCL
  {
  public:
    // Constructors
    MCL(const unsigned int num_particles,
        const VelModel motion_model,
        const BeamModel sensor_model
       );

    // Motor state callback
    void motorCb(const vesc_msgs::VescStateStamped::ConstPtr& msg);

    // Servo state callback
    void servoCb(const std_msgs::Float64::ConstPtr& msg);

    // Sensor scan callback
    void sensorCb(const sensor_msgs::LaserScan::ConstPtr& msg);

  private:
    VelModel motion_model_;   // Motion model
    BeamModel sensor_model_;  // Sensor model

    std::vector<Particle> particles_; // Particle set distribution
    std::vector<double> weights_;     // Particle importance weights
    double servo_pos_;                // Last Servo (steering) position

    ros::Time curr_t_;  // Current time
    ros::Time prev_t_;  // Previous time

    std::mutex motor_mtx_;  // Motor state callback lock
    std::mutex servo_mtx_;  // Servo state callback lock
    std::mutex sensor_mtx_; // Sensor scan callback lock

  }; // class MCL

} // namespace localize

#endif // MCL_H
