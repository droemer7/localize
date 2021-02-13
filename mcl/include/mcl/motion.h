#ifndef MOTION_H
#define MOTION_H

#include <vector>

#include "mcl/util.h"

namespace localize
{
  // Velocity-based probabilistic model for an Ackermann drive robot
  //
  // The model assumes the robot is driven with linear and angular velocity
  // commands and the resulting motion obeys a circular motion kinematic
  // constraint (linear motion is described by a circle of infinite radius).
  // Note that this is a _kinematic_ model, and as such it ignores dynamics
  // which generate important effects such as tire slip (i.e., a violation) of
  // the circular motion assumption). Ultimately these effects are treated
  // as model uncertainty and accounted for in the noise parameters.
  //
  // Ref: Probabilistic Robotics (Thrun 2006)
  class VelModel
  {
  public:
    // Constructor
    VelModel(const double car_length,
             const double speed_to_erpm_gain,
             const double speed_to_erpm_offset,
             const double steering_angle_to_servo_gain,
             const double steering_angle_to_servo_offset,
             const double lin_vel_n1,
             const double lin_vel_n2,
             const double ang_vel_n1,
             const double ang_vel_n2,
             const double th_n1,
             const double th_n2
            );

    // Applies the motion model to generate new samples of particles from p(x[t] | u[t], x[t-1])
    // Algorithm 5.3 from Probabilistic Robotics (Thrun 2006, page 124)
    void apply(const double motor_erpm,
               const double servo_pos,
               const double dt,
               std::vector<Particle>& particles
              );

  private:
    // Model parameters
    double car_length_;                     // Car length
    double speed_to_erpm_gain_;             // Gain for converting from velocity to electrical RPM (ERPM)
    double speed_to_erpm_offset_;           // Bias for converting from velocity to electrical RPM (ERPM)
    double steering_angle_to_servo_gain_;   // Gain for converting from steering angle to Servo position
    double steering_angle_to_servo_offset_; // Bias for converting from steering angle to Servo position
    double lin_vel_n1_;                     // Linear velocity noise coefficient 1
    double lin_vel_n2_;                     // Linear velocity noise coefficient 2
    double ang_vel_n1_;                     // Angular velocity noise coefficient 1
    double ang_vel_n2_;                     // Angular velocity noise coefficient 2
    double th_n1_;                          // Final rotation noise coefficient 1
    double th_n2_;                          // Final rotation noise coefficient 2
    NormalDistributionSampler sampler_;     // Normal distribution sampler

  }; // class VelModel

} // namespace localize

#endif // MOTION_H