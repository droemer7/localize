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
    VelModel(const double car_length,       // Car length
             const double lin_vel_n1,       // Model linear velocity noise coefficient 1
             const double lin_vel_n2,       // Model linear velocity noise coefficient 2
             const double ang_vel_n1,       // Model angular velocity noise coefficient 1
             const double ang_vel_n2,       // Model angular velocity noise coefficient 2
             const double th_n1,            // Model final rotation noise coefficient 1
             const double th_n2,            // Model final rotation noise coefficient 2
             ParticleVector& particles,     // Particle distribution
             RecursiveMutex& particles_mtx  // Recursive particle distribution mutex
            );

    // Apply the motion model to generate new samples of particles from
    // p(x[t] | u[t], x[t-1])
    // Algorithm 5.3 from Probabilistic Robotics (Thrun 2006, page 124)
    ParticleVector& update(const double lin_vel,
                           const double steering_angle,
                           const double dt
                          );

  private:
    ParticleVector& particles_;     // Particle distribution
    RecursiveMutex& particles_mtx_; // Recursive particle distribution mutex

    // Model parameters
    double car_length_; // Car length
    double lin_vel_n1_; // Model linear velocity noise coefficient 1
    double lin_vel_n2_; // Model linear velocity noise coefficient 2
    double ang_vel_n1_; // Model angular velocity noise coefficient 1
    double ang_vel_n2_; // Model angular velocity noise coefficient 2
    double th_n1_;      // Model final rotation noise coefficient 1
    double th_n2_;      // Model final rotation noise coefficient 2

    NormalDistributionSampler<double> sampler_; // Normal distribution sampler
  };

} // namespace localize

#endif // MOTION_H