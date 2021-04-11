#ifndef MOTION_H
#define MOTION_H

#include "mcl/distribution.h"
#include "mcl/common.h"

namespace localize
{
  // Velocity-based probabilistic model for an Ackermann drive robot
  //
  // The model assumes the robot is driven with linear and angular velocity commands and the resulting motion obeys a
  // circular motion kinematic constraint (linear motion is described by a circle of infinite radius).
  // Note that this is a _kinematic_ model, and as such it ignores dynamics which generate important effects such as
  // tire slip (i.e., a violation) of the circular motion assumption). Ultimately these effects are treated as model
  // uncertainty and accounted for in the noise parameters.
  //
  // Ref: Probabilistic Robotics (Thrun 2006)
  class VelModel
  {
  public:
    // Constructor
    VelModel(const double car_length,                 // Car length
             const double car_back_to_motion_frame_x, // Car back (midpoint between back wheels) to motion frame x translation
             const double car_back_to_motion_frame_y  // Car back (midpoint between back wheels) to motion frame y translation
            );

    // Apply the motion model to generate new samples of particles from p(x[t] | u[t], x[t-1])
    void apply(ParticleDistribution& dist,
               const double vel_lin,
               const double steering_angle,
               const double dt
              );

  private:
    double car_length_;                 // Car length
    double car_back_to_motion_frame_x_; // Car back (midpoint between back wheels) to motion frame x translation
    double car_back_to_motion_frame_y_; // Car back (midpoint between back wheels) to motion frame y translation

    NormalDistributionSampler<double> sampler_; // Normal distribution sampler
  };

} // namespace localize

#endif // MOTION_H