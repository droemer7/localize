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
    VelModel(const double car_length,               // Car length
             const double particle_to_back_frame_x, // Particle frame to back (midpoint between back wheels) x translation
             const double particle_to_back_frame_y  // Particle frame to back (midpoint between back wheels) x translation
            );

    // Apply the motion model to generate new samples of particles from p(x[t] | u[t], x[t-1])
    // Algorithm 5.3 from Probabilistic Robotics (Thrun 2006, page 124)
    void apply(ParticleDistribution& dist,
               const double vel_lin,
               const double steering_angle,
               const double dt
              );

  private:
    double car_length_;               // Car length
    double particle_to_back_frame_x_; // Particle frame to back (midpoint between back wheels) x translation
    double particle_to_back_frame_y_; // Particle frame to back (midpoint between back wheels) x translation

    NormalDistributionSampler<double> sampler_; // Normal distribution sampler
  };

} // namespace localize

#endif // MOTION_H