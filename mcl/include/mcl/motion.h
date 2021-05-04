#ifndef MOTION_H
#define MOTION_H

#include "mcl/distribution.h"
#include "mcl/common.h"

namespace localize
{
  // Velocity-based probabilistic model for an Ackermann drive robot
  //
  // The model assumes the car is driven with linear and angular velocity commands and the resulting motion obeys a
  // circular motion constraint (linear motion is described by a circle of infinite radius).
  // Note that this is a _kinematic_ model, and as such it does not explicitly model dynamics which generate important
  // effects such as tire slip. Ultimately these unmodeled dynamics are compensated for in noise parameters.
  class VelModel
  {
  public:
    // Constructor
    VelModel(const double car_length,                         // Car length
             const double car_back_center_to_motion_frame_x,  // Car back center to motion frame x translation
             const double car_back_center_to_motion_frame_y,  // Car back center to motion frame y translation
             const double vel_lin_n1,                         // Model linear velocity noise coefficient 1
             const double vel_lin_n2,                         // Model linear velocity noise coefficient 2
             const double vel_ang_n1,                         // Model angular velocity noise coefficient 1
             const double vel_ang_n2,                         // Model angular velocity noise coefficient 2
             const double th_n1,                              // Model final rotation noise coefficient 1
             const double th_n2,                              // Model final rotation noise coefficient 2
             const double vel_ang_bias_scale,                 // Model slip scale factor: decrease angular velocity according to scale * v^2 / r
             const Map& map                                   // Map
            );

    // Apply the motion model to generate new samples of particles from p(x[t] | u[t], x[t-1])
    void apply(ParticleDistribution& dist,
               const double vel_lin,
               const double steer_angle,
               const double dt
              );

  private:
    const double car_length_;                         // Car length
    const double car_back_center_to_motion_frame_x_;  // Car back center to motion frame x translation
    const double car_back_center_to_motion_frame_y_;  // Car back center to motion frame y translation
    const double vel_lin_n1_;                         // Model linear velocity noise coefficient 1
    const double vel_lin_n2_;                         // Model linear velocity noise coefficient 2
    const double vel_ang_n1_;                         // Model angular velocity noise coefficient 1
    const double vel_ang_n2_;                         // Model angular velocity noise coefficient 2
    const double th_n1_;                              // Model final rotation noise coefficient 1
    const double th_n2_;                              // Model final rotation noise coefficient 2
    const double vel_ang_bias_scale_;                 // Model slip scale factor: decrease angular velocity according to scale * v^2 / r
    const Map& map_;                                  // Map

    NormalDistributionSampler<double> sampler_; // Normal distribution sampler
  };

} // namespace localize

#endif // MOTION_H