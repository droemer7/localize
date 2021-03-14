#include <float.h>

#include "mcl/motion.h"

using namespace localize;

VelModel::VelModel(const double car_length,
                   const double lin_vel_n1,
                   const double lin_vel_n2,
                   const double ang_vel_n1,
                   const double ang_vel_n2,
                   const double th_n1,
                   const double th_n2
                  ) :
  car_length_(car_length),
  lin_vel_n1_(lin_vel_n1),
  lin_vel_n2_(lin_vel_n2),
  ang_vel_n1_(ang_vel_n1),
  ang_vel_n2_(ang_vel_n2),
  th_n1_(th_n1),
  th_n2_(th_n2)
{}

void VelModel::update(ParticleDistribution& dist,
                      const double lin_vel,
                      const double steering_angle,
                      const double dt
                     )
{
  // Calculate angular velocity from steering angle and linear velocity
  double ang_vel = (lin_vel / car_length_) * std::tan(steering_angle);

  // Calculate standard deviation of noise
  double lin_vel_sq = lin_vel * lin_vel;
  double ang_vel_sq = ang_vel * ang_vel;

  double lin_vel_var = lin_vel_n1_ * lin_vel_sq + lin_vel_n2_ * ang_vel_sq;
  double ang_vel_var = ang_vel_n1_ * lin_vel_sq + ang_vel_n2_ * ang_vel_sq;
  double th_var = th_n1_ * lin_vel_sq + th_n2_ * ang_vel_sq;

  double lin_vel_std_dev = lin_vel_var > DBL_MIN ? std::sqrt(lin_vel_var) : 0.0;
  double ang_vel_std_dev = ang_vel_var > DBL_MIN ? std::sqrt(ang_vel_var) : 0.0;
  double th_std_dev = th_var > DBL_MIN ? std::sqrt(th_var) : 0.0;

  double lin_vel_noise = 0.0;
  double ang_vel_noise = 0.0;
  double th_noise = 0.0;
  double lin_vel_adj = 0.0;
  double ang_vel_adj = 0.0;

  // Update each particle
  for (size_t i = 0; i < dist.size(); ++i) {

    // Calculate noise for velocities and rotation
    lin_vel_noise = lin_vel_std_dev > DBL_MIN ? sampler_.gen(0.0, lin_vel_std_dev) : 0.0;
    ang_vel_noise = ang_vel_std_dev > DBL_MIN ? sampler_.gen(0.0, ang_vel_std_dev) : 0.0;
    th_noise = th_std_dev > DBL_MIN ? sampler_.gen(0.0, th_std_dev) : 0.0;

    // Add noise to velocities
    lin_vel_adj = lin_vel + lin_vel_noise;
    ang_vel_adj = ang_vel + ang_vel_noise;

    // Calculate new x and y using noisy velocities
    dist.particle(i).x_ += (  (lin_vel_adj / ang_vel_adj)
                            * (-std::sin(dist.particle(i).th_) + std::sin(dist.particle(i).th_ + ang_vel_adj * dt))
                           );
    dist.particle(i).y_ += (  (lin_vel_adj / ang_vel_adj)
                            * ( std::cos(dist.particle(i).th_) - std::cos(dist.particle(i).th_ + ang_vel_adj * dt))
                           );
    // Calculate new orientation, adding additional noise
    dist.particle(i).th_ += (ang_vel_adj + th_noise) * dt;

    // Wrap orientation to (-pi, pi]
    dist.particle(i).th_ = wrapAngle(dist.particle(i).th_);
  }
  return;
}
