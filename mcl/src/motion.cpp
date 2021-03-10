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

void VelModel::update(ParticleVector& particles,
                      const size_t num_particles,
                      const double lin_vel,
                      const double steering_angle,
                      const double dt
                     )
{
  double ang_vel = (lin_vel / car_length_) * std::tan(steering_angle);
  double lin_vel_sq = lin_vel * lin_vel;
  double ang_vel_sq = ang_vel * ang_vel;
  double lin_vel_noise = 0.0;
  double ang_vel_noise = 0.0;
  double th_noise = 0.0;
  double lin_vel_adj = 0.0;
  double ang_vel_adj = 0.0;

  // Numerical check for square root
  if (   lin_vel_sq > DBL_EPSILON
      || ang_vel_sq > DBL_EPSILON
     ) {
    // TBD restore num_particles
    for (size_t i = 0; i < num_particles; ++i) {
      if (i >= 200000) {
        printf("Invalid index i=%lu", i);
      }
      // Calculate noise for velocities and rotation
      lin_vel_noise = sampler_.gen(0.0, std::sqrt(  lin_vel_n1_ * lin_vel_sq
                                                  + lin_vel_n2_ * ang_vel_sq
                                                 )
                                  );
      ang_vel_noise = sampler_.gen(0.0, std::sqrt(  ang_vel_n1_ * lin_vel_sq
                                                  + ang_vel_n2_ * ang_vel_sq
                                                 )
                                  );
      th_noise = sampler_.gen(0.0, std::sqrt(  th_n1_ * lin_vel_sq
                                             + th_n2_ * ang_vel_sq
                                            )
                             );

      // Add noise to velocities
      lin_vel_adj = lin_vel + lin_vel_noise;
      ang_vel_adj = ang_vel + ang_vel_noise;

      // Calculate new pose x and y using noisy velocities
      particles[i].x_ += (  (lin_vel_adj / ang_vel_adj)
                          * (-std::sin(particles[i].th_) + std::sin(particles[i].th_ + ang_vel_adj * dt))
                         );
      particles[i].y_ += (  (lin_vel_adj / ang_vel_adj)
                          * ( std::cos(particles[i].th_) - std::cos(particles[i].th_ + ang_vel_adj * dt))
                         );
      // Calculate new pose rotation, adding additional noise
      particles[i].th_ += wrapAngle((ang_vel_adj + th_noise) * dt);
    }
  }
  return;
}
