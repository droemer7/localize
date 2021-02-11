#include "mcl/motion.h"

using namespace localize;

VelModel::VelModel(const double car_length,
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
                  ) :
  car_length_(car_length),
  speed_to_erpm_gain_(speed_to_erpm_gain),
  speed_to_erpm_offset_(speed_to_erpm_offset),
  steering_angle_to_servo_gain_(steering_angle_to_servo_gain),
  steering_angle_to_servo_offset_(steering_angle_to_servo_offset),
  lin_vel_n1_(lin_vel_n1),
  lin_vel_n2_(lin_vel_n2),
  ang_vel_n1_(ang_vel_n1),
  ang_vel_n2_(ang_vel_n2),
  th_n1_(th_n1),
  th_n2_(th_n2)
{}

void VelModel::apply(const double motor_erpm,
                     const double servo_pos,
                     const double dt,
                     std::vector<Particle>& particles
                    )
{
  double lin_vel = (motor_erpm - speed_to_erpm_offset_) / speed_to_erpm_gain_;
  double steering_angle = (servo_pos - steering_angle_to_servo_offset_) / steering_angle_to_servo_gain_;

  double ang_vel = (lin_vel / car_length_) * std::tan(steering_angle);
  double lin_vel_sq = lin_vel * lin_vel;
  double ang_vel_sq = ang_vel * ang_vel;
  double lin_vel_noise = 0.0;
  double ang_vel_noise = 0.0;
  double th_noise = 0.0;
  double lin_vel_adj = 0.0;
  double ang_vel_adj = 0.0;

  if (   lin_vel_sq > DBL_EPSILON
      || ang_vel_sq > DBL_EPSILON
     )
  {
    for (Particle& particle : particles)
    {
      // Calculate noise for velocities and rotation
      lin_vel_noise = sampler_(std::sqrt(lin_vel_n1_ * lin_vel_sq + lin_vel_n2_ * ang_vel_sq));
      ang_vel_noise = sampler_(std::sqrt(ang_vel_n1_ * lin_vel_sq + ang_vel_n2_ * ang_vel_sq));
      th_noise = sampler_(std::sqrt(th_n1_ * lin_vel_sq + th_n2_ * ang_vel_sq));

      // Add noise to velocities
      lin_vel_adj = lin_vel + lin_vel_noise;
      ang_vel_adj = ang_vel + ang_vel_noise;

      // Calculate new pose x and y using noisy velocities
      particle.x_ += (  (lin_vel_adj / ang_vel_adj)
                      * (-std::sin(particle.th_) + std::sin(particle.th_ + ang_vel_adj * dt))
                     );
      particle.y_ += (  (lin_vel_adj / ang_vel_adj)
                      * ( std::cos(particle.th_) - std::cos(particle.th_ + ang_vel_adj * dt))
                     );

      // Calculate new pose rotation, adding additional noise
      particle.th_ += (ang_vel_adj + th_noise) * dt;
    }
  }

  // ROS_INFO("MCL: dt = %f s", dt);
  // ROS_INFO("MCL: x, y, theta = %f, %f, %f", particles_[0].x_, particles_[0].y_, particles_[0].rot_);
  // ROS_INFO("MCL: Linear velocity = %f m/s", lin_vel_);
  // ROS_INFO("MCL: Angular velocity = %f rad/s", rot_vel);
}
