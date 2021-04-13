#include "mcl/motion.h"

static const double VEL_LIN_N1 = 0.01;  // Linear velocity noise coefficient 1
static const double VEL_LIN_N2 = 0.05;  // Linear velocity noise coefficient 2
static const double VEL_ANG_N1 = 0.01;  // Angular velocity noise coefficient 1
static const double VEL_ANG_N2 = 0.01;  // Angular velocity noise coefficient 2
static const double TH_N1 = 0.005;      // Final rotation noise coefficient 1
static const double TH_N2 = 0.005;      // Final rotation noise coefficient 2

using namespace localize;

// ========== VelModel ========== //
VelModel::VelModel(const double car_length,
                   const double car_back_to_motion_frame_x,
                   const double car_back_to_motion_frame_y,
                   const Map& map
                  ) :
  car_length_(car_length),
  car_back_to_motion_frame_x_(car_back_to_motion_frame_x),
  car_back_to_motion_frame_y_(car_back_to_motion_frame_y),
  map_(map)
{}

void VelModel::apply(ParticleDistribution& dist,
                     const double vel_lin,
                     const double steering_angle,
                     const double dt
                    )
{
  // Calculate angular velocity in the particle distribution's frame from steering angle and linear velocity
  // First find the radius of the particle frame's ICR (instantaneous center of rotation)
  double vel_ang = 0.0;
  double tan_steering_angle = std::tan(steering_angle);

  if (   !std::isnan(tan_steering_angle)
      && std::abs(tan_steering_angle) > DBL_EPSILON
     ) {
    // Calculate particle ICR by offsetting from the motion model reference point
    // Reference point is the midpoint between back wheels, with x axis collinear with velocity
    double particle_icr_x = car_back_to_motion_frame_x_;
    double particle_icr_y = (car_length_ / tan_steering_angle) + car_back_to_motion_frame_y_;
    double particle_icr_radius = std::sqrt(particle_icr_x * particle_icr_x + particle_icr_y * particle_icr_y);

    // Radius of ICR sign must match steering angle sign so the resulting angular velocity has the correct sign as well
    particle_icr_radius = std::signbit(tan_steering_angle) ? -particle_icr_radius : particle_icr_radius;

    // Angular velocity = v / r
    vel_ang = vel_lin / particle_icr_radius;
  }
  else {
    vel_ang = 0.0;
  }
  // Calculate standard deviation of noise
  double vel_lin_sq = vel_lin * vel_lin;
  double vel_ang_sq = vel_ang * vel_ang;

  double vel_lin_std_dev = std::sqrt(VEL_LIN_N1 * vel_lin_sq + VEL_LIN_N2 * vel_ang_sq);
  double vel_ang_std_dev = std::sqrt(VEL_ANG_N1 * vel_lin_sq + VEL_ANG_N2 * vel_ang_sq);
  double th_std_dev = std::sqrt(TH_N1 * vel_lin_sq + TH_N2 * vel_ang_sq);

  // Per particle quantities
  double vel_lin_noise = 0.0;
  double vel_ang_noise = 0.0;
  double th_noise = 0.0;
  double vel_lin_adj = 0.0;
  double vel_ang_adj = 0.0;

  double particle_new_x = 0.0;
  double particle_new_y = 0.0;

  // Apply to each particle in the distribution
  for (size_t i = 0; i < dist.count(); ++i) {
    // Calculate noise for velocities and rotation
    vel_lin_noise = vel_lin_std_dev > DBL_MIN ? sampler_.gen(0.0, vel_lin_std_dev) : 0.0;
    vel_ang_noise = vel_ang_std_dev > DBL_MIN ? sampler_.gen(0.0, vel_ang_std_dev) : 0.0;
    th_noise = th_std_dev > DBL_MIN ? sampler_.gen(0.0, th_std_dev) : 0.0;

    // Add noise to velocities
    vel_lin_adj = vel_lin + vel_lin_noise;
    vel_ang_adj = vel_ang + vel_ang_noise;

    // Calculate new x and y using noisy velocities
    particle_new_x = dist.particle(i).x_ + (  (vel_lin_adj / vel_ang_adj)
                                            * (- std::sin(dist.particle(i).th_)
                                               + std::sin(dist.particle(i).th_ + vel_ang_adj * dt)
                                              )
                                           );
    particle_new_y = dist.particle(i).y_ + (  (vel_lin_adj / vel_ang_adj)
                                            * (  std::cos(dist.particle(i).th_)
                                               - std::cos(dist.particle(i).th_ + vel_ang_adj * dt)
                                              )
                                           );
    // Position validity check: only move particle if landed in free space
    if (!map_.occupied(particle_new_x, particle_new_y)) {
      // New position is valid
      dist.particle(i).x_ = particle_new_x;
      dist.particle(i).y_ = particle_new_y;

      // Calculate new orientation, adding additional noise
      dist.particle(i).th_ += (vel_ang_adj + th_noise) * dt;

      // Wrap orientation to (-pi, pi]
      dist.particle(i).th_ = wrapAngle(dist.particle(i).th_);
    }
  }
  return;
}