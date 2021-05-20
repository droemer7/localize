#include "amcl/motion.h"

static const double EPSILON = 1e-6;

using namespace localize;

// ========== VelModel ========== //
VelModel::VelModel(const double car_length,
                   const double car_back_center_to_motion_frame_x,
                   const double car_back_center_to_motion_frame_y,
                   const double vel_lin_n1,
                   const double vel_lin_n2,
                   const double vel_ang_n1,
                   const double vel_ang_n2,
                   const double th_n1,
                   const double th_n2,
                   const double vel_ang_bias_scale,
                   const Map& map
                  ) :
  car_length_(car_length),
  car_back_center_to_motion_frame_x_(car_back_center_to_motion_frame_x),
  car_back_center_to_motion_frame_y_(car_back_center_to_motion_frame_y),
  vel_lin_n1_(vel_lin_n1),
  vel_lin_n2_(vel_lin_n2),
  vel_ang_n1_(vel_ang_n1),
  vel_ang_n2_(vel_ang_n2),
  th_n1_(th_n1),
  th_n2_(th_n2),
  vel_ang_bias_scale_(vel_ang_bias_scale),
  map_(map)
{}

void VelModel::apply(ParticleDistribution& dist,
                     const double vel_lin,
                     const double steer_angle,
                     const double dt
                    )
{
  // Calculate angular velocity and centripetal acceleration in the motion model's frame
  double vel_lin_sq = vel_lin * vel_lin;
  double vel_ang = 0.0;
  double vel_ang_sq = 0.0;
  double acc_center = 0.0;
  double tan_steer_angle = std::tan(steer_angle);

  if (std::abs(tan_steer_angle) > EPSILON) {
    // Calculate ICR radius by offsetting from the motion model reference point (midpoint between back wheels)
    double icr_radius_x = std::abs(car_back_center_to_motion_frame_x_);
    double icr_radius_y = (car_length_ / tan_steer_angle) + car_back_center_to_motion_frame_y_;
    double icr_radius = std::sqrt(icr_radius_x * icr_radius_x + icr_radius_y * icr_radius_y);

    // Calculate angular velocity
    vel_ang = std::signbit(tan_steer_angle) ? -vel_lin / icr_radius
                                            :  vel_lin / icr_radius;
    vel_ang_sq = vel_ang * vel_ang;

    // Calculate centripetal acceleration
    acc_center = vel_lin_sq / icr_radius;
  }
  // Calculate noise parameters
  double vel_lin_std_dev = std::sqrt(vel_lin_n1_ * vel_lin_sq + vel_lin_n2_ * vel_ang_sq);
  double vel_ang_std_dev = std::sqrt(vel_ang_n1_ * vel_lin_sq + vel_ang_n2_ * vel_ang_sq);
  double th_std_dev = std::sqrt(th_n1_ * vel_lin_sq + th_n2_ * vel_ang_sq);
  double vel_ang_scale_std_dev = std::sqrt(vel_ang_bias_scale_ * acc_center);

  // Per particle quantities
  double vel_lin_noise = 0.0;
  double vel_ang_noise = 0.0;
  double vel_ang_scale_noise = 0.0;
  double th_noise = 0.0;
  double vel_lin_adj = 0.0;
  double vel_ang_adj = 0.0;
  Point new_point;

  bool printed = false;

  // Apply to each particle in the distribution
  for (size_t i = 0; i < dist.count(); ++i) {
    // Calculate noise terms
    vel_lin_noise       = vel_lin_std_dev       > EPSILON ? sampler_.gen(0.0, vel_lin_std_dev)       : 0.0;
    vel_ang_noise       = vel_ang_std_dev       > EPSILON ? sampler_.gen(0.0, vel_ang_std_dev)       : 0.0;
    vel_ang_scale_noise = vel_ang_scale_std_dev > EPSILON ? sampler_.gen(0.0, vel_ang_scale_std_dev) : 0.0;
    th_noise            = th_std_dev            > EPSILON ? sampler_.gen(0.0, th_std_dev)            : 0.0;

    // Add noise to velocities
    vel_lin_adj = vel_lin + vel_lin_noise;
    vel_ang_adj = (vel_ang + vel_ang_noise) / (1.0 + std::abs(vel_ang_scale_noise));

    // Set initial values for x & y
    new_point.x_ = dist.particle(i).x_;
    new_point.y_ = dist.particle(i).y_;

    // Apply kinematics to propagate x & y forward with noise-adjusted values
    if (std::abs(vel_ang_adj) > EPSILON) {
      new_point.x_ += (  (vel_lin_adj / vel_ang_adj)
                       * (  std::sin(dist.particle(i).th_ + vel_ang_adj * dt)
                          - std::sin(dist.particle(i).th_)
                         )
                      );
      new_point.y_ += (  (vel_lin_adj / vel_ang_adj)
                       * (  std::cos(dist.particle(i).th_)
                          - std::cos(dist.particle(i).th_ + vel_ang_adj * dt)
                         )
                      );
    }
    else {
      new_point.x_ += vel_lin_adj * std::cos(dist.particle(i).th_ + vel_ang_noise * dt) * dt;
      new_point.y_ += vel_lin_adj * std::sin(dist.particle(i).th_ + vel_ang_noise * dt) * dt;
    }
    // Only move particle if it landed in free space, otherwise leave it where it is
    if (!map_.occupied(worldToMap(map_, new_point))) {
      // Update location
      dist.particle(i).x_ = new_point.x_;
      dist.particle(i).y_ = new_point.y_;

      // Calculate new orientation, adding additional noise
      dist.particle(i).th_ += (vel_ang_adj + th_noise) * dt;

      // Wrap orientation to (-pi, pi]
      dist.particle(i).th_ = wrapAngle(dist.particle(i).th_);
    }
  }
}
