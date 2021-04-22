#include "mcl/motion.h"

static const double VEL_LIN_N1 = 0.003;  // Linear velocity noise coefficient 1
static const double VEL_LIN_N2 = 0.003;  // Linear velocity noise coefficient 2
static const double VEL_ANG_N1 = 0.003;  // Angular velocity noise coefficient 1
static const double VEL_ANG_N2 = 0.003;  // Angular velocity noise coefficient 2
static const double TH_N1 = 0.003;       // Final rotation noise coefficient 1
static const double TH_N2 = 0.003;       // Final rotation noise coefficient 2

using namespace localize;

// ========== VelModel ========== //
VelModel::VelModel(const double car_length,
                   const double car_back_center_to_motion_frame_x,
                   const double car_back_center_to_motion_frame_y,
                   const Map& map
                  ) :
  car_length_(car_length),
  car_back_center_to_motion_frame_x_(car_back_center_to_motion_frame_x),
  car_back_center_to_motion_frame_y_(car_back_center_to_motion_frame_y),
  map_(map)
{}

void VelModel::apply(ParticleDistribution& dist,
                     const double vel_lin,
                     const double steering_angle,
                     const double dt
                    )
{
  // Calculate angular velocity in the motion model's frame from the car's steering angle, linear velocity and the
  // motion frame's ICR (instantaneous center of rotation)
  double vel_ang = 0.0;
  double tan_steering_angle = std::tan(steering_angle);

  if (   !std::isnan(tan_steering_angle)
      && std::abs(tan_steering_angle) > DBL_EPSILON
     ) {
    // Calculate ICR radius by offsetting from the motion model reference point (midpoint between back wheels)
    double icr_radius_x = std::abs(car_back_center_to_motion_frame_x_);
    double icr_radius_y = (car_length_ / tan_steering_angle) + car_back_center_to_motion_frame_y_;
    double icr_radius = std::sqrt(icr_radius_x * icr_radius_x + icr_radius_y * icr_radius_y);

    // Sign of ICR radius must match steering angle sign so the resulting angular velocity has the correct sign
    icr_radius = std::signbit(tan_steering_angle) ? -icr_radius : icr_radius;

    // Angular velocity = v / r
    vel_ang = vel_lin / icr_radius;
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
  Point point;

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
    point.x_ = dist.particle(i).x_ + (  (vel_lin_adj / vel_ang_adj)
                                      * (- std::sin(dist.particle(i).th_)
                                         + std::sin(dist.particle(i).th_ + vel_ang_adj * dt)
                                        )
                                     );
    point.y_ = dist.particle(i).y_ + (  (vel_lin_adj / vel_ang_adj)
                                      * (  std::cos(dist.particle(i).th_)
                                         - std::cos(dist.particle(i).th_ + vel_ang_adj * dt)
                                        )
                                     );
    // Only move particle if it landed in free space, otherwise leave it where it is
    if (!map_.occupied(worldToMap(map_, point))) {
      // Update location
      dist.particle(i).x_ = point.x_;
      dist.particle(i).y_ = point.y_;

      // Calculate new orientation, adding additional noise
      dist.particle(i).th_ += (vel_ang_adj + th_noise) * dt;

      // Wrap orientation to (-pi, pi]
      dist.particle(i).th_ = wrapAngle(dist.particle(i).th_);
    }
  }
}