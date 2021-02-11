#include "mcl/mcl.h"

using namespace localize;

MCL::MCL(const unsigned int num_particles,
         const VelModel motion_model,
         const BeamModel sensor_model
        ) :
  motion_model_(motion_model),
  sensor_model_(sensor_model),
  particles_(num_particles),
  weights_(num_particles),
  servo_pos_(0.0),
  curr_t_(ros::Time::now().toSec()),
  prev_t_(ros::Time::now().toSec())
{}

void MCL::motorCb(const vesc_msgs::VescStateStamped::ConstPtr& msg)
{
  // ROS_INFO("MCL: motorCb()");
  std::lock_guard<std::mutex> lock(motor_mtx_);

  // Calculate inputs
  double motor_erpm = msg->state.speed;
  double dt = (msg->header.stamp - prev_t_).toSec();
  prev_t_ = msg->header.stamp;

  // Apply the motion model to update particle poses
  motion_model_.apply(motor_erpm,
                      servo_pos_,
                      dt,
                      particles_
                     );
}

void MCL::servoCb(const std_msgs::Float64::ConstPtr& msg)
{
  // ROS_INFO("MCL: servoCb()");
  std::lock_guard<std::mutex> lock(servo_mtx_);
  servo_pos_ = msg->data;
}

void MCL::sensorCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // ROS_INFO("MCL: sensorCb()");
  std::lock_guard<std::mutex> lock(sensor_mtx_);

  // sensor_model_.apply(msg->ranges,
  //                     particles_,
  //                     weights_
  //                    );
}
