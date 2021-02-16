#include <ros/service.h>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>

#include <nav_msgs/GetMap.h>

#include "mcl/mcl_node.h"
#include "mcl/util.h"

using namespace localize;

MCLNode::MCLNode(const std::string& motor_topic,
                 const std::string& servo_topic,
                 const std::string& sensor_topic,
                 const std::string& map_topic
                ) :
  motor_spinner_(1, &motor_cb_queue_),
  servo_spinner_(1, &servo_cb_queue_),
  sensor_spinner_(1, &sensor_cb_queue_),
  curr_t_(ros::Time::now()),
  prev_t_(ros::Time::now()),
  servo_pos_(0.0)
{
  // ROS launch parameters
  ros::NodeHandle nh;
  if (   !getParam(nh, "localizer/num_particles", num_particles_)
      || !getParam(nh, "vesc/chassis_length", car_length_)
      || !getParam(nh, "vesc/speed_to_erpm_gain", motor_speed_to_erpm_gain_)
      || !getParam(nh, "vesc/speed_to_erpm_offset", motor_speed_to_erpm_offset_)
      || !getParam(nh, "vesc/steering_angle_to_servo_gain", motor_steering_angle_to_servo_gain_)
      || !getParam(nh, "vesc/steering_angle_to_servo_offset", motor_steering_angle_to_servo_offset_)
      || !getParam(nh, "localizer/motion_lin_vel_n1", motion_lin_vel_n1_)
      || !getParam(nh, "localizer/motion_lin_vel_n2", motion_lin_vel_n2_)
      || !getParam(nh, "localizer/motion_ang_vel_n1", motion_ang_vel_n1_)
      || !getParam(nh, "localizer/motion_ang_vel_n2", motion_ang_vel_n2_)
      || !getParam(nh, "localizer/motion_th_n1", motion_th_n1_)
      || !getParam(nh, "localizer/motion_th_n2", motion_th_n2_)
      || !getParam(nh, "laser/range_min", sensor_range_min_)
      || !getParam(nh, "laser/range_max", sensor_range_max_)
      || !getParam(nh, "localizer/sensor_range_no_obj", sensor_range_no_obj_)
      || !getParam(nh, "localizer/sensor_std_dev", sensor_range_std_dev_)
      || !getParam(nh, "localizer/sensor_weight_new_obj_decay_rate", sensor_new_obj_decay_rate_)
      || !getParam(nh, "localizer/sensor_weight_no_obj", sensor_weight_no_obj_)
      || !getParam(nh, "localizer/sensor_weight_new_obj", sensor_weight_new_obj_)
      || !getParam(nh, "localizer/sensor_weight_map_obj", sensor_weight_map_obj_)
      || !getParam(nh, "localizer/sensor_weight_rand_effect", sensor_weight_rand_effect_)
     ) {
    throw std::runtime_error("MCL: Missing required parameters");
  }
  // ROS map parameters
  nav_msgs::GetMap get_map_msg;
  if (   !ros::service::waitForService(map_topic, ros::Duration(30))
      || !ros::service::call(map_topic, get_map_msg)
     ) {
    throw std::runtime_error(std::string("MCL: Failed to retrieve map from ") + map_topic);
  }
  nav_msgs::OccupancyGrid map_msg = get_map_msg.response.map;
  tf2::Quaternion quat;
  tf2::convert(map_msg.info.origin.orientation, quat);
  tf2::Matrix3x3 matrix(quat);
  double roll, pitch, yaw = 0.0;
  matrix.getRPY(roll, pitch, yaw);

  map_height_ = map_msg.info.height;
  map_width_ = map_msg.info.width;
  map_m_per_pxl_ = map_msg.info.resolution;
  map_th_ = yaw;
  map_origin_x_ = map_msg.info.origin.position.x;
  map_origin_y_ = map_msg.info.origin.position.y;

  // Create the localizer
  mcl();

  // Assign ROS callback queues for each topic
  motor_nh_.setCallbackQueue(&motor_cb_queue_);
  servo_nh_.setCallbackQueue(&servo_cb_queue_);
  sensor_nh_.setCallbackQueue(&sensor_cb_queue_);

  // Subscribe to motor, servo (vesc) and sensor (laser scan) topics
  motor_sub_ = motor_nh_.subscribe(motor_topic,
                                   1,
                                   &MCLNode::motorCb,
                                   this,
                                   ros::TransportHints().tcpNoDelay()
                                  );
  servo_sub_ = servo_nh_.subscribe(servo_topic,
                                   1,
                                   &MCLNode::servoCb,
                                   this,
                                   ros::TransportHints().tcpNoDelay()
                                  );
  sensor_sub_ = sensor_nh_.subscribe(sensor_topic,
                                     1,
                                     &MCLNode::sensorCb,
                                     this,
                                     ros::TransportHints().tcpNoDelay()
                                    );

  // Dedicate one thread to each callback
  motor_spinner_.start();
  servo_spinner_.start();
  sensor_spinner_.start();
}

MCL & MCLNode::mcl()
{
  static MCL localizer(num_particles_,
                       car_length_,
                       motion_lin_vel_n1_,
                       motion_lin_vel_n2_,
                       motion_ang_vel_n1_,
                       motion_ang_vel_n2_,
                       motion_th_n1_,
                       motion_th_n2_,
                       sensor_range_min_,
                       sensor_range_max_,
                       sensor_range_no_obj_,
                       sensor_range_std_dev_,
                       sensor_new_obj_decay_rate_,
                       sensor_weight_no_obj_,
                       sensor_weight_new_obj_,
                       sensor_weight_map_obj_,
                       sensor_weight_rand_effect_,
                       1000,
                       map_height_,
                       map_width_,
                       map_m_per_pxl_,
                       map_th_,
                       map_origin_x_,
                       map_origin_y_,
                       map_occ_data_
                      );
  return localizer;
}

void MCLNode::motorCb(const vesc_msgs::VescStateStamped::ConstPtr& msg)
{
  // ROS_INFO("MCL: motorCb()");
  // Convert motor ERPM and steering servo position
  double motor_erpm = msg->state.speed;
  double lin_vel = (  (motor_erpm - motor_speed_to_erpm_offset_)
                    / motor_speed_to_erpm_gain_
                   );
  double steering_angle = 0.0;
  {
    std::lock_guard<std::mutex> lock(servo_mtx_);
    steering_angle = (  (servo_pos_ - motor_steering_angle_to_servo_offset_)
                      / motor_steering_angle_to_servo_gain_
                     );
  }

  // Calculate time delta
  double dt = (msg->header.stamp - prev_t_).toSec();
  prev_t_ = msg->header.stamp;

  // Perform motion update
  mcl().motionUpdate(lin_vel,
                     steering_angle,
                     dt
                    );
}

void MCLNode::servoCb(const std_msgs::Float64::ConstPtr& msg)
{
  // ROS_INFO("MCL: servoCb()");
  std::lock_guard<std::mutex> lock(servo_mtx_);
  servo_pos_ = msg->data;
}

void MCLNode::sensorCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // ROS_INFO("MCL: sensorCb()");
  // Perform sensor update
  mcl().sensorUpdate(msg->ranges);
}