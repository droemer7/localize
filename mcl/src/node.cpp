#include <float.h>

#include <ros/service.h>
#include <tf2/utils.h>

#include <nav_msgs/GetMap.h>

#include "mcl/node.h"
#include "mcl/util.h"

using namespace localize;

RayScanMsg::RayScanMsg(const sensor_msgs::LaserScan::ConstPtr& msg) :
  RayScan(msg->ranges.size())
{
  th_inc_ = msg->angle_increment;
  t_inc_ = msg->time_increment;
  t_dur_ = msg->scan_time;

  for (size_t i = 0; i < rays_.size(); ++i) {
    rays_[i].range_ = msg->ranges[i];
    rays_[i].th_ = msg->angle_min + th_inc_ * i;
  }
  rays_ = rays_;
}

MCLNode::MCLNode(const std::string& motor_topic,
                 const std::string& servo_topic,
                 const std::string& sensor_topic,
                 const std::string& map_topic
                ) :
  motor_spinner_(1, &motor_cb_queue_),
  servo_spinner_(1, &servo_cb_queue_),
  sensor_spinner_(1, &sensor_cb_queue_),
  status_spinner_(1, &status_cb_queue_),
  timer_cb_dur_(1.0),
  motor_t_prev_(ros::Time::now()),
  motion_dur_msec_(0.0),
  motion_dur_worst_msec_(0.0),
  sensor_dur_msec_(0.0),
  sensor_dur_worst_msec_(0.0),
  servo_pos_(0.0)
{
  // Motion and sensor model parameters
  ros::NodeHandle nh;
  if (   !getParam(nh, "localizer/mcl_num_particles_min", mcl_num_particles_min_)
      || !getParam(nh, "localizer/mcl_num_particles_max", mcl_num_particles_max_)
      || !getParam(nh, "localizer/mcl_kld_eps", mcl_kld_eps_)
      || !getParam(nh, "localizer/mcl_hist_pos_res", mcl_hist_pos_res_)
      || !getParam(nh, "localizer/mcl_hist_th_res", mcl_hist_th_res_)
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
      || !getParam(nh, "localizer/sensor_th_sample_res", sensor_th_sample_res_)
      || !getParam(nh, "localizer/sensor_th_raycast_res", sensor_th_raycast_res_)
      || !getParam(nh, "localizer/sensor_new_obj_decay_rate", sensor_new_obj_decay_rate_)
      || !getParam(nh, "localizer/sensor_weight_no_obj", sensor_weight_no_obj_)
      || !getParam(nh, "localizer/sensor_weight_new_obj", sensor_weight_new_obj_)
      || !getParam(nh, "localizer/sensor_weight_map_obj", sensor_weight_map_obj_)
      || !getParam(nh, "localizer/sensor_weight_rand_effect", sensor_weight_rand_effect_)
      || !getParam(nh, "localizer/sensor_uncertainty_factor", sensor_uncertainty_factor_)
      || !getParam(nh, "localizer/sensor_table_res", sensor_table_res_)
     ) {
    throw std::runtime_error("MCL: Missing required parameters");
  }
  // Map parameters
  nav_msgs::GetMap get_map_msg;
  if (   !ros::service::waitForService(map_topic, ros::Duration(5))
      || !ros::service::call(map_topic, get_map_msg)
     ) {
    throw std::runtime_error(std::string("MCL: Failed to retrieve map from ") + map_topic);
  }
  const nav_msgs::OccupancyGrid& map_msg = get_map_msg.response.map;
  map_width_ = map_msg.info.width;
  map_height_ = map_msg.info.height;
  map_x_origin_ = map_msg.info.origin.position.x;
  map_y_origin_ = map_msg.info.origin.position.y;
  map_th_ = tf2::getYaw(map_msg.info.origin.orientation);
  map_scale_ = map_msg.info.resolution;
  map_data_ = map_msg.data;

  // Construct the localizer with retrieved parameters before starting threads
  try {
    mcl_ptr_ = std::unique_ptr<MCL>(new MCL(mcl_num_particles_min_,
                                            mcl_num_particles_max_,
                                            mcl_kld_eps_,
                                            mcl_hist_pos_res_,
                                            mcl_hist_th_res_,
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
                                            sensor_th_sample_res_,
                                            sensor_th_raycast_res_,
                                            sensor_new_obj_decay_rate_,
                                            sensor_weight_no_obj_,
                                            sensor_weight_new_obj_,
                                            sensor_weight_map_obj_,
                                            sensor_weight_rand_effect_,
                                            sensor_uncertainty_factor_,
                                            sensor_table_res_,
                                            map_width_,
                                            map_height_,
                                            map_x_origin_,
                                            map_y_origin_,
                                            map_th_,
                                            map_scale_,
                                            map_data_
                                           )
                                   );
  }
  catch (std::runtime_error error) {
    throw std::runtime_error(error.what());
  }

  // Assign ROS callback queues
  motor_nh_.setCallbackQueue(&motor_cb_queue_);
  servo_nh_.setCallbackQueue(&servo_cb_queue_);
  sensor_nh_.setCallbackQueue(&sensor_cb_queue_);
  status_nh_.setCallbackQueue(&status_cb_queue_);

  // Subscribe to topics for motor, steering servo and sensor data
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
  // Create timer to handle printing status info
  status_timer_ = status_nh_.createTimer(timer_cb_dur_,
                                         &MCLNode::statusCb,
                                         this
                                        );
  // Start threads
  // Note: Callback queues were assigned to each thread in the initializer list
  motor_spinner_.start();
  servo_spinner_.start();
  sensor_spinner_.start();
  status_spinner_.start();
}

void MCLNode::motorCb(const vesc_msgs::VescStateStamped::ConstPtr& msg)
{
  // Start timer
  ros::Time start = ros::Time::now();

  // Calculate velocity
  double vel = (  (msg->state.speed - motor_speed_to_erpm_offset_)
                / motor_speed_to_erpm_gain_
               );
  // Calculate steering angle
  double steering_angle = 0.0;
  {
    std::lock_guard<std::mutex> lock(servo_mtx_);
    steering_angle = (  (servo_pos_ - motor_steering_angle_to_servo_offset_)
                      / motor_steering_angle_to_servo_gain_
                     );
  }
  // Calculate motor time delta
  double dt = (msg->header.stamp - motor_t_prev_).toSec();
  motor_t_prev_ = msg->header.stamp;

  // Perform motion update
  mcl_ptr_->update(vel, steering_angle, dt);

  // Save duration
  motion_dur_msec_ = (ros::Time::now() - start).toSec() * 1000.0;
  motion_dur_worst_msec_ = motion_dur_msec_ > motion_dur_worst_msec_?
                           motion_dur_msec_ : motion_dur_worst_msec_;
}

void MCLNode::servoCb(const std_msgs::Float64::ConstPtr& msg)
{
  // Update servo position
  std::lock_guard<std::mutex> lock(servo_mtx_);
  servo_pos_ = msg->data;
}

void MCLNode::sensorCb(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  // Start timer
  ros::Time start = ros::Time::now();

  // Perform sensor update
  // TBD remove try
  try {
    mcl_ptr_->update(RayScanMsg(msg));
  }
  catch (std::runtime_error error) {
    save();
    throw std::runtime_error(error);
  }

  // Save duration
  sensor_dur_msec_ = (ros::Time::now() - start).toSec() * 1000.0;
  sensor_dur_worst_msec_ = sensor_dur_msec_ > sensor_dur_worst_msec_?
                           sensor_dur_msec_ : sensor_dur_worst_msec_;
}

void MCLNode::save()
{
  mcl_ptr_->save("particles.csv");
}

template <class T>
bool MCLNode::getParam(const ros::NodeHandle& nh,
                       std::string name,
                       T& value
                      )
{
  bool result = true;

  if (!nh.getParam(name, value)) {
    ROS_FATAL("MCL: Parameter '%s' not found", name.c_str());
    result = false;
  }
  return result;
}

void MCLNode::statusCb(const ros::TimerEvent& event)
{
  if (motion_dur_msec_ > 0.01) {
    ROS_INFO("MCL: Motion update time (last) = %.2f ms", motion_dur_msec_);
    ROS_INFO("MCL: Motion update time (worst) = %.2f ms", motion_dur_worst_msec_);
  }

  if (sensor_dur_msec_ > 0.01) {
    ROS_INFO("MCL: Sensor update time (last) = %.2f ms", sensor_dur_msec_);
    ROS_INFO("MCL: Sensor update time (worst) = %.2f ms", sensor_dur_worst_msec_);
  }
}

void MCLNode::printMotionParams()
{
  ROS_INFO("MCL: car_length_ = %f", car_length_);
  ROS_INFO("MCL: motor_speed_to_erpm_gain_ = %f", motor_speed_to_erpm_gain_);
  ROS_INFO("MCL: motor_speed_to_erpm_offset_ = %f", motor_speed_to_erpm_offset_);
  ROS_INFO("MCL: motor_steering_angle_to_servo_gain_ = %f", motor_steering_angle_to_servo_gain_);
  ROS_INFO("MCL: motor_steering_angle_to_servo_offset_ = %f", motor_steering_angle_to_servo_offset_);
  ROS_INFO("MCL: motion_lin_vel_n1_ = %f", motion_lin_vel_n1_);
  ROS_INFO("MCL: motion_lin_vel_n2_ = %f", motion_lin_vel_n2_);
  ROS_INFO("MCL: motion_ang_vel_n1_ = %f", motion_ang_vel_n1_);
  ROS_INFO("MCL: motion_ang_vel_n2_ = %f", motion_ang_vel_n2_);
  ROS_INFO("MCL: motion_th_n1_ = %f", motion_th_n1_);
  ROS_INFO("MCL: motion_th_n2_ = %f", motion_th_n2_);
}

void MCLNode::printSensorParams()
{
  ROS_INFO("MCL: sensor_range_min_ = %f", sensor_range_min_);
  ROS_INFO("MCL: sensor_range_max_ = %f", sensor_range_max_);
  ROS_INFO("MCL: sensor_range_no_obj_ = %f", sensor_range_no_obj_);
  ROS_INFO("MCL: sensor_range_std_dev_ = %f", sensor_range_std_dev_);
  ROS_INFO("MCL: sensor_new_obj_decay_rate_ = %f", sensor_new_obj_decay_rate_);
  ROS_INFO("MCL: sensor_weight_no_obj_ = %f", sensor_weight_no_obj_);
  ROS_INFO("MCL: sensor_weight_new_obj_ = %f", sensor_weight_new_obj_);
  ROS_INFO("MCL: sensor_weight_map_obj_ = %f", sensor_weight_map_obj_);
  ROS_INFO("MCL: sensor_weight_rand_effect_ = %f", sensor_weight_rand_effect_);
}

void MCLNode::printMapParams()
{
  ROS_INFO("MCL: map_width_ = %d", map_width_);
  ROS_INFO("MCL: map_height_ = %d", map_height_);
  ROS_INFO("MCL: map_x_ = %f", map_x_origin_);
  ROS_INFO("MCL: map_y_ = %f", map_y_origin_);
  ROS_INFO("MCL: map_th_ = %f", map_th_);
  ROS_INFO("MCL: map_scale_ = %f", map_scale_);
}