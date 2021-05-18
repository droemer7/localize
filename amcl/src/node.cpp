#include "amcl/node.h"

using namespace localize;

PoseMsg localize::poseMsg(const Particle& particle)
{
  PoseMsg pose_msg;

  // Convert heading angle to quaternion orientation
  tf2::Quaternion particle_orient;
  particle_orient.setRPY(0.0, 0.0, particle.th_);

  // Populate message
  pose_msg.position.x = particle.x_;
  pose_msg.position.y = particle.y_;
  pose_msg.position.z = 0.0;
  pose_msg.orientation.x = particle_orient.x();
  pose_msg.orientation.y = particle_orient.y();
  pose_msg.orientation.z = particle_orient.z();
  pose_msg.orientation.w = particle_orient.w();

  return pose_msg;
}

RayScan localize::rayScan(const SensorScanMsg::ConstPtr& msg)
{
  RayScan ray_scan(msg->ranges.size());
  ray_scan.th_inc_ = msg->angle_increment;
  ray_scan.t_inc_ = msg->time_increment;
  ray_scan.t_dur_ = msg->scan_time;

  for (size_t i = 0; i < ray_scan.rays_.size(); ++i) {
    ray_scan.rays_[i].range_ = msg->ranges[i];
    ray_scan.rays_[i].th_ = msg->angle_min + ray_scan.th_inc_ * i;
  }
  return ray_scan;
}

// ========== AMCLNode ========== //
AMCLNode::AMCLNode(const std::string& amcl_node_name,
                   const std::string& amcl_pose_topic_name,
                   const std::string& amcl_pose_array_topic_name,
                   const std::string& drive_node_name,
                   const std::string& drive_vel_topic_name,
                   const std::string& drive_steer_topic_name,
                   const std::string& sensor_node_name,
                   const std::string& sensor_topic_name
                  ) :
  drive_vel_spinner_(1, &drive_vel_cb_queue_),
  drive_steer_spinner_(1, &drive_steer_cb_queue_),
  sensor_spinner_(1, &sensor_cb_queue_),
  estimate_spinner_(1, &estimate_cb_queue_),
  status_spinner_(1, &status_cb_queue_),
  drive_t_prev_(ros::Time::now()),
  tf_buffer_(ros::Duration(1)),
  tf_listener_(tf_buffer_),
  drive_steer_servo_pos_(0.0),
  motion_update_time_msec_(0.0),
  motion_update_time_worst_msec_(0.0),
  sensor_update_time_msec_(0.0),
  sensor_update_time_worst_msec_(0.0)
{
  // Localizer parameters
  ros::NodeHandle nh;
  map_frame_id_ = nh.param(amcl_node_name + "frame_ids/map", std::string("map"));
  if (   !getParam(nh, amcl_node_name + "/amcl/update_rate", amcl_update_rate_)
      || !getParam(nh, amcl_node_name + "/amcl/use_modified_map", amcl_use_modified_map_)
      || !getParam(nh, amcl_node_name + "/amcl/num_particles_min", amcl_num_particles_min_)
      || !getParam(nh, amcl_node_name + "/amcl/num_particles_max", amcl_num_particles_max_)
      || !getParam(nh, amcl_node_name + "/amcl/weight_avg_random_sample", amcl_weight_avg_random_sample_)
      || !getParam(nh, amcl_node_name + "/amcl/weight_rel_dev_resample", amcl_weight_rel_dev_resample_)
      || !getParam(nh, amcl_node_name + "/motion/vel_lin_n1", motion_vel_lin_n1_)
      || !getParam(nh, amcl_node_name + "/motion/vel_lin_n2", motion_vel_lin_n2_)
      || !getParam(nh, amcl_node_name + "/motion/vel_ang_n1", motion_vel_ang_n1_)
      || !getParam(nh, amcl_node_name + "/motion/vel_ang_n2", motion_vel_ang_n2_)
      || !getParam(nh, amcl_node_name + "/motion/th_n1", motion_th_n1_)
      || !getParam(nh, amcl_node_name + "/motion/th_n2", motion_th_n2_)
      || !getParam(nh, amcl_node_name + "/motion/vel_ang_bias_scale", motion_vel_ang_bias_scale_)
      || !getParam(nh, amcl_node_name + "/sensor/range_std_dev", sensor_range_std_dev_)
      || !getParam(nh, amcl_node_name + "/sensor/decay_rate_new_obj", sensor_decay_rate_new_obj_)
      || !getParam(nh, amcl_node_name + "/sensor/weight_no_obj", sensor_weight_no_obj_)
      || !getParam(nh, amcl_node_name + "/sensor/weight_new_obj", sensor_weight_new_obj_)
      || !getParam(nh, amcl_node_name + "/sensor/weight_map_obj", sensor_weight_map_obj_)
      || !getParam(nh, amcl_node_name + "/sensor/weight_rand_effect", sensor_weight_rand_effect_)
      || !getParam(nh, amcl_node_name + "/sensor/weight_uncertainty_factor", sensor_weight_uncertainty_factor_)
      || !getParam(nh, amcl_node_name + "/sensor/prob_new_obj_reject", sensor_prob_new_obj_reject_)
      || !getParam(nh, amcl_node_name + "/frame_ids/car_base", car_base_frame_id_)
      || !getParam(nh, amcl_node_name + "/frame_ids/car_wheel_back_left", car_wheel_back_left_frame_id_)
      || !getParam(nh, drive_node_name + "/frame_id", odom_frame_id_)
      || !getParam(nh, drive_node_name + "/chassis_length", car_length_)
      || !getParam(nh, drive_node_name + "/speed_to_erpm_gain", drive_vel_to_erpm_gain_)
      || !getParam(nh, drive_node_name + "/speed_to_erpm_offset", drive_vel_to_erpm_offset_)
      || !getParam(nh, drive_node_name + "/steering_angle_to_servo_gain", drive_steer_angle_to_servo_gain_)
      || !getParam(nh, drive_node_name + "/steering_angle_to_servo_offset", drive_steer_angle_to_servo_offset_)
      || !getParam(nh, sensor_node_name + "/frame_id", sensor_frame_id_)
      || !getParam(nh, sensor_node_name + "/range_min", sensor_range_min_)
      || !getParam(nh, sensor_node_name + "/range_max", sensor_range_max_)
      || !getParam(nh, sensor_node_name + "/range_no_obj", sensor_range_no_obj_)
     ) {
    throw std::runtime_error("AMCL: Missing required parameters");
  }
  // Map occupancy grid
  nav_msgs::OccupancyGrid::ConstPtr map_msg_ptr;

  if (amcl_use_modified_map_) {
    map_msg_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(std::string("/map_modified"), nh);
  }
  else {
    map_msg_ptr = ros::topic::waitForMessage<nav_msgs::OccupancyGrid>(std::string("/map_actual"), nh);
  }
  // Transforms
  TransformStampedMsg tf_base_to_sensor;
  TransformStampedMsg tf_wheel_back_left_to_sensor;

  try {
    tf_base_to_sensor = tf_buffer_.lookupTransform(sensor_frame_id_,
                                                   car_base_frame_id_,
                                                   ros::Time(0),
                                                   ros::Duration(15.0)
                                                  );
    tf_wheel_back_left_to_sensor = tf_buffer_.lookupTransform(sensor_frame_id_,
                                                              car_wheel_back_left_frame_id_,
                                                              ros::Time(0),
                                                              ros::Duration(15.0)
                                                             );
  }
  catch (tf2::TransformException & except) {
    throw std::runtime_error(except.what());
  }
  // Construct localizer before starting threads
  amcl_ptr_ = std::unique_ptr<AMCL>(new AMCL(amcl_num_particles_min_,
                                             amcl_num_particles_max_,
                                             amcl_weight_avg_random_sample_,
                                             amcl_weight_rel_dev_resample_,
                                             car_length_,
                                             tf_base_to_sensor.transform.translation.x,
                                             tf_base_to_sensor.transform.translation.y,
                                             tf2::getYaw(tf_base_to_sensor.transform.rotation),
                                             tf_wheel_back_left_to_sensor.transform.translation.x,
                                             tf_base_to_sensor.transform.translation.y,
                                             motion_vel_lin_n1_,
                                             motion_vel_lin_n2_,
                                             motion_vel_ang_n1_,
                                             motion_vel_ang_n2_,
                                             motion_th_n1_,
                                             motion_th_n2_,
                                             motion_vel_ang_bias_scale_,
                                             sensor_range_min_,
                                             sensor_range_max_,
                                             sensor_range_no_obj_,
                                             sensor_range_std_dev_,
                                             sensor_decay_rate_new_obj_,
                                             sensor_weight_no_obj_,
                                             sensor_weight_new_obj_,
                                             sensor_weight_map_obj_,
                                             sensor_weight_rand_effect_,
                                             sensor_weight_uncertainty_factor_,
                                             sensor_prob_new_obj_reject_,
                                             map_msg_ptr->info.width,
                                             map_msg_ptr->info.height,
                                             map_msg_ptr->info.origin.position.x,
                                             map_msg_ptr->info.origin.position.y,
                                             tf2::getYaw(map_msg_ptr->info.origin.orientation),
                                             map_msg_ptr->info.resolution,
                                             map_msg_ptr->data
                                            )
                                   );
  // Assign ROS callback queues
  drive_vel_nh_.setCallbackQueue(&drive_vel_cb_queue_);
  drive_steer_nh_.setCallbackQueue(&drive_steer_cb_queue_);
  sensor_nh_.setCallbackQueue(&sensor_cb_queue_);
  estimate_nh_.setCallbackQueue(&estimate_cb_queue_);
  status_nh_.setCallbackQueue(&status_cb_queue_);

  // Subscribe to topics for motor, steering servo and sensor data
  drive_vel_sub_ = drive_vel_nh_.subscribe(drive_vel_topic_name,
                                           1,
                                           &AMCLNode::driveVelCb,
                                           this,
                                           ros::TransportHints().tcpNoDelay()
                                          );
  drive_steer_sub_ = drive_steer_nh_.subscribe(drive_steer_topic_name,
                                               1,
                                               &AMCLNode::driveSteerCb,
                                               this,
                                               ros::TransportHints().tcpNoDelay()
                                              );
  sensor_sub_ = sensor_nh_.subscribe(sensor_topic_name,
                                     1,
                                     &AMCLNode::sensorCb,
                                     this,
                                     ros::TransportHints().tcpNoDelay()
                                    );
  // Create timers for estimate and status updates
  estimate_timer_ = estimate_nh_.createTimer(ros::Duration(1.0 / amcl_update_rate_),
                                             &AMCLNode::estimateCb,
                                             this
                                            );
  status_timer_ = status_nh_.createTimer(ros::Duration(5.0),
                                         &AMCLNode::statusCb,
                                         this
                                        );
  // Setup publishers for pose estimates
  pose_pub_ = pose_nh_.advertise<PoseStampedMsg>(amcl_pose_topic_name, 1);
  pose_array_pub_ = pose_array_nh_.advertise<PoseArrayMsg>(amcl_pose_array_topic_name, 1);

  // Start threads
  drive_vel_spinner_.start();
  drive_steer_spinner_.start();
  sensor_spinner_.start();
  estimate_spinner_.start();
  status_spinner_.start();
}

void AMCLNode::driveVelCb(const DriveStateStampedMsg::ConstPtr& drive_msg)
{
  // Start timer
  ros::Time start = ros::Time::now();

  // Calculate velocity and steering angle
  double vel = (drive_msg->state.speed - drive_vel_to_erpm_offset_) / drive_vel_to_erpm_gain_;
  double steer_angle = 0.0;
  {
    Lock lock(drive_steer_servo_pos_mtx_);
    steer_angle = (  (drive_steer_servo_pos_ - drive_steer_angle_to_servo_offset_)
                   / drive_steer_angle_to_servo_gain_
                  );
  }
  // Calculate motor time delta
  double dt = (drive_msg->header.stamp - drive_t_prev_).toSec();
  drive_t_prev_ = drive_msg->header.stamp;

  // Update localizer with motion data
  amcl_ptr_->motionUpdate(vel, steer_angle, dt);

  // Save duration
  motion_update_time_msec_ = (ros::Time::now() - start).toSec() * 1000.0;
  motion_update_time_worst_msec_ = motion_update_time_msec_ > motion_update_time_worst_msec_?
                                   motion_update_time_msec_ : motion_update_time_worst_msec_;
}

void AMCLNode::driveSteerCb(const DriveSteerMsg::ConstPtr& drive_msg)
{
  // Update servo position
  Lock lock(drive_steer_servo_pos_mtx_);
  drive_steer_servo_pos_ = drive_msg->data;
}

void AMCLNode::sensorCb(const SensorScanMsg::ConstPtr& msg_sensor)
{
  // Start timer
  ros::Time start = ros::Time::now();

  // Update localizer with sensor data
  amcl_ptr_->sensorUpdate(rayScan(msg_sensor));

  // Save duration
  sensor_update_time_msec_ = (ros::Time::now() - start).toSec() * 1000.0;
  sensor_update_time_worst_msec_ = sensor_update_time_msec_ > sensor_update_time_worst_msec_?
                                   sensor_update_time_msec_ : sensor_update_time_worst_msec_;
}

void AMCLNode::estimateCb(const ros::TimerEvent& event)
{
  // Get estimates
  ParticleVector estimates = amcl_ptr_->estimates();
  Particle estimate = estimates.size() > 0 ? estimates[0] : Particle();

  // Publish transform and poses
  publishTf(estimate);
  publishPose(estimate);
  publishPoseArray(estimates);
}

void AMCLNode::statusCb(const ros::TimerEvent& event)
{
  printMotionUpdateTime(0.01);
  printSensorUpdateTime(0.5);
}

void AMCLNode::publishTf(const Particle& estimate)
{
  // AMCL estimates the transform from the car base to the map frame. In order to complete the transform tree, we
  // need to publish the missing map to odom frame transformation.
  if (estimate.weight_normed_ > 0.0) {
    try {
      // Get the latest transform of the odometry frame relative to the car base frame
      TransformStampedMsg tf_odom_to_base = tf_buffer_.lookupTransform(car_base_frame_id_,
                                                                       odom_frame_id_,
                                                                       ros::Time(0)
                                                                      );
      // Rotate the odometry to car base x & y translations into the map frame
      // Then, map to odom = base to map [in map] + odom to base [in map]
      double tf_map_to_odom_x = (  estimate.x_
                                 + (  tf_odom_to_base.transform.translation.x * std::cos(estimate.th_)
                                    - tf_odom_to_base.transform.translation.y * std::sin(estimate.th_)
                                   )
                                );
      double tf_map_to_odom_y = (  estimate.y_
                                 + (  tf_odom_to_base.transform.translation.x * std::sin(estimate.th_)
                                    + tf_odom_to_base.transform.translation.y * std::cos(estimate.th_)
                                   )
                                );
      double tf_map_to_odom_th = estimate.th_ + tf2::getYaw(tf_odom_to_base.transform.rotation);

      // Convert map to odom angle to quaternion orientation
      tf2::Quaternion tf_map_to_odom_orient;
      tf_map_to_odom_orient.setRPY(0.0, 0.0, tf_map_to_odom_th);

      // Create transform message
      TransformStampedMsg tf_map_to_odom;
      tf_map_to_odom.header.stamp = ros::Time::now();
      tf_map_to_odom.header.frame_id = map_frame_id_;
      tf_map_to_odom.child_frame_id = odom_frame_id_;
      tf_map_to_odom.transform.translation.x = tf_map_to_odom_x;
      tf_map_to_odom.transform.translation.y = tf_map_to_odom_y;
      tf_map_to_odom.transform.translation.z = 0.0;
      tf_map_to_odom.transform.rotation.x = tf_map_to_odom_orient.x();
      tf_map_to_odom.transform.rotation.y = tf_map_to_odom_orient.y();
      tf_map_to_odom.transform.rotation.z = tf_map_to_odom_orient.z();
      tf_map_to_odom.transform.rotation.w = tf_map_to_odom_orient.w();

      // Broadcast transform
      tf_broadcaster_.sendTransform(tf_map_to_odom);
    }
    catch (tf2::TransformException & except) {
      ROS_WARN("AMCL: %s", except.what());
    }
  }
}

void AMCLNode::publishPose(const Particle& estimate)
{
  if (estimate.weight_normed_ > 0.0) {
    // Create pose message
    PoseStampedMsg pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_frame_id_;
    pose_msg.pose = poseMsg(estimate);

    // Publish pose
    pose_pub_.publish(pose_msg);
  }
}

void AMCLNode::publishPoseArray(const ParticleVector& estimates)
{
  if (estimates.size() > 0) {
    // Create pose array message
    PoseArrayMsg pose_array_msg;
    pose_array_msg.header.stamp = ros::Time::now();
    pose_array_msg.header.frame_id = map_frame_id_;

    for (size_t i = 0; i < estimates.size(); ++i) {
      if (estimates[i].weight_normed_ > 0.0) {
        pose_array_msg.poses.push_back(poseMsg(estimates[i]));
      }
    }
    // Publish pose array
    pose_array_pub_.publish(pose_array_msg);
  }
}

void AMCLNode::printMotionUpdateTime(const double min_msec)
{
  if (motion_update_time_msec_ > min_msec) {
    ROS_INFO("AMCL: Motion update times: last = %.2f ms, worst = %.2f",
             motion_update_time_msec_, motion_update_time_worst_msec_
            );
  }
}

void AMCLNode::printSensorUpdateTime(const double min_msec)
{
  if (sensor_update_time_msec_ > min_msec) {
    ROS_INFO("AMCL: Sensor update times: last = %.2f ms, worst = %.2f",
             sensor_update_time_msec_, sensor_update_time_worst_msec_
            );
  }
}