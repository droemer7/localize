#include "mcl/node.h"

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

// ========== MCLNode ========== //
MCLNode::MCLNode(const std::string& localizer_node_name,
                 const std::string& localizer_pose_topic_name,
                 const std::string& localizer_pose_array_topic_name,
                 const std::string& drive_node_name,
                 const std::string& drive_vel_topic_name,
                 const std::string& drive_steer_topic_name,
                 const std::string& sensor_node_name,
                 const std::string& sensor_topic_name,
                 const std::string& map_topic
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
  map_frame_id_ = nh.param(localizer_node_name + "/map_frame_id", std::string("map"));
  if (   !getParam(nh, localizer_node_name + "/num_particles_min", num_particles_min_)
      || !getParam(nh, localizer_node_name + "/num_particles_max", num_particles_max_)
      || !getParam(nh, localizer_node_name + "/base_frame_id", base_frame_id_)
      || !getParam(nh, localizer_node_name + "/wheel_back_left_frame_id", wheel_back_left_frame_id_)
      || !getParam(nh, localizer_node_name + "/publish_tf", publish_tf_)
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
    throw std::runtime_error("MCL: Missing required parameters");
  }
  // Map parameters
  nav_msgs::GetMap map_msg;

  if (   !ros::service::waitForService(map_topic, ros::Duration(5))
      || !ros::service::call(map_topic, map_msg)
     ) {
    throw std::runtime_error(std::string("MCL: Failed to retrieve map from topic '") + map_topic + "'");
  }
  // Transforms
  TransformStampedMsg tf_base_to_sensor;
  TransformStampedMsg tf_wheel_back_left_to_sensor;

  try {
    tf_base_to_sensor = tf_buffer_.lookupTransform(sensor_frame_id_,
                                                   base_frame_id_,
                                                   ros::Time(0),
                                                   ros::Duration(15.0)
                                                  );
    tf_wheel_back_left_to_sensor = tf_buffer_.lookupTransform(sensor_frame_id_,
                                                              wheel_back_left_frame_id_,
                                                              ros::Time(0),
                                                              ros::Duration(15.0)
                                                             );
  }
  catch (tf2::TransformException & except) {
    throw std::runtime_error(except.what());
  }
  // Construct localizer before starting threads
  mcl_ptr_ = std::unique_ptr<MCL>(new MCL(num_particles_min_,
                                          num_particles_max_,
                                          car_length_,
                                          tf_base_to_sensor.transform.translation.x,
                                          tf_base_to_sensor.transform.translation.y,
                                          tf2::getYaw(tf_base_to_sensor.transform.rotation),
                                          tf_wheel_back_left_to_sensor.transform.translation.x,
                                          tf_base_to_sensor.transform.translation.y,
                                          sensor_range_min_,
                                          sensor_range_max_,
                                          sensor_range_no_obj_,
                                          map_msg.response.map.info.width,
                                          map_msg.response.map.info.height,
                                          map_msg.response.map.info.origin.position.x,
                                          map_msg.response.map.info.origin.position.y,
                                          tf2::getYaw(map_msg.response.map.info.origin.orientation),
                                          map_msg.response.map.info.resolution,
                                          map_msg.response.map.data
                                         )
                                 );
  // Assign ROS callback queues
  drive_vel_nh_.setCallbackQueue(&drive_vel_cb_queue_);
  drive_steer_nh.setCallbackQueue(&drive_steer_cb_queue_);
  sensor_nh_.setCallbackQueue(&sensor_cb_queue_);
  estimate_nh_.setCallbackQueue(&estimate_cb_queue_);
  status_nh_.setCallbackQueue(&status_cb_queue_);

  // Subscribe to topics for motor, steering servo and sensor data
  drive_vel_sub_ = drive_vel_nh_.subscribe(drive_vel_topic_name,
                                           1,
                                           &MCLNode::driveVelCb,
                                           this,
                                           ros::TransportHints().tcpNoDelay()
                                          );
  drive_steer_sub_ = drive_steer_nh.subscribe(drive_steer_topic_name,
                                              1,
                                              &MCLNode::driveSteerCb,
                                              this,
                                              ros::TransportHints().tcpNoDelay()
                                             );
  sensor_sub_ = sensor_nh_.subscribe(sensor_topic_name,
                                     1,
                                     &MCLNode::sensorCb,
                                     this,
                                     ros::TransportHints().tcpNoDelay()
                                    );
  // Create timers for estimate and status updates
  estimate_timer_ = status_nh_.createTimer(ros::Duration(0.02),
                                           &MCLNode::estimateCb,
                                           this
                                          );
  status_timer_ = status_nh_.createTimer(ros::Duration(5.0),
                                         &MCLNode::statusCb,
                                         this
                                        );
  // Setup publishers for pose estimates
  pose_pub_ = pose_nh_.advertise<PoseStampedMsg>(localizer_pose_topic_name, 1);
  pose_array_pub_ = pose_array_nh_.advertise<PoseArrayMsg>(localizer_pose_array_topic_name, 1);

  // Start threads
  drive_vel_spinner_.start();
  drive_steer_spinner_.start();
  sensor_spinner_.start();
  estimate_spinner_.start();
  status_spinner_.start();
}

void MCLNode::driveVelCb(const DriveStateStampedMsg::ConstPtr& msg)
{
  // Start timer
  ros::Time start = ros::Time::now();

  // Calculate velocity and steering angle
  double vel = (msg->state.speed - drive_vel_to_erpm_offset_) / drive_vel_to_erpm_gain_;
  double steer_angle = 0.0;
  {
    Lock lock(drive_steer_servo_pos_mtx_);
    steer_angle = (  (drive_steer_servo_pos_ - drive_steer_angle_to_servo_offset_)
                   / drive_steer_angle_to_servo_gain_
                  );
  }
  // Calculate motor time delta
  double dt = (msg->header.stamp - drive_t_prev_).toSec();
  drive_t_prev_ = msg->header.stamp;

  // Update localizer with motion data
  mcl_ptr_->update(vel, steer_angle, dt);

  // Save duration
  motion_update_time_msec_ = (ros::Time::now() - start).toSec() * 1000.0;
  motion_update_time_worst_msec_ = motion_update_time_msec_ > motion_update_time_worst_msec_?
                                   motion_update_time_msec_ : motion_update_time_worst_msec_;
}

void MCLNode::driveSteerCb(const DriveSteerMsg::ConstPtr& msg)
{
  // Update servo position
  Lock lock(drive_steer_servo_pos_mtx_);
  drive_steer_servo_pos_ = msg->data;
}

void MCLNode::sensorCb(const SensorScanMsg::ConstPtr& msg)
{
  // Start timer
  ros::Time start = ros::Time::now();

  // Update localizer with sensor data
  mcl_ptr_->update(rayScan(msg));

  // Save duration
  sensor_update_time_msec_ = (ros::Time::now() - start).toSec() * 1000.0;
  sensor_update_time_worst_msec_ = sensor_update_time_msec_ > sensor_update_time_worst_msec_?
                                   sensor_update_time_msec_ : sensor_update_time_worst_msec_;
}

void MCLNode::estimateCb(const ros::TimerEvent& event)
{
  // Publish transform and pose
  if (publish_tf_) {
    publishTf();
  }
  publishPose();
  publishPoseArray();
}

void MCLNode::statusCb(const ros::TimerEvent& event)
{
  printMotionUpdateTime(0.01);
  printSensorUpdateTime(0.5);
}

void MCLNode::publishTf()
{
  // MCL estimates the transform from the robot base frame to the map frame. In order to complete the transform tree,
  // we need to publish the missing map to odom frame transformation. This transformation represents a correction
  // which, when applied to a point in the odometry frame, yields the MCL estimate of where the robot is in the map.
  //
  // See REP 105 at https://www.ros.org/reps/rep-0105.html
  try {
    TransformStampedMsg tf_base_to_odom = tf_buffer_.lookupTransform(odom_frame_id_,
                                                                     base_frame_id_,
                                                                     ros::Time(0)
                                                                    );
    Particle tf_base_to_map = mcl_ptr_->estimate();

    if (tf_base_to_map.weight_normed_ > 0.0) {
      // Rotate the map frame into the odometry frame, and then calculate the delta from the map point (MCL estimate)
      // to the odometry point
      double tf_map_to_odom_th = tf2::getYaw(tf_base_to_odom.transform.rotation) - tf_base_to_map.th_;
      double tf_map_to_odom_x = (  tf_base_to_odom.transform.translation.x
                                 - (  tf_base_to_map.x_ * std::cos(tf_map_to_odom_th)
                                    - tf_base_to_map.y_ * std::sin(tf_map_to_odom_th)
                                   )
                                );
      double tf_map_to_odom_y = (  tf_base_to_odom.transform.translation.y
                                 - (  tf_base_to_map.x_ * std::sin(tf_map_to_odom_th)
                                    + tf_base_to_map.y_ * std::cos(tf_map_to_odom_th)
                                   )
                                );
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
  }
  catch (tf2::TransformException & except) {
    ROS_WARN("MCL: %s", except.what());
  }
}

void MCLNode::publishPose()
{
  // Get estimate
  Particle estimate = mcl_ptr_->estimate();

  // Estimates with a zero weight are considered null
  if (estimate.weight_normed_ > 0.0) {
    PoseStampedMsg pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_frame_id_;
    pose_msg.pose = poseMsg(estimate);

    pose_pub_.publish(pose_msg);
  }
}

void MCLNode::publishPoseArray()
{
  // Get estimates
  ParticleVector estimates = mcl_ptr_->estimates();

  // Check if we have anything to publish
  if (estimates.size() > 0) {
    PoseArrayMsg pose_array_msg;
    pose_array_msg.header.stamp = ros::Time::now();
    pose_array_msg.header.frame_id = map_frame_id_;

    for (size_t i = 0; i < estimates.size(); ++i) {
      // Estimates with a zero weight are considered null
      if (estimates[i].weight_normed_ > 0.0) {
        pose_array_msg.poses.push_back(poseMsg(estimates[i]));
      }
    }
    pose_array_pub_.publish(pose_array_msg);
  }
}

void MCLNode::printMotionUpdateTime(const double min_msec)
{
  if (motion_update_time_msec_ > min_msec) {
    ROS_INFO("MCL: Motion update times: last = %.2f ms, worst = %.2f",
             motion_update_time_msec_, motion_update_time_worst_msec_
            );
  }
}

void MCLNode::printSensorUpdateTime(const double min_msec)
{
  if (sensor_update_time_msec_ > min_msec) {
    ROS_INFO("MCL: Sensor update times: last = %.2f ms, worst = %.2f",
             sensor_update_time_msec_, sensor_update_time_worst_msec_
            );
  }
}