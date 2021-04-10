#include "mcl/node.h"

static const int NUM_UPDATES = 10000; // TBD remove

using namespace localize;

// ========== RayScanMsg ========== //
RayScanMsg::RayScanMsg(const SensorScanMsg::ConstPtr& msg) :
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

// ========== MCLNode ========== //
MCLNode::MCLNode(const std::string& pose_topic,
                 const std::string& drive_vel_topic,
                 const std::string& drive_steer_topic,
                 const std::string& sensor_topic,
                 const std::string& map_topic
                ) :
  map_frame_id_("map"),
  drive_vel_spinner_(1, &drive_vel_cb_queue_),
  drive_steer_spinner_(1, &drive_steer_cb_queue_),
  sensor_spinner_(1, &sensor_cb_queue_),
  status_spinner_(1, &status_cb_queue_),
  timer_cb_dur_(5.0),
  drive_t_prev_(ros::Time::now()),
  tf_buffer_(ros::Duration(1)),
  tf_listener_(tf_buffer_, true, ros::TransportHints().tcpNoDelay()),
  drive_steer_servo_pos_(0.0),
  motion_update_time_msec_(0.0),
  motion_update_time_worst_msec_(0.0),
  sensor_update_time_msec_(0.0),
  sensor_update_time_worst_msec_(0.0)
{
  // Localizer parameters
  ros::NodeHandle nh;
  if (   !getParam(nh, "localizer/num_particles_min", num_particles_min_)
      || !getParam(nh, "localizer/num_particles_max", num_particles_max_)
      || !getParam(nh, "localizer/base_frame_id", base_frame_id_)
      || !getParam(nh, "localizer/wheel_bl_frame_id", wheel_bl_frame_id_)
      || !getParam(nh, "vesc/frame_id", odom_frame_id_)
      || !getParam(nh, "vesc/chassis_length", car_length_)
      || !getParam(nh, "vesc/speed_to_erpm_gain", drive_vel_to_erpm_gain_)
      || !getParam(nh, "vesc/speed_to_erpm_offset", drive_vel_to_erpm_offset_)
      || !getParam(nh, "vesc/steering_angle_to_servo_gain", drive_steer_angle_to_servo_gain_)
      || !getParam(nh, "vesc/steering_angle_to_servo_offset", drive_steer_angle_to_servo_offset_)
      || !getParam(nh, "laser/frame_id", sensor_frame_id_)
      || !getParam(nh, "laser/range_min", sensor_range_min_)
      || !getParam(nh, "laser/range_max", sensor_range_max_)
      || !getParam(nh, "laser/range_no_obj", sensor_range_no_obj_)
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
  const OccupancyGridMsg& map_msg = get_map_msg.response.map;
  map_width_ = map_msg.info.width;
  map_height_ = map_msg.info.height;
  map_x_origin_ = map_msg.info.origin.position.x;
  map_y_origin_ = map_msg.info.origin.position.y;
  map_th_ = tf2::getYaw(map_msg.info.origin.orientation);
  map_scale_ = map_msg.info.resolution;
  map_data_ = map_msg.data;

  TransformStampedMsg tf_base_to_sensor;
  TransformStampedMsg tf_wheel_bl_to_sensor;

  try {
    tf_base_to_sensor = tf_buffer_.lookupTransform(sensor_frame_id_,
                                                   base_frame_id_,
                                                   ros::Time(0),
                                                   ros::Duration(15.0)
                                                  );
    tf_wheel_bl_to_sensor = tf_buffer_.lookupTransform(sensor_frame_id_,
                                                       wheel_bl_frame_id_,
                                                       ros::Time(0),
                                                       ros::Duration(15.0)
                                                      );
  }
  catch (tf2::TransformException & except) {
    throw std::runtime_error(except.what());
  }

  // Construct the localizer with retrieved parameters before starting threads
  mcl_ptr_ = std::unique_ptr<MCL>(new MCL(num_particles_min_,
                                          num_particles_max_,
                                          car_length_,
                                          tf_base_to_sensor.transform.translation.x,
                                          tf_base_to_sensor.transform.translation.y,
                                          tf2::getYaw(tf_base_to_sensor.transform.rotation),
                                          tf_wheel_bl_to_sensor.transform.translation.x,
                                          tf_base_to_sensor.transform.translation.y,
                                          sensor_range_min_,
                                          sensor_range_max_,
                                          sensor_range_no_obj_,
                                          map_width_,
                                          map_height_,
                                          map_x_origin_,
                                          map_y_origin_,
                                          map_th_,
                                          map_scale_,
                                          map_data_
                                         )
                                 );
  // Assign ROS callback queues
  drive_vel_nh_.setCallbackQueue(&drive_vel_cb_queue_);
  drive_steer_nh.setCallbackQueue(&drive_steer_cb_queue_);
  sensor_nh_.setCallbackQueue(&sensor_cb_queue_);
  status_nh_.setCallbackQueue(&status_cb_queue_);

  // Subscribe to topics for motor, steering servo and sensor data
  drive_vel_sub_ = drive_vel_nh_.subscribe(drive_vel_topic,
                                           1,
                                           &MCLNode::driveVelCb,
                                           this,
                                           ros::TransportHints().tcpNoDelay()
                                          );
  drive_steer_sub_ = drive_steer_nh.subscribe(drive_steer_topic,
                                              1,
                                              &MCLNode::driveSteerCb,
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
  // Setup publisher for pose estimates
  pose_pub_ = pose_nh_.advertise<PoseStampedMsg>(pose_topic, 1);

  // Start threads
  drive_vel_spinner_.start();
  drive_steer_spinner_.start();
  sensor_spinner_.start();
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
  mcl_ptr_->update(RayScanMsg(msg));

  // Publish transform and pose
  publishTf();
  publishPose();

  // Save duration
  sensor_update_time_msec_ = (ros::Time::now() - start).toSec() * 1000.0;
  sensor_update_time_worst_msec_ = sensor_update_time_msec_ > sensor_update_time_worst_msec_?
                                   sensor_update_time_msec_ : sensor_update_time_worst_msec_;
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

void MCLNode::publishTf()
{
  // MCL estimates the transform from the robot base frame to the map frame. In order to complete the transform tree,
  // we need to publish this transform or its inverse.
  //
  // ROS convention unfortunately overloads the usual definition of a transform for the map to odom frame 'transform'.
  // It is comprised of two components:
  //   1) What you expect: A transformation from the (fixed) map frame to the (fixed) odom frame. Note that the odom
  //      frame is located at the robot's initial position, which may be anywhere on the map.
  //   2) What you don't expect: A _correction_ to the odom frame data which, when applied to the odom frame, adjusts
  //      the odometry-derived pose to the 'true' (estimated) pose provided by MCL.
  //
  // Following ROS convention, the transformation from the odom to map frame is determined here by subtracting the
  // robot base to odom transformation (provided by another module) from the robot base to map frame transform we have
  // estimated here.
  //
  // See REP 105 at https://www.ros.org/reps/rep-0105.html
  try {
    TransformStampedMsg tf_base_to_odom = tf_buffer_.lookupTransform(odom_frame_id_,
                                                                     base_frame_id_,
                                                                     ros::Time(0)
                                                                    );
    Particle tf_base_to_map = mcl_ptr_->estimate();

    if (tf_base_to_map.weight_normed_ > 0.0) {
      // Calculate odom to map transformation as delta between known transforms
      double tf_odom_to_map_x = tf_base_to_map.x_ - tf_base_to_odom.transform.translation.x;
      double tf_odom_to_map_y = tf_base_to_map.y_ - tf_base_to_odom.transform.translation.y;
      tf2::Quaternion tf_odom_to_map_orient;
      tf_odom_to_map_orient.setRPY(0.0, 0.0, tf_base_to_map.th_ - tf2::getYaw(tf_base_to_odom.transform.rotation));

      // Create transform message
      TransformStampedMsg tf_odom_to_map;
      tf_odom_to_map.header.stamp = ros::Time::now();
      tf_odom_to_map.header.frame_id = odom_frame_id_;
      tf_odom_to_map.child_frame_id = map_frame_id_;
      tf_odom_to_map.transform.translation.x = tf_odom_to_map_x;
      tf_odom_to_map.transform.translation.y = tf_odom_to_map_y;
      tf_odom_to_map.transform.translation.z = 0.0;
      tf_odom_to_map.transform.rotation.x = tf_odom_to_map_orient.x();
      tf_odom_to_map.transform.rotation.y = tf_odom_to_map_orient.y();
      tf_odom_to_map.transform.rotation.z = tf_odom_to_map_orient.z();
      tf_odom_to_map.transform.rotation.w = tf_odom_to_map_orient.w();

      // Broadcast transform
      // tf_broadcaster_.sendTransform(tf_odom_to_map); // TBD add sim/real parameter to launch and use here
    }
  }
  catch (tf2::TransformException & except) {
    ROS_WARN("MCL: %s", except.what());
  }
}

void MCLNode::publishPose()
{
  // Get estimate
  Particle particle = mcl_ptr_->estimate();

  if (particle.weight_normed_ > 0.0) {
    // Convert heading angle to quaternion
    tf2::Quaternion particle_orient;
    particle_orient.setRPY(0.0, 0.0, particle.th_);

    // Construct message
    PoseStampedMsg pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = map_frame_id_;
    pose_msg.pose.position.x = particle.x_;
    pose_msg.pose.position.y = particle.y_;
    pose_msg.pose.position.z = 0.0;
    pose_msg.pose.orientation.x = particle_orient.x();
    pose_msg.pose.orientation.y = particle_orient.y();
    pose_msg.pose.orientation.z = particle_orient.z();
    pose_msg.pose.orientation.w = particle_orient.w();

    // Publish pose
    pose_pub_.publish(pose_msg);
  }
}

void MCLNode::statusCb(const ros::TimerEvent& event)
{
  printMotionUpdateTime(0.5);
  printSensorUpdateTime(0.5);
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

void MCLNode::printMotionParams()
{
  ROS_INFO("MCL: car_length = %f", car_length_);
  ROS_INFO("MCL: drive_vel_to_erpm_gain = %f", drive_vel_to_erpm_gain_);
  ROS_INFO("MCL: drive_vel_to_erpm_offset = %f", drive_vel_to_erpm_offset_);
  ROS_INFO("MCL: drive_steer_angle_to_servo_gain = %f", drive_steer_angle_to_servo_gain_);
  ROS_INFO("MCL: drive_steer_angle_to_servo_offset = %f", drive_steer_angle_to_servo_offset_);
}

void MCLNode::printSensorParams()
{
  ROS_INFO("MCL: sensor_range_min = %f", sensor_range_min_);
  ROS_INFO("MCL: sensor_range_max = %f", sensor_range_max_);
  ROS_INFO("MCL: sensor_range_no_obj = %f", sensor_range_no_obj_);
}

void MCLNode::printMapParams()
{
  ROS_INFO("MCL: map_width = %d", map_width_);
  ROS_INFO("MCL: map_height_= %d", map_height_);
  ROS_INFO("MCL: map_x_origin = %f", map_x_origin_);
  ROS_INFO("MCL: map_y_origin = %f", map_y_origin_);
  ROS_INFO("MCL: map_th = %f", map_th_);
  ROS_INFO("MCL: map_scale = %f", map_scale_);
}