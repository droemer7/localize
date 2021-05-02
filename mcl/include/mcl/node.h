#ifndef MCL_NODE_H
#define MCL_NODE_H

#include <memory>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/publisher.h>
#include <ros/service.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <vesc_msgs/VescStateStamped.h>

#include "mcl/common.h"
#include "mcl/mcl.h"

namespace localize
{
  typedef vesc_msgs::VescStateStamped DriveStateStampedMsg;
  typedef std_msgs::Float64 DriveSteerMsg;
  typedef sensor_msgs::LaserScan SensorScanMsg;
  typedef nav_msgs::OccupancyGrid OccupancyGridMsg;
  typedef geometry_msgs::Pose PoseMsg;
  typedef geometry_msgs::PoseStamped PoseStampedMsg;
  typedef geometry_msgs::PoseArray PoseArrayMsg;
  typedef geometry_msgs::TransformStamped TransformStampedMsg;

  PoseMsg poseMsg(const Particle& particle);

  RayScan rayScan(const SensorScanMsg::ConstPtr& msg);

  // Retrieve the desired parameter value from the ROS parameter server
  template <class T>
  bool getParam(const ros::NodeHandle& nh,
                std::string name,
                T& val
               )
  {
    bool result = true;

    if (!nh.getParam(name, val)) {
      ROS_FATAL("MCL: Parameter '%s' not found", name.c_str());
      result = false;
    }
    return result;
  }

  class MCLNode
  {
  public:
    MCLNode(const std::string& localizer_node_name,             // Localizer node name
            const std::string& localizer_pose_topic_name,       // Localizer pose topic name
            const std::string& localizer_pose_array_topic_name, // Localizer pose array topic name
            const std::string& drive_node_name,                 // Drive node name
            const std::string& drive_vel_topic_name,            // Drive velocity topic name
            const std::string& drive_steer_topic_name,          // Drive steering topic name
            const std::string& sensor_node_name,                // Sensor node name
            const std::string& sensor_topic_name                // Sensor topic name
           );

    // Drive velocity state callback
    void driveVelCb(const DriveStateStampedMsg::ConstPtr& msg);

    // Drive steering state callback
    void driveSteerCb(const DriveSteerMsg::ConstPtr& msg);

    // Sensor scan callback
    void sensorCb(const SensorScanMsg::ConstPtr& msg);

    // State estimate callback
    void estimateCb(const ros::TimerEvent& event);

    // Status info callback
    void statusCb(const ros::TimerEvent& event);

  private:
    // Publish the transform estimated by the localizer
    void publishTf();

    // Publish the best pose estimate from the localizer
    void publishPose();

    // Publish all pose estimates from the localizer
    void publishPoseArray();

    // Print Motion update times (last and worst)
    void printMotionUpdateTime(const double min_msec = 0.0);

    // Print Sensor update times (last and worst)
    void printSensorUpdateTime(const double min_msec = 0.0);

  private:
    Mutex drive_steer_servo_pos_mtx_;  // Servo mutex

    // ROS interface
    std::string base_frame_id_;
    std::string wheel_back_left_frame_id_;
    std::string sensor_frame_id_;
    std::string odom_frame_id_;
    std::string map_frame_id_;

    ros::NodeHandle pose_nh_;
    ros::NodeHandle pose_array_nh_;
    ros::NodeHandle drive_vel_nh_;
    ros::NodeHandle drive_steer_nh;
    ros::NodeHandle sensor_nh_;
    ros::NodeHandle estimate_nh_;
    ros::NodeHandle status_nh_;

    ros::CallbackQueue drive_vel_cb_queue_;
    ros::CallbackQueue drive_steer_cb_queue_;
    ros::CallbackQueue sensor_cb_queue_;
    ros::CallbackQueue estimate_cb_queue_;
    ros::CallbackQueue status_cb_queue_;

    ros::Publisher pose_pub_;
    ros::Publisher pose_array_pub_;
    ros::Subscriber drive_vel_sub_;
    ros::Subscriber drive_steer_sub_;
    ros::Subscriber sensor_sub_;

    ros::AsyncSpinner drive_vel_spinner_;
    ros::AsyncSpinner drive_steer_spinner_;
    ros::AsyncSpinner sensor_spinner_;
    ros::AsyncSpinner estimate_spinner_;
    ros::AsyncSpinner status_spinner_;

    ros::Time drive_t_prev_;
    ros::Timer estimate_timer_;
    ros::Timer status_timer_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // MCL
    std::unique_ptr<MCL> mcl_ptr_;
    bool real_;               // Running on real robot
    bool load_map_modified_;  // Serve the localizer with an alternate map for testing a dynamic environment
    int num_particles_min_;   // Minimum number of particles to use
    int num_particles_max_;   // Maximum number of particles to use

    // Motion model parameters
    double car_length_;                         // Car length
    double drive_vel_to_erpm_gain_;             // Gain for converting motor velocity to electrical rpm
    double drive_vel_to_erpm_offset_;           // Bias for converting motor velocity to electrical rpm
    double drive_steer_angle_to_servo_gain_;    // Gain for converting steering angle to servo position
    double drive_steer_angle_to_servo_offset_;  // Bias for converting steering angle to servo position
    double drive_steer_servo_pos_;              // Steering servo position
    double motion_update_time_msec_;            // Motion update last time (milliseconds)
    double motion_update_time_worst_msec_;      // Motion update worst time (milliseconds)

    // Sensor model parameters
    float sensor_range_min_;                // Sensor min range in meters
    float sensor_range_max_;                // Sensor max range in meters
    float sensor_range_no_obj_;             // Sensor range reported when nothing is detected
    double sensor_update_time_msec_;        // Sensor update last time (milliseconds)
    double sensor_update_time_worst_msec_;  // Sensor update worst time (milliseconds)
  };

} // namespace localize

#endif // MCL_NODE_H