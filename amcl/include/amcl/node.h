#ifndef AMCL_NODE_H
#define AMCL_NODE_H

#include <memory>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/publisher.h>
#include <ros/service.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/exceptions.h>
#include <tf2/utils.h>

#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/GetMap.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <vesc_msgs/VescStateStamped.h>

#include "amcl/common.h"
#include "amcl/amcl.h"

namespace localize
{
  using DriveStateStampedMsg = vesc_msgs::VescStateStamped;
  using DriveSteerMsg = std_msgs::Float64;
  using SensorScanMsg = sensor_msgs::LaserScan;
  using OccupancyGridMsg = nav_msgs::OccupancyGrid;
  using PoseMsg = geometry_msgs::Pose;
  using PoseStampedMsg = geometry_msgs::PoseStamped;
  using PoseArrayMsg = geometry_msgs::PoseArray;
  using TransformStampedMsg = geometry_msgs::TransformStamped;

  // Convert a particle to a pose message
  PoseMsg poseMsg(const Particle& particle);

  // Retrieve the desired parameter value from the ROS parameter server
  template <class T>
  bool getParam(const ros::NodeHandle& nh,
                std::string name,
                T& val
               )
  {
    bool result = true;

    if (!nh.getParam(name, val)) {
      ROS_FATAL("AMCL: Parameter '%s' not found", name.c_str());
      result = false;
    }
    return result;
  }

  class AMCLNode
  {
  public:
    AMCLNode(const std::string& amcl_node_name,             // AMCL node name
             const std::string& amcl_pose_topic_name,       // AMCL pose topic name
             const std::string& amcl_pose_array_topic_name, // AMCL pose array topic name
             const std::string& drive_node_name,            // Drive node name
             const std::string& drive_vel_topic_name,       // Drive velocity topic name
             const std::string& drive_steer_topic_name,     // Drive steering topic name
             const std::string& sensor_node_name,           // Sensor node name
             const std::string& sensor_topic_name           // Sensor topic name
            );

    // Drive velocity state callback
    void driveVelCb(const DriveStateStampedMsg::ConstPtr& drive_msg);

    // Drive steering state callback
    void driveSteerCb(const DriveSteerMsg::ConstPtr& drive_msg);

    // Sensor scan callback
    void sensorCb(const SensorScanMsg::ConstPtr& sensor_msg);

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
    std::string car_base_frame_id_;
    std::string car_wheel_back_left_frame_id_;
    std::string sensor_frame_id_;
    std::string odom_frame_id_;
    std::string map_frame_id_;

    ros::NodeHandle pose_nh_;
    ros::NodeHandle pose_array_nh_;
    ros::NodeHandle drive_vel_nh_;
    ros::NodeHandle drive_steer_nh_;
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

    PoseStampedMsg pose_stamped_msg_;
    PoseArrayMsg pose_array_msg_;

    // Localizer parameters
    std::unique_ptr<AMCL> amcl_ptr_;        // AMCL localizer
    ParticleVector amcl_estimates_;         // AMCL estimates
    double amcl_update_rate_;               // AMCL update rate (hz)
    bool amcl_use_modified_map_;            // AMCL will localize within an alternate map (for testing a dynamic environments)
    int amcl_num_particles_min_;            // AMCL minimum number of particles
    int amcl_num_particles_max_local_;      // AMCL maximum number of particles during local tracking
    int amcl_num_particles_max_global_;     // AMCL maximum number of particles during global localization
    double amcl_weight_avg_random_sample_;  // AMCL weight average below which random sampling is enabled
    double amcl_weight_rel_dev_resample_;   // AMCL weight relative standard deviation above which resampling is performed

    // Motion model parameters
    double car_length_;                         // Car length
    double drive_vel_to_erpm_gain_;             // Gain for converting motor velocity to electrical rpm
    double drive_vel_to_erpm_offset_;           // Bias for converting motor velocity to electrical rpm
    double drive_steer_angle_to_servo_gain_;    // Gain for converting steering angle to servo position
    double drive_steer_angle_to_servo_offset_;  // Bias for converting steering angle to servo position
    double drive_steer_servo_pos_;              // Steering servo position
    double motion_vel_lin_n1_;                  // Motion model linear velocity noise coefficient 1
    double motion_vel_lin_n2_;                  // Motion model linear velocity noise coefficient 2
    double motion_vel_ang_n1_;                  // Motion model angular velocity noise coefficient 1
    double motion_vel_ang_n2_;                  // Motion model angular velocity noise coefficient 2
    double motion_th_n1_;                       // Motion model final rotation noise coefficient 1
    double motion_th_n2_;                       // Motion model final rotation noise coefficient 2
    double motion_vel_ang_bias_scale_;          // Motion model slip scale factor: decrease angular velocity according to scale * v^2 / r
    double motion_update_time_msec_;            // Motion update last time (milliseconds)
    double motion_update_time_worst_msec_;      // Motion update worst time (milliseconds)

    // Sensor model parameters
    float sensor_range_min_;                    // Sensor min range in meters
    float sensor_range_max_;                    // Sensor max range in meters
    float sensor_range_no_obj_;                 // Sensor range reported when nothing is detected
    float sensor_range_std_dev_;                // Sensor model range measurement standard deviation
    float sensor_decay_rate_new_obj_;           // Sensor model decay rate for new / unexpected object probability
    double sensor_weight_no_obj_;               // Sensor model weight for no object detected probability
    double sensor_weight_new_obj_;              // Sensor model weight for new / unexpected object probability
    double sensor_weight_map_obj_;              // Sensor model weight for mapped / expected object probability
    double sensor_weight_rand_effect_;          // Sensor model weight for random effect probability
    double sensor_weight_uncertainty_factor_;   // Sensor model weight uncertainty factor (extra noise added to final weight)
    double sensor_prob_new_obj_reject_;         // Sensor model probability above which a ray is rejected for representing a new / unexpected object
    double sensor_update_time_msec_;            // Sensor update last time (milliseconds)
    double sensor_update_time_worst_msec_;      // Sensor update worst time (milliseconds)
    RayScan sensor_scan_;                       // Sensor scan data
  };

} // namespace localize

#endif // AMCL_NODE_H