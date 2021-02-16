#ifndef MCL_NODE_H
#define MCL_NODE_H

#include <mutex>
#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>
#include <ros/subscriber.h>

#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Float64.h>
#include <vesc_msgs/VescStateStamped.h>

#include "mcl/mcl.h"

namespace localize
{
  class MCLNode
  {
  public:
    MCLNode(const std::string& motor_topic,
            const std::string& servo_topic,
            const std::string& sensor_topic,
            const std::string& map_topic
           );

    // MCL
    MCL & mcl();

    // Motor state callback
    void motorCb(const vesc_msgs::VescStateStamped::ConstPtr& msg);

    // Servo state callback
    void servoCb(const std_msgs::Float64::ConstPtr& msg);

    // Sensor scan callback
    void sensorCb(const sensor_msgs::LaserScan::ConstPtr& msg);

  private:
    // ROS interface
    ros::NodeHandle motor_nh_;
    ros::NodeHandle servo_nh_;
    ros::NodeHandle sensor_nh_;
    ros::CallbackQueue motor_cb_queue_;
    ros::CallbackQueue servo_cb_queue_;
    ros::CallbackQueue sensor_cb_queue_;
    ros::Subscriber motor_sub_;
    ros::Subscriber servo_sub_;
    ros::Subscriber sensor_sub_;
    ros::AsyncSpinner motor_spinner_;
    ros::AsyncSpinner servo_spinner_;
    ros::AsyncSpinner sensor_spinner_;
    ros::Time curr_t_;
    ros::Time prev_t_;
    std::mutex servo_mtx_;

    // MCL
    double num_particles_;  // Number of particles
    // Motion model
    double car_length_;                           // Car length
    double motor_speed_to_erpm_gain_;             // Gain for converting from velocity to electrical RPM (ERPM)
    double motor_speed_to_erpm_offset_;           // Bias for converting from velocity to electrical RPM (ERPM)
    double motor_steering_angle_to_servo_gain_;   // Gain for converting from steering angle to Servo position
    double motor_steering_angle_to_servo_offset_; // Bias for converting from steering angle to Servo position
    double servo_pos_;                            // Steering servo position
    double motion_lin_vel_n1_;                    // Linear velocity noise coefficient 1
    double motion_lin_vel_n2_;                    // Linear velocity noise coefficient 2
    double motion_ang_vel_n1_;                    // Angular velocity noise coefficient 1
    double motion_ang_vel_n2_;                    // Angular velocity noise coefficient 2
    double motion_th_n1_;                         // Final rotation noise coefficient 1
    double motion_th_n2_;                         // Final rotation noise coefficient 2

    // Sensor model
    double sensor_range_min_;           // Sensor max range in meters
    double sensor_range_max_;           // Sensor max range in meters
    double sensor_range_no_obj_;        // Range reported by the sensor when nothing is detected
    double sensor_range_std_dev_;       // Sensor standard deviation
    double sensor_new_obj_decay_rate_;  // Decay rate for unexpected object probability
    double sensor_weight_no_obj_;       // Sensor weight for no object detected probability
    double sensor_weight_new_obj_;      // Sensor weight for new (unexpected) object probability
    double sensor_weight_map_obj_;      // Sensor weight for map (expected) object probability
    double sensor_weight_rand_effect_;  // Sensor weight for random effect probability

    // Map
    uint32_t map_height_;               // Map height
    uint32_t map_width_;                // Map width
    float map_m_per_pxl_;               // Map resolution (meters per pixel)
    double map_th_;                     // Map angle
    double map_origin_x_;               // Map origin x position
    double map_origin_y_;               // Map origin y position
    std::vector<int8_t> map_occ_data_;  // Map occupancy data in 1D vector, -1: Unknown, 0: Free, 100: Occupied
  };

} // namespace localize

#endif // MCL_NODE_H