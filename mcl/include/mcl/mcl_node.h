#ifndef MCL_NODE_H
#define MCL_NODE_H

#include <memory>
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
    MCLNode(const std::string& motor_topic,   // Motor topic name
            const std::string& servo_topic,   // Steering servo topic name
            const std::string& sensor_topic,  // Sensor topic name
            const std::string& map_topic      // Map topic name
           );

    // Motor state callback
    void motorCb(const vesc_msgs::VescStateStamped::ConstPtr& msg);

    // Servo state callback
    void servoCb(const std_msgs::Float64::ConstPtr& msg);

    // Sensor scan callback
    void sensorCb(const sensor_msgs::LaserScan::ConstPtr& msg);

    // Retrieves the desired parameter value from the ROS parameter server
    template <class T>
    bool getParam(const ros::NodeHandle& nh,
                  std::string name,
                  T& value
                 );

    // Diagnostics - print Motion Model parameters
    void printMotionParams();

    // Diagnostics - print Sensor Model parameters
    void printSensorParams();

    // Diagnostics - print Map parameters
    void printMapParams();

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
    ros::Time prev_t_;

    // MCL
    std::unique_ptr<MCL> mcl_ptr_;
    double num_particles_;  // Number of particles

    // Motion Model parameters
    double car_length_;                           // Car length
    double motor_speed_to_erpm_gain_;             // Gain for converting motor velocity to electrical RPM (ERPM)
    double motor_speed_to_erpm_offset_;           // Bias for converting motor velocity to electrical RPM (ERPM)
    double motor_steering_angle_to_servo_gain_;   // Gain for converting steering angle to servo position
    double motor_steering_angle_to_servo_offset_; // Bias for converting steering angle to servo position
    double servo_pos_;                            // Steering servo position
    double motion_lin_vel_n1_;                    // Motion model linear velocity noise coefficient 1
    double motion_lin_vel_n2_;                    // Motion model linear velocity noise coefficient 2
    double motion_ang_vel_n1_;                    // Motion model angular velocity noise coefficient 1
    double motion_ang_vel_n2_;                    // Motion model angular velocity noise coefficient 2
    double motion_th_n1_;                         // Motion model final rotation noise coefficient 1
    double motion_th_n2_;                         // Motion model final rotation noise coefficient 2

    // Sensor Model parameters
    double sensor_range_min_;           // Sensor min range in meters
    double sensor_range_max_;           // Sensor max range in meters
    double sensor_range_no_obj_;        // Sensor range reported when nothing is detected
    double sensor_range_std_dev_;       // Sensor standard deviation
    double sensor_new_obj_decay_rate_;  // Sensor model decay rate for unexpected object probability
    double sensor_weight_no_obj_;       // Sensor model weight for no object detected probability
    double sensor_weight_new_obj_;      // Sensor model weight for new (unexpected) object probability
    double sensor_weight_map_obj_;      // Sensor model weight for map (expected) object probability
    double sensor_weight_rand_effect_;  // Sensor model weight for random effect probability

    // Map parameters
    unsigned int map_width_;            // Map width
    unsigned int map_height_;           // Map height
    float map_m_per_pxl_;               // Map resolution (meters per pixel)
    double map_th_;                     // Map angle
    double map_origin_x_;               // Map origin x position
    double map_origin_y_;               // Map origin y position
    std::vector<int8_t> map_occ_data_;  // Map occupancy data in 1D vector, -1: Unknown, 0: Free, 100: Occupied

    std::mutex servo_mtx_;
  };

} // namespace localize

#endif // MCL_NODE_H