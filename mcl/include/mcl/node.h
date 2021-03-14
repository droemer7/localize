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
#include "mcl/util.h"

namespace localize
{
  class RayScanMsg : public RayScan
  {
  public:
    // Constructors
    RayScanMsg(const sensor_msgs::LaserScan::ConstPtr& msg);
  };

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

    // Status info callback
    void statusCb(const ros::TimerEvent& event);

    // Save particles to file
    void save();

    // Retrieves the desired parameter value from the ROS parameter server
    template <class T>
    bool getParam(const ros::NodeHandle& nh,
                  std::string name,
                  T& value
                 );

    // Print Motion model parameters
    void printMotionParams();

    // Print Sensor model parameters
    void printSensorParams();

    // Print Map parameters
    void printMapParams();

  private:
    std::mutex servo_mtx_;  // Servo mutex

    // ROS interface
    ros::NodeHandle motor_nh_;
    ros::NodeHandle servo_nh_;
    ros::NodeHandle sensor_nh_;
    ros::NodeHandle status_nh_;
    ros::CallbackQueue motor_cb_queue_;
    ros::CallbackQueue servo_cb_queue_;
    ros::CallbackQueue sensor_cb_queue_;
    ros::CallbackQueue status_cb_queue_;
    ros::Subscriber motor_sub_;
    ros::Subscriber servo_sub_;
    ros::Subscriber sensor_sub_;
    ros::AsyncSpinner motor_spinner_;
    ros::AsyncSpinner servo_spinner_;
    ros::AsyncSpinner sensor_spinner_;
    ros::AsyncSpinner status_spinner_;
    ros::Duration timer_cb_dur_;
    ros::Time motor_t_prev_;
    ros::Timer status_timer_;

    // MCL
    std::unique_ptr<MCL> mcl_ptr_;

    // MCL parameters
    double mcl_num_particles_min_;  // Minimum number of particles
    double mcl_num_particles_max_;  // Maximum number of particles
    double mcl_kld_eps_;            // KL distance threshold
    double mcl_hist_pos_res_;       // Histogram bin size for x and y position
    double mcl_hist_th_res_;        // Histogram bin size for heading angle

    // Motion model parameters
    double car_length_;                           // Car length
    double motor_speed_to_erpm_gain_;             // Gain for converting motor velocity to electrical  ()
    double motor_speed_to_erpm_offset_;           // Bias for converting motor velocity to electrical  ()
    double motor_steering_angle_to_servo_gain_;   // Gain for converting steering angle to servo position
    double motor_steering_angle_to_servo_offset_; // Bias for converting steering angle to servo position
    double servo_pos_;                            // Steering servo position
    double motion_lin_vel_n1_;                    // Motion model linear velocity noise coefficient 1
    double motion_lin_vel_n2_;                    // Motion model linear velocity noise coefficient 2
    double motion_ang_vel_n1_;                    // Motion model angular velocity noise coefficient 1
    double motion_ang_vel_n2_;                    // Motion model angular velocity noise coefficient 2
    double motion_th_n1_;                         // Motion model final rotation noise coefficient 1
    double motion_th_n2_;                         // Motion model final rotation noise coefficient 2
    double motion_dur_msec_;                      // Motion model last update time (milliseconds)
    double motion_dur_worst_msec_;                // Motion model worst update time (milliseconds)

    // Sensor model parameters
    float sensor_range_min_;            // Sensor min range in meters
    float sensor_range_max_;            // Sensor max range in meters
    float sensor_range_no_obj_;         // Sensor range reported when nothing is detected
    float sensor_range_std_dev_;        // Sensor range standard deviation
    float sensor_th_sample_res_;        // Sensor angle resolution at which to sample observations (rad per sample)
    float sensor_th_raycast_res_;       // Sensor angle resolution for raycast (rad per increment)
    float sensor_new_obj_decay_rate_;   // Sensor model decay rate for new (unexpected) object probability
    double sensor_weight_no_obj_;       // Sensor model weight for no object detected probability
    double sensor_weight_new_obj_;      // Sensor model weight for new (unexpected) object probability
    double sensor_weight_map_obj_;      // Sensor model weight for map (expected) object probability
    double sensor_weight_rand_effect_;  // Sensor model weight for random effect probability
    double sensor_uncertainty_factor_;  // Sensor model uncertainty factor - extra noise added to calculation
    double sensor_table_res_;           // Sensor model table resolution (meters per cell)
    double sensor_dur_msec_;            // Sensor model last update time (milliseconds)
    double sensor_dur_worst_msec_;      // Sensor model worst update time (milliseconds)

    // Map parameters
    unsigned int map_width_;        // Map number of pixels along x axis
    unsigned int map_height_;       // Map number of pixels along y axis
    float map_x_origin_;            // Map x translation of origin (cell 0,0) relative to world frame (meters)
    float map_y_origin_;            // Map y translation of origin (cell 0,0) relative to world frame (meters)
    float map_th_;                  // Map angle relative to world frame (rad)
    float map_scale_;               // Map scale relative to world frame (meters per pixel)
    std::vector<int8_t> map_data_;  // Map occupancy data in 1D vector, -1: Unknown, 0: Free, 100: Occupied
  };

} // namespace localize

#endif // MCL_NODE_H