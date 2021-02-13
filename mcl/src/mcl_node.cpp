#include <string>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/spinner.h>

#include "mcl/mcl.h"

using namespace localize;

int main(int argc, char** argv)
{
  // Start ROS node
  ros::init(argc, argv, "localizer");
  ros::NodeHandle nh;

  // MCL parameters
  double num_particles;

  // Motion model parameters
  double car_length;
  double motor_speed_to_erpm_gain;
  double motor_speed_to_erpm_offset;
  double motor_steering_angle_to_servo_gain;
  double motor_steering_angle_to_servo_offset;
  double motion_lin_vel_n1;
  double motion_lin_vel_n2;
  double motion_ang_vel_n1;
  double motion_ang_vel_n2;
  double motion_th_n1;
  double motion_th_n2;

  // Sensor model parameters
  double sensor_range_min;
  double sensor_range_max;
  double sensor_range_no_obj;
  double sensor_range_std_dev;
  double sensor_new_obj_decay_rate;
  double sensor_weight_no_obj;
  double sensor_weight_new_obj;
  double sensor_weight_map_obj;
  double sensor_weight_rand_effect;

 if (   !getParam(nh, "localizer/num_particles", num_particles)
     || !getParam(nh, "vesc/chassis_length", car_length)
     || !getParam(nh, "vesc/speed_to_erpm_gain", motor_speed_to_erpm_gain)
     || !getParam(nh, "vesc/speed_to_erpm_offset", motor_speed_to_erpm_offset)
     || !getParam(nh, "vesc/steering_angle_to_servo_gain", motor_steering_angle_to_servo_gain)
     || !getParam(nh, "vesc/steering_angle_to_servo_offset", motor_steering_angle_to_servo_offset)
     || !getParam(nh, "localizer/motion_lin_vel_n1", motion_lin_vel_n1)
     || !getParam(nh, "localizer/motion_lin_vel_n2", motion_lin_vel_n2)
     || !getParam(nh, "localizer/motion_ang_vel_n1", motion_ang_vel_n1)
     || !getParam(nh, "localizer/motion_ang_vel_n2", motion_ang_vel_n2)
     || !getParam(nh, "localizer/motion_th_n1", motion_th_n1)
     || !getParam(nh, "localizer/motion_th_n2", motion_th_n2)
     || !getParam(nh, "laser/range_min", sensor_range_min)
     || !getParam(nh, "laser/range_max", sensor_range_max)
     || !getParam(nh, "localizer/sensor_range_no_obj", sensor_range_no_obj)
     || !getParam(nh, "localizer/sensor_std_dev", sensor_range_std_dev)
     || !getParam(nh, "localizer/sensor_weight_new_obj_decay_rate", sensor_new_obj_decay_rate)
     || !getParam(nh, "localizer/sensor_weight_no_obj", sensor_weight_no_obj)
     || !getParam(nh, "localizer/sensor_weight_new_obj", sensor_weight_new_obj)
     || !getParam(nh, "localizer/sensor_weight_map_obj", sensor_weight_map_obj)
     || !getParam(nh, "localizer/sensor_weight_rand_effect", sensor_weight_rand_effect)
    ) {
    return 1;
  }

  // Create localizer
  MCL mcl(num_particles,
          VelModel(car_length,
                   motor_speed_to_erpm_gain,
                   motor_speed_to_erpm_offset,
                   motor_steering_angle_to_servo_gain,
                   motor_steering_angle_to_servo_offset,
                   motion_lin_vel_n1,
                   motion_lin_vel_n2,
                   motion_ang_vel_n1,
                   motion_ang_vel_n2,
                   motion_th_n1,
                   motion_th_n2
                  ),
          BeamModel(sensor_range_min,
                    sensor_range_max,
                    sensor_range_no_obj,
                    sensor_range_std_dev,
                    sensor_new_obj_decay_rate,
                    sensor_weight_no_obj,
                    sensor_weight_new_obj,
                    sensor_weight_map_obj,
                    sensor_weight_rand_effect
                   )
         );

  // Setup ROS callback queues and corresponding node handles
  ros::NodeHandle motor_nh;
  ros::NodeHandle servo_nh;
  ros::NodeHandle sensor_nh;
  ros::CallbackQueue motor_cb_queue;
  ros::CallbackQueue servo_cb_queue;
  ros::CallbackQueue sensor_cb_queue;
  motor_nh.setCallbackQueue(&motor_cb_queue);
  servo_nh.setCallbackQueue(&servo_cb_queue);
  sensor_nh.setCallbackQueue(&sensor_cb_queue);

  // Subscribe to vesc and laser data
  std::string motor_topic = "vesc/sensors/core";
  std::string servo_topic = "vesc/sensors/servo_position_command";
  std::string sensor_topic = "scan";
  ros::Subscriber motor_sub = motor_nh.subscribe(motor_topic,
                                                 1,
                                                 &MCL::motorCb,
                                                 &mcl,
                                                 ros::TransportHints().tcpNoDelay()
                                                );
  ros::Subscriber servo_sub = servo_nh.subscribe(servo_topic,
                                                 1,
                                                 &MCL::servoCb,
                                                 &mcl,
                                                 ros::TransportHints().tcpNoDelay()
                                                );
  ros::Subscriber sensor_sub = sensor_nh.subscribe(sensor_topic,
                                                   1,
                                                   &MCL::sensorCb,
                                                   &mcl,
                                                   ros::TransportHints().tcpNoDelay()
                                                  );

  // Dedicate one thread to each callback
  ros::AsyncSpinner motor_spinner(1, &motor_cb_queue);
  ros::AsyncSpinner servo_spinner(1, &servo_cb_queue);
  ros::AsyncSpinner sensor_spinner(1, &sensor_cb_queue);
  motor_spinner.start();
  servo_spinner.start();
  sensor_spinner.start();

  ros::waitForShutdown();

  return 0;
}