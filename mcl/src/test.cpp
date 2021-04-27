#include <chrono>
#include <float.h>
#include <stdio.h>

#include <tf2_ros/transform_listener.h>

#include "mcl/mcl.h"
#include "mcl/sensor.h"
#include "mcl/common.h"

#include "includes/RangeLib.h"

#define DEG2RAD(x) ((x) * L_PI / 180.0)
#define RAD2DEG(x) ((x) * 180.0 / L_PI)

using namespace localize;

std::chrono::_V2::high_resolution_clock::time_point start;
std::chrono::_V2::high_resolution_clock::time_point end;
std::chrono::duration<double> dur;

void testAngleUtils()
{
  double top = -179.0 * L_PI / 180.0;
  double bot = 175.0 * L_PI / 180.0;
  printf("th_delta = %.4f\n", angleDelta(top, bot) * 180.0 / L_PI);

  top = 230.0 * L_PI / 180.0;
  bot = -10.0 * L_PI / 180.0;
  printf("\n");
  printf("th_delta = %.4f\n", angleDelta(top, bot) * 180.0 / L_PI);

  top = -175.0 * L_PI / 180.0;
  bot = 150.0 * L_PI / 180.0;
  printf("\n");
  printf("th_delta = %.4f\n", angleDelta(top, bot) * 180.0 / L_PI);

  top = 0.0 * L_PI / 180.0;
  bot = -10.0 * L_PI / 180.0;
  size_t top_count = 1;
  size_t bot_count = 3;

  printf("\n");
  printf("th_delta = %.4f\n", angleDelta(top, bot) * 180.0 / L_PI);
  printf("th_avg = %.4f\n", wrapAngle(top + angleDelta(top, bot) * bot_count / (bot_count + top_count)) * 180.0 / L_PI);
}

int main(int argc, char** argv)
{
  // double car_length = 0.3;
  // double car_width = 0.225 + 2 * 0.02225;
  // double radius_rear_meas = car_length / std::tan(0.34) - car_width;
  // printf("radius_rear_meas = %.4f (inches)\n", radius_rear_meas * 100.0 / 2.54);

  printf("\n");
  Pose map(2.144, 1.719, DEG2RAD(35.744)); // map(3,3,DEG2RAD(80));
  Pose odom(0.163, 0.0, DEG2RAD(-0.001));  // odom(1,1,DEG2RAD(45));
  Pose map_to_odom;

  // Translate
  map_to_odom.x_ = odom.x_;
  map_to_odom.y_ = odom.y_;
  map_to_odom.th_ = odom.th_ - map.th_;

  // Rotate
  map_to_odom.x_ -= map.x_ * std::cos(map_to_odom.th_) - map.y_ * std::sin(map_to_odom.th_);
  map_to_odom.y_ -= map.x_ * std::sin(map_to_odom.th_) + map.y_ * std::cos(map_to_odom.th_);

  printf("Transform:\n");
  printf("x = %.2f\n", map_to_odom.x_);             // x = -3.18
  printf("y = %.2f\n", map_to_odom.y_);             // y = 0.26
  printf("th = %.2f\n", RAD2DEG(map_to_odom.th_));  // th = -35.00
  printf("==========================\n");

  // Rotate
  double temp_map_x = map.x_;
  map.x_ = temp_map_x * std::cos(map_to_odom.th_) - map.y_ * std::sin(map_to_odom.th_);
  map.y_ = temp_map_x * std::sin(map_to_odom.th_) + map.y_ * std::cos(map_to_odom.th_);

  // Translate
  map.x_ += map_to_odom.x_;
  map.y_ += map_to_odom.y_;
  map.th_ += map_to_odom.th_;

  printf("Map point in odom:\n");
  printf("x = %.2f\n", map.x_);             // x = 1.00
  printf("y = %.2f\n", map.y_);             // y = 1.00
  printf("th = %.2f\n", RAD2DEG(map.th_));  // th = 45.00
  printf("==========================\n");

  /*
  Pose map(2.144, 1.719, DEG2RAD(35.744)); // map(3,3,DEG2RAD(80));
  Pose odom(0.163, 0.0, DEG2RAD(-0.001));  // odom(1,1,DEG2RAD(45));
  Pose map_to_odom;

  // Rotate
  map_to_odom.th_ = odom.th_ - map.th_;
  map_to_odom.x_ = odom.x_ * std::cos(map_to_odom.th_) + odom.y_ * std::sin(map_to_odom.th_);
  map_to_odom.y_ = -odom.x_ * std::sin(map_to_odom.th_) + odom.y_ * std::cos(map_to_odom.th_);

  // Translate
  map_to_odom.x_ -= map.x_;
  map_to_odom.y_ -= map.y_;

  printf("Transform (reverse?):\n");
  printf("x = %.2f\n", map_to_odom.x_);
  printf("y = %.2f\n", map_to_odom.y_);
  printf("th = %.2f\n", RAD2DEG(map_to_odom.th_));
  printf("==========================\n");

  map = Pose(3,3, DEG2RAD(80.0));

  // Translate
  map.x_ += map_to_odom.x_;
  map.y_ += map_to_odom.y_;
  map.th_ += map_to_odom.th_;

  // Rotate
  double temp_map_x = map.x_;
  map.x_ = temp_map_x * std::cos(map_to_odom.th_) - map.y_ * std::sin(map_to_odom.th_);
  map.y_ = temp_map_x * std::sin(map_to_odom.th_) + map.y_ * std::cos(map_to_odom.th_);

  printf("Map point in odom:\n");
  printf("x = %.2f\n", map.x_);
  printf("y = %.2f\n", map.y_);
  printf("th = %.2f\n", RAD2DEG(map.th_));
  printf("==========================\n");
  */

  return 0;
}