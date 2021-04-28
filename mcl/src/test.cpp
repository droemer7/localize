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
  Pose base_to_map(0.938, 0.377, DEG2RAD(56.507));
  Pose map_to_base;

  // Rotate
  map_to_base.x_ = -(base_to_map.x_ * std::cos(-base_to_map.th_) - base_to_map.y_ * std::sin(-base_to_map.th_));
  map_to_base.y_ = -(base_to_map.x_ * std::sin(-base_to_map.th_) + base_to_map.y_ * std::cos(-base_to_map.th_));
  map_to_base.th_ = -base_to_map.th_;

  printf("map_to_base:\n");
  printf("x = %.3f\n", map_to_base.x_);
  printf("y = %.3f\n", map_to_base.y_);
  printf("th = %.3f\n", RAD2DEG(map_to_base.th_));
  printf("==========================\n");

  Pose base_to_odom(0.960, 0.632, DEG2RAD(57.268));
  Pose odom_to_map;

  odom_to_map.x_ = (  base_to_odom.x_
                    + (  map_to_base.x_ * std::cos(base_to_odom.th_)
                       - map_to_base.y_ * std::sin(base_to_odom.th_)
                      )
                   );
  odom_to_map.y_ = (  base_to_odom.y_
                    + (  map_to_base.x_ * std::sin(base_to_odom.th_)
                       + map_to_base.y_ * std::cos(base_to_odom.th_)
                      )
                   );
  odom_to_map.th_ = base_to_odom.th_ + map_to_base.th_;

  printf("odom_to_map:\n");
  printf("x = %.3f\n", odom_to_map.x_);
  printf("y = %.3f\n", odom_to_map.y_);
  printf("th = %.3f\n", RAD2DEG(odom_to_map.th_));
  printf("==========================\n");

  Pose map_to_odom;

  // Rotate
  map_to_odom.x_ = -(odom_to_map.x_ * std::cos(-odom_to_map.th_) - odom_to_map.y_ * std::sin(-odom_to_map.th_));
  map_to_odom.y_ = -(odom_to_map.x_ * std::sin(-odom_to_map.th_) + odom_to_map.y_ * std::cos(-odom_to_map.th_));
  map_to_odom.th_ = -odom_to_map.th_;

  printf("map_to_odom:\n");
  printf("x = %.3f\n", map_to_odom.x_);
  printf("y = %.3f\n", map_to_odom.y_);
  printf("th = %.3f\n", RAD2DEG(map_to_odom.th_));
  printf("==========================\n");

  return 0;
}