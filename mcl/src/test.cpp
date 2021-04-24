#include <chrono>
#include <float.h>
#include <stdio.h>

#include <tf2_ros/transform_listener.h>

#include "mcl/mcl.h"
#include "mcl/sensor.h"
#include "mcl/common.h"

#include "includes/RangeLib.h"

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
  double car_length = 0.3;
  double radius_r = car_length / std::tan(0.34);

  double beta = std::atan2(0.158 * std::tan(0.34), car_length);
  double radius_cg = car_length / (std::cos(beta) * std::tan(0.34));

  printf("radius_f = %.4f (inches)\n", car_length / std::sin(0.34) * 100.0 / 2.54);
  printf("radius_cg = %.4f (inches)\n", radius_cg * 100.0 / 2.54);
  printf("radius_r = %.4f (inches)\n", radius_r * 100.0 / 2.54);

  return 0;
}