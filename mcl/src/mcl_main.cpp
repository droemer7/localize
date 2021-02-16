#include "mcl/mcl_node.h"

using namespace localize;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "localizer");

  MCLNode mcl_node("vesc/sensors/core",
                   "vesc/sensors/servo_position_command",
                   "scan",
                   "/static_map"
                  );

  ros::waitForShutdown();

  return 0;
}