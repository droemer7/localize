#include "mcl/mcl_node.h"

using namespace localize;

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "localizer");

  try {
    // Start MCL node
    MCLNode mcl_node("vesc/sensors/core",
                     "vesc/sensors/servo_position_command",
                     "scan",
                     "/static_map"
                    );

    // Run node until ROS is shutdown
    ros::waitForShutdown();
  }
  catch (std::runtime_error error) {
    printf("%s\n", error.what());
  }

  return 0;
}