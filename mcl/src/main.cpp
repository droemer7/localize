#include "mcl/node.h"

using namespace localize;

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "localizer");
  ros::NodeHandle nh;

  try {
    // Start MCL node
    MCLNode mcl_node("pose",
                     "vesc/sensors/core",
                     "vesc/sensors/servo_position_command",
                     "laser/scan",
                     "/static_map"
                    );
    // Run MCL node until ROS is shutdown
    ros::waitForShutdown();
  }
  catch (std::runtime_error & except) {
    ROS_FATAL("%s", except.what());

    return 1;
  }
  return 0;
}