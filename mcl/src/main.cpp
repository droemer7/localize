
#include <sys/resource.h>

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
    // Memory usage
    rusage stackusage;
    getrusage(RUSAGE_SELF, &stackusage);
    ROS_INFO("MCL: Memory usage = %ld MB", stackusage.ru_maxrss / 1024);

    // Run node until ROS is shutdown
    ros::waitForShutdown();
  }
  catch (std::runtime_error & except) {
    ROS_FATAL("%s", except.what());

    return 1;
  }
  return 0;
}