
#include <sys/resource.h>

#include "mcl/node.h"

using namespace localize;

// TBD remove stack checks
int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "localizer");
  ros::NodeHandle nh;

  try {
    // Start MCL node
    MCLNode mcl_node("vesc/sensors/core",
                     "vesc/sensors/servo_position_command",
                     "laser/scan",
                     "/static_map"
                    );
    // Stack usage
    rusage stackusage;
    getrusage(RUSAGE_SELF, &stackusage);
    ROS_INFO("MCL: Memory usage = %ld MB", stackusage.ru_maxrss / 1024);

    // Run node until ROS is shutdown
    ros::waitForShutdown();
  }
  catch (std::runtime_error error) {
    ROS_FATAL("%s", error.what());

    return 1;
  }
  return 0;
}