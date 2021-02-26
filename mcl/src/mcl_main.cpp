
#include <sys/resource.h>

#include "mcl/mcl_node.h"

using namespace localize;

// TBD remove stack checks
int main(int argc, char** argv)
{
  rlimit stacklim;
  rusage stackusage;
  getrlimit(RLIMIT_STACK, &stacklim);
  printf("Stack limit (MB): %lu\n", stacklim.rlim_cur / 1024);

  // Initialize ROS
  ros::init(argc, argv, "localizer");
  ros::NodeHandle nh;

  try {
    // Start MCL node
    MCLNode mcl_node("vesc/sensors/core",
                     "vesc/sensors/servo_position_command",
                     "/scan",
                     "/static_map"
                    );
    // Stack usage
    getrusage(RUSAGE_SELF, &stackusage);
    printf("Stack usage (MB): %ld\n", stackusage.ru_maxrss / 1024);

    // Run node until ROS is shutdown
    ros::waitForShutdown();
  }
  catch (std::runtime_error error) {
    printf("%s\n", error.what());
  }

  return 0;
}