#include "mcl/node.h"

using namespace localize;

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "localizer");

  // Get node names
  ros::NodeHandle pnh("~");
  std::string localizer_node_name = pnh.param("localizer_node_name", std::string("localizer"));
  std::string drive_node_name = pnh.param("drive_node_name", std::string("vesc"));
  std::string sensor_node_name = pnh.param("sensor_node_name", std::string("laser"));

  try {
    // Start localizer node
    MCLNode mcl_node(localizer_node_name,
                     localizer_node_name + "/pose",
                     localizer_node_name + "/pose_array",
                     drive_node_name,
                     drive_node_name + "/sensors/core",
                     drive_node_name + "/sensors/servo_position_command",
                     sensor_node_name,
                     sensor_node_name + "/scan"
                    );
    // Run MCL node until ROS is shutdown
    ros::waitForShutdown();
  }
  catch (std::runtime_error & except) {
    ROS_FATAL("MCL: %s", except.what());

    return 1;
  }
  return 0;
}