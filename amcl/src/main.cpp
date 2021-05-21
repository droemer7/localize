#include "amcl/node.h"

using namespace localize;

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "localizer");

  // Get node names
  ros::NodeHandle pnh("~");
  std::string amcl_node_name = pnh.param("node_names/amcl", std::string("localizer"));
  std::string drive_node_name = pnh.param("node_names/drive", std::string("vesc"));
  std::string sensor_node_name = pnh.param("node_names/sensor", std::string("laser"));

  try {
    // Start localizer node
    AMCLNode amcl_node(amcl_node_name,
                       amcl_node_name + "/pose",
                       amcl_node_name + "/pose_array",
                       drive_node_name,
                       drive_node_name + "/sensors/core",
                       drive_node_name + "/sensors/servo_position_command",
                       sensor_node_name,
                       sensor_node_name + "/scan"
                      );
    // Run localizer node until ROS is shutdown
    ros::waitForShutdown();
  }
  catch (std::exception & except) {
    ROS_FATAL("AMCL: %s", except.what());

    return 1;
  }
  return 0;
}