#include "ben_tf01_lidar/tf01_broadcast.h"

int main(int argc, char **argv)
{
  // init ROS
  ros::init(argc, argv, "tf01_broadcast_node");
  ros::NodeHandle *g_nh = new ros::NodeHandle();

  // create control node
  Tf01Broadcast g_node(g_nh);

  // set 100 Hz rate
  ros::Rate rate(100);

  // Main loop
  while (ros::ok())
  {
    g_node.workLoop();

    ros::spinOnce();
    rate.sleep();
  }
}
