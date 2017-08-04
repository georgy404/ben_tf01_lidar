/**
*  main.cpp
*
*  This file is part of ben_tf01_lidar.
*  File contains main function of program.
*  This ROS node designed for Benewake TF01 LIDAR (http://www.benewake.com/TF01_e.html)
*
*  Copyright (c) 2017 Konovalov Georgy <konovalov.g.404@gmail.com>
**/


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
