/**
*  tf01_broadcast.h
*
*  This file is part of ben_tf01_lidar.
*  File contains class for reading data from lidar and provide ROS interface.
*  This ROS node designed for Benewake TF01 LIDAR (http://www.benewake.com/TF01_e.html)
*
*  Copyright (c) 2017 Konovalov Georgy <konovalov.g.404@gmail.com>
**/


#ifndef __TF01_BROADCAST_H__
#define __TF01_BROADCAST_H__

// ROS
#include "ros/ros.h"
#include "sensor_msgs/Range.h"

// Serial port lib
extern "C" {
  #include "serial_port_worker/serial_port_worker.h"
}

class Tf01Broadcast
{
public:
  Tf01Broadcast(ros::NodeHandle *nh);
  ~Tf01Broadcast();

  // --- Main class function
  // Read data from serial port and send result to ROS topic.
  void workLoop();

private:
  // ROS nodehanle object
  ros::NodeHandle *nh_;

  // ROS publisher objects
  ros::Publisher range_pub_;

  // Data variables
  std::string frame_id_;

  // Lidar protocol variables
  const int16_t msg_bytes_count_ = 9; // one msg from lidar has lenght 9 byte
  const int16_t read_buf_len_ = 20; // lenght of read buffer
  const int8_t header_ = 0x59; // header byte

private:
  // --- External program interfaces functions
  // Send msg to std err stream and exit program with wrong code
  void failureExit(const char *err_msg);
  // Send distance value to ROS topic in sensor_msgs::Range format
  void sendDistance2Ros(float dist);

  // --- Protocol parse functions
  // Parse msg and return distance in meters
  float getDistanceFromMsg(uint8_t *msg);
  // Calculate checksum and return result of comare with chechsum msg byte
  bool calcChecksum(uint8_t *buf);
  // Check bytes with start protocol msg bytes
  bool isMsgBegin(uint8_t *buf, int16_t byte_num);
};

#endif // __TF01_BROADCAST_H__
