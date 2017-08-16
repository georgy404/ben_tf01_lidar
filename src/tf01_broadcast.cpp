/**
*  tf01_broadcast.cpp
*
*  This file is part of ben_tf01_lidar.
*  File contains class for reading data from lidar and provide ROS interface.
*  This ROS node designed for Benewake TF01 LIDAR (http://www.benewake.com/TF01_e.html)
*
*  Copyright (c) 2017 Konovalov Georgy <konovalov.g.404@gmail.com>
**/


#include "ben_tf01_lidar/tf01_broadcast.h"

Tf01Broadcast::Tf01Broadcast(ros::NodeHandle *nh)
{
  // init ROS
  nh_ = nh;
  range_pub_ = nh_->advertise<sensor_msgs::Range>("ir_height", 1000);

  // get params data and check it
  std::string dev_port;
  if (!nh_->getParam("/ben_tf01_lidar_node/port", dev_port))
    failureExit("Port is not set! Please set port as rosparam \"port\".");
  if (!nh_->getParam("/ben_tf01_lidar_node/frame", frame_id_))
    failureExit("Frame id is not set! Please set frame id as rosparam \"frame\".");

  // init serial port
  if(!serial_open(dev_port.c_str(), 115200))
    failureExit("Doesn't open port. Check rosparam \"port\" or device port.");
  else
    ROS_INFO("Port successfully opened.");
}

Tf01Broadcast::~Tf01Broadcast()
{
  // Close serial port before exit
  serial_close();
}

// ----------------------------------------------------------------------------
// --- Main class function

// Read data from serial port and send result to ROS topic
void Tf01Broadcast::workLoop()
{
  // init read buffer
  uint8_t read_buf[read_buf_len_];

  // read data from port and fill buffer
  int cur_len = 0;
  int count = 1;
  while (read_buf_len_ - count > cur_len)
  {
    cur_len = serial_receive(&read_buf[count-1], (read_buf_len_ - count));
    count += cur_len;
  }

  // find msgs from lidar in received buffer
  int16_t i = 0;
  while(i < count)
  {
    // check buffer end
    if(count - i < msg_bytes_count_)
      break;

    // if find start of msg
    if (isMsgBegin(read_buf, i))
    {
      std::cout << "begin\n";

      // copy bytes for one msg
      uint8_t msg[msg_bytes_count_];
      for(int j = 0; j < msg_bytes_count_; j++)
        msg[j] = read_buf[i+j];

      // debug output
      for(int j = 0; j < msg_bytes_count_; j++)
        printf("%d = %.2X | ", j, msg[j]);
      printf("\n");

      // check msg and get distance
      //if (calcChecksum(msg))
      //{
        std::cout << "calcChecksum() res = " << calcChecksum(msg) << std::endl;
        // get distance from msg and send to ROS
        float dist = getDistanceFromMsg(msg);
        sendDistance2Ros(dist);
      //}
    }

    // shift index
    i++;
  }
}


// ----------------------------------------------------------------------------
// --- External program interfaces functions

// Send distance value to ROS topic in sensor_msgs::Range format
void Tf01Broadcast::sendDistance2Ros(float dist)
{
  sensor_msgs::Range msg;
  msg.header.frame_id = frame_id_;

  msg.radiation_type = sensor_msgs::Range::INFRARED;
  msg.field_of_view = 0.0349066; // 2 degree in radians
  msg.max_range = 10;
  msg.min_range = 0.3;
  msg.range = dist;

  range_pub_.publish(msg);
}

// Send msg to std err stream and exit program with wrong code
void Tf01Broadcast::failureExit(const char *err_msg)
{
  ROS_ERROR(err_msg);
  serial_close();
  ros::shutdown();
  exit(1);
}

// ----------------------------------------------------------------------------
// --- Protocol parse functions

// Check bytes with start protocol msg bytes
bool Tf01Broadcast::isMsgBegin(uint8_t *buf, int16_t byte_num)
{
  // return true if two first bytes is header bytes
  return (buf[byte_num] == header_ && buf[byte_num+1] == header_);
}

// Calculate checksum and return result of comare with chechsum msg byte
bool Tf01Broadcast::calcChecksum(uint8_t *buf)
{
  uint8_t checksum = 0x0;
  for(int i = 0; i < (msg_bytes_count_-1); i++)
    checksum += buf[i];

  printf("check = %.2X\n", (checksum & 0xff));

  // return result of compare chechsum and checksum byte
  return (buf[msg_bytes_count_-1] == (checksum & 0xff));
}

// Parse msg and return distance in meters
float Tf01Broadcast::getDistanceFromMsg(uint8_t *msg)
{
  return float(msg[2] + (msg[3] * 256));
}


