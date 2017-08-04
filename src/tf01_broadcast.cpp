#include "ben_tf01_lidar/tf01_broadcast.h"
#include <iostream>

Tf01Broadcast::Tf01Broadcast(ros::NodeHandle *nh)
{
  // init ROS
  nh_ = nh;
  range_pub_ = nh_->advertise<sensor_msgs::Range>("ir_height", 1000);

  // get params data and check it
  std::string dev_port;
  if (!nh_->getParam("/port", dev_port))
    failureExit("Port is not set! Please set port as rosparam \"port\".");
  if (!nh_->getParam("/frame", frame_id_))
    failureExit("Frame id is not set! Please set frame id as rosparam \"frame\".");

  // init serial port
  if(!serial_open(dev_port.c_str(), 9600))
    failureExit("Doesn't open port. Check rosparam \"port\" or device port.");
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
  // init read buf
  uint8_t read_buf[read_buf_len_];

  // read data from port
  int ret = serial_receive(read_buf, read_buf_len_);
  std::cout << "ret = " << ret << std::endl;

  // find msgs from lidar in received buffer
  int16_t i = 0;
  while(i < read_buf_len_)
  {
    // check buffer end
    if(read_buf_len_ - i < msg_bytes_count_)
      break;

    // if find start of msg
    if (isMsgBegin(read_buf, i))
    {
      // copy bytes for one msg
      uint8_t msg[msg_bytes_count_];
      memcpy(msg, (read_buf), msg_bytes_count_);

      // check msg and get distance
      if (calcChecksum(msg))
      {
        // get distance from msg and send to ROS
        float dist = getDistanceFromMsg(msg);
        sendDistance2Ros(dist);
      }
    }

    // shift index on message size
    i += msg_bytes_count_;
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
  return (buf[byte_num] == header_ && buf[byte_num++] == header_);
}

// Calculate checksum and return result of comare with chechsum msg byte
bool Tf01Broadcast::calcChecksum(uint8_t *buf)
{
  uint8_t checksum = 0x0;
  int index = 0;

  // add to chechsum two first header bytes
  checksum += (header_*2);
  index += 2;
  // get sum from 3 to 8 bytes
  while (index < 8)
    checksum += buf[index];

  // return result of compare chechsum and checksum byte
  return (buf[index] == (checksum & 0xff));
}

// Parse msg and return distance in meters
float Tf01Broadcast::getDistanceFromMsg(uint8_t *msg)
{
  return (msg[2] + (msg[3] * 256)) / 100.0;
}

