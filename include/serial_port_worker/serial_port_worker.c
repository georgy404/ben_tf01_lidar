/**
*  serial_port_worker.c
*
*  This file is part of ben_tf01_lidar.
*  File contains functions for serial port.
*  This ROS node designed for Benewake TF01 LIDAR (http://www.benewake.com/TF01_e.html)
*
*  Copyright (c) 2017 Konovalov Georgy <konovalov.g.404@gmail.com>
**/


#include "serial_port_worker.h"


// Open serial port in raw mode, with custom baudrate if necessary
bool serial_open(const char *device, int rate)
{
  struct termios2 ntio;

  if ((file_descriptor = open(device,O_RDONLY|O_NOCTTY)) == -1)
    file_descriptor = -1;

  ioctl(file_descriptor, TCGETS2, &ntio);
  ntio.c_cflag &= ~CBAUD;
  ntio.c_cflag |= BOTHER;
  ntio.c_ispeed = rate;
  ntio.c_ospeed = rate;
  ntio.c_cc[VMIN] = 0; // Set non-blocking mode
  ntio.c_cc[VTIME] = 10.0 / (double)50; // read timeout
  int retval = ioctl(file_descriptor, TCSETS2, &ntio);

  // check if port doesn't open
  if (retval != 0)
  {
    perror("ioctl");
    return false;
  }

  printf("Success open serial port with rate %d\n", rate);
  return true;
}

// Send data to serial port
void serial_send(const uint8_t *buf, uint16_t len)
{
  size_t ret;

  while (len != 0 && (ret = write(file_descriptor, buf, len)) != 0)
  {
    if (ret == -1)
    {
      if (errno == EINTR)
        continue;
      perror("Error while send msg to serial port\n");
      break;
    }

    len -= ret;
    buf += ret;
  }
}

// Receive data from serial port; non blocking reading
// Return read byte count
int serial_receive(uint8_t *buf, uint16_t len)
{
  size_t nr;

start:
  nr = read(file_descriptor, buf, len);
  if (nr == -1)
  {
    if (errno == EINTR)
      goto start; // something went wrong
    if (errno == EAGAIN)
      return nr; // don't recieve needing len
    else
      perror("Error while receive msg from serial port\n");
  }

  return nr;
}

// Close port
void serial_close()
{
  if(file_descriptor != -1)
    close(file_descriptor);
}
