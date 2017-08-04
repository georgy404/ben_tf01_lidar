/**
*  serial_port_worker.h
*
*  This file is part of ben_tf01_lidar.
*  File contains functions for serial port.
*  This ROS node designed for Benewake TF01 LIDAR (http://www.benewake.com/TF01_e.html)
*
*  Copyright (c) 2017 Konovalov Georgy <konovalov.g.404@gmail.com>
**/


#ifndef __SERIAL_PORT_WORKER_H__
#define __SERIAL_PORT_WORKER_H__

#include <stdlib.h>	// for Standard Data Types
#include <stdint.h>	// for Standard Data Types
#include <stdio.h>
#include <errno.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h>
#include <asm/termios.h> // for setup port rate
#include <fcntl.h>

static int file_descriptor = -1;

bool serial_open(const char *device, int rate);
void serial_send(const uint8_t *buf, uint16_t len);
int serial_receive(uint8_t *buf, uint16_t len);
void serial_close();

#endif // __SERIAL_PORT_WORKER_H__
