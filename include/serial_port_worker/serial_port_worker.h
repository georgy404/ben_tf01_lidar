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
