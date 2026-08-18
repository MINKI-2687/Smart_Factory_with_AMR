#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H
#ifndef _POSIX_C_SOURCE
#define _POSIX_C_SOURCE 200809L
#endif

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <sys/file.h>
#include <sys/ioctl.h>
int serial_port_open(const char *device, int baud);
void serial_port_close(int fd);

#endif /* SERIAL_PORT_H */
