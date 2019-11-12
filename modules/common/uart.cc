/******************************************************************************
 * MIT License

 * Copyright (c) 2019 Geekstyle

 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
******************************************************************************/
#include "modules/common/uart.h"

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>
#include "cyber/common/log.h"

Uart::Uart(const char* dev) {
  char tty[32] = "/dev/";

  strcat(tty, dev);
  // snprintf(tty, sizeof(tty), "%s%s", tty, dev);

  fd = open(tty, O_RDWR | O_NOCTTY);
  if (fd > 0) {
    AINFO << "opened :" << tty;
  } else {
    AERROR << "opened error:" << tty;
  }
}

Uart::~Uart() {
  if (fd > 0) close(fd);
}

int Uart::SetOpt(int speed, int bits, char event, int stop) {
  if (fd < 0) {
    return -1;
    AERROR << "Uart Init failed";
  }

  fcntl(fd, F_SETFL, 0);

  struct termios newtio, oldtio;
  if (tcgetattr(fd, &oldtio) != 0) {
    AERROR << "tcgetattr error";
    return -1;
  }

  tcflush(fd, TCIOFLUSH);

  bzero(&newtio, sizeof(newtio));
  newtio.c_cflag |= CLOCAL | CREAD;
  newtio.c_cflag &= ~CSIZE;

  newtio.c_lflag &= ~(ICANON | ISIG | ECHO | ECHOE | IEXTEN);
  newtio.c_oflag &= ~OPOST;
  newtio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | BRKINT | INPCK | ISTRIP);
  newtio.c_cflag &= ~CRTSCTS;

  switch (bits) {
    case 7:
      newtio.c_cflag |= CS7;
      break;
    case 8:
      newtio.c_cflag |= CS8;
      break;
  }

  switch (event) {
    case 'O':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag |= PARODD;
      newtio.c_iflag |= (INPCK | ISTRIP);
      break;
    case 'E':
      newtio.c_cflag |= PARENB;
      newtio.c_cflag &= ~PARODD;
      newtio.c_iflag |= (INPCK | ISTRIP);
      break;
    case 'N':
      newtio.c_cflag &= ~PARENB;
      break;
  }

  switch (speed) {
    case 4800:
      cfsetispeed(&newtio, B4800);
      cfsetospeed(&newtio, B4800);
      break;
    default:
    case 9600:
      cfsetispeed(&newtio, B9600);
      cfsetospeed(&newtio, B9600);
      break;
    case 19200:
      cfsetispeed(&newtio, B19200);
      cfsetospeed(&newtio, B19200);
      break;
    case 38400:
      cfsetispeed(&newtio, B38400);
      cfsetospeed(&newtio, B38400);
      break;
    case 57600:
      cfsetispeed(&newtio, B57600);
      cfsetospeed(&newtio, B57600);
      break;
    case 115200:
      cfsetispeed(&newtio, B115200);
      cfsetospeed(&newtio, B115200);
      break;
    case 921600:
      cfsetispeed(&newtio, B921600);
      cfsetospeed(&newtio, B921600);
      break;
  }

  if (stop == 1) {
    newtio.c_cflag &= ~CSTOPB;
  } else if (stop == 2) {
    newtio.c_cflag |= CSTOPB;
  }

  newtio.c_cc[VTIME] = 5;
  newtio.c_cc[VMIN] = 0;

  if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
    AERROR << "Arduino :set opt error";
    return -1;
  }

  tcflush(fd, TCIFLUSH);

  return 0;
}

int Uart::Write(char* buf, int size) {
  if (fd < 0) {
    return -1;
    AERROR << "Uart Init failed";
  }

  int ret = static_cast<int>(write(fd, buf, size));
  if (ret < 0) {
    AERROR << "uart write failed";
  }
  return ret;
}

int Uart::Read(char* buf, int size) {
  if (fd < 0) {
    return -1;
    AERROR << "Uart Init failed";
  }

  fd_set set;
  struct timeval timeout;

  FD_ZERO(&set);
  FD_SET(fd, &set);

  timeout.tv_sec = 1;
  timeout.tv_usec = 2000;

  int ret = select(fd + 1, &set, NULL, NULL, &timeout);

  if (ret > 0) {
    ret = static_cast<int>(read(fd, buf, size));
  }

  return ret;
}

int Uart::GetFd() { return fd; }
