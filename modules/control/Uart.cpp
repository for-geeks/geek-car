#include "Uart.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>

Uart::Uart(const char * dev)
{
    char tty[32] = "/dev/";

    strcat(tty, dev);

    fd = open(tty, O_RDWR | O_NOCTTY); 
    //fd = open(tty, O_RDWR | O_NOCTTY | O_NDELAY); 
    if (fd > 0) {
        printf("open %s!\n", tty);
    }
}

Uart::~Uart()
{
    if (fd > 0)
        close(fd);
}

int Uart::SetOpt(int speed, int bits, char event, int stop)
{
    if (fd < 0) {
        return -1;
        perror("Uart Init failed");
    }

    fcntl(fd, F_SETFL, 0);

    struct termios newtio, oldtio;
    if ( tcgetattr( fd, &oldtio ) != 0) {
        perror("tcgetattr error");
        return -1;
    }

    tcflush(fd, TCIOFLUSH);

    bzero( &newtio, sizeof(newtio) );
    newtio.c_cflag |= CLOCAL | CREAD;
    newtio.c_cflag &= ~CSIZE;

    newtio.c_lflag &= ~(ICANON | ISIG | ECHO | ECHOE | IEXTEN);
    newtio.c_oflag &= ~OPOST;
    newtio.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | BRKINT | INPCK | ISTRIP);
    newtio.c_cflag &= ~CRTSCTS;

    switch ( bits ) {
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
    }

    if (stop == 1) {
        newtio.c_cflag &= ~CSTOPB;
    }
    else if (stop == 2) {
        newtio.c_cflag |= CSTOPB;
    }

    newtio.c_cc[VTIME] = 5;
    newtio.c_cc[VMIN] = 0;

    

    if ((tcsetattr(fd, TCSANOW, &newtio)) != 0) {
        perror("set opt error");
        return -1;
    }

    tcflush(fd, TCIFLUSH);

    return 0;
}

int Uart::Write(char * buf, int size)
{
    if (fd < 0) {
        return -1;
        perror("Uart Init failed");
    }

    int ret = write(fd, buf, size);
    if (ret < 0) {
        perror("uart write failed");
    }
    return ret;
}

int Uart::Read(char * buf, int size) 
{
    if (fd < 0) {
        return -1;
        perror("Uart Init failed");
    }
    
    fd_set set;
    struct timeval timeout;

    FD_ZERO(&set);
    FD_SET(fd, &set);

    timeout.tv_sec = 1;
    timeout.tv_usec = 200000;

    int ret = select(fd + 1, &set, NULL, NULL, &timeout);

    if (ret > 0) {
        ret = read(fd, buf, size);
    }
  
    return ret;
}

int Uart::GetFd()
{
    return fd;
}

