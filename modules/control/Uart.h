#ifndef _LL_UART_UTILS_H_
#define _LL_UART_UTILS_H_

/*
usage:
    Uart uart("ttyS1");
    uart.SetOpt(115200, 8, 'N' 1);
*/

class Uart
{
public:
    Uart(const char * dev);
    ~Uart();

    int SetOpt(int speed, int bits, char event, int stop);
    int Write(char * buf, int size);
    int Read(char * buf, int size);
    int GetFd();

private:
    int fd;
};

#endif

