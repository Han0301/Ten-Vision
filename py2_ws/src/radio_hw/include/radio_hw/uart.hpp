#ifndef UART_HPP_
#define UART_HPP_
//sys include
#include <asm/termbits.h>//以修改源码

#include <termios.h> /*PPSIX 终端控制定义*/
#include <linux/serial.h>
class Uart{
public:
    Uart(){}
    ~Uart(){}
    int SerialInit(int &fd, const char* devname, int baude);
    int baude_changed(int &fd,int speed);
private:
    struct termios uart;
     int serial_set_speci_baud(int &fd,int baude);
     int setCustomBaudRate(int&fd,int baude);
};
#endif
