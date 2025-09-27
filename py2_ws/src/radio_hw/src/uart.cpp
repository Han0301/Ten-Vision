#include <iostream>
#include <errno.h> /*错误号定义*/
#include <fcntl.h>   /*文件控制定义*/
#include <sys/ioctl.h>
//#include <asm/termbits.h>//以修改源码

#include <sys/types.h>
#include <sys/stat.h>
#include <string.h>
#include "radio_hw/uart.hpp"

int Uart::SerialInit(int &fd, const char* devname, int baude){
    fd = open(devname, O_RDWR | O_NOCTTY | O_NDELAY);//以只读形式、不将此终端作为此进程的终端控制器、非阻塞的形式打开串口
    if (-1 == fd)
    {
        printf("Open %s Error!\n",devname);
        perror("\n");
        return -1;
    }
    if ((fcntl(fd, F_SETFL, 0)) < 0)//设置串口非阻塞，因为这里是以非阻塞形式打开的，所以第三个参数为0，
    {
        perror("Fcntl F_SETFL Error!\n");
        return -1;
    }
    if (tcgetattr(fd, &uart) != 0)
    {
        perror("tcgetattr error!\n");
        return -1;
    }
    cfmakeraw(&uart); //将终端设置为原始模式，该模式下全部的输入数据以字节为单位被处理
    //set speed
/***********************普通设置波特率****************************/
    cfsetispeed(&uart, B115200);
    cfsetospeed(&uart, B115200);
    uart.c_cflag |= CLOCAL | CREAD;//本地连接和接受使能
    uart.c_iflag &= ~ICRNL;    //禁止将输入中的回车翻译为新行 (除非设置了 IGNCR)
    uart.c_iflag &= ~ISTRIP;   //禁止将所有接收的字符裁减为7比特
    uart.c_cflag &= ~PARENB;   //禁止奇偶校验码的生成和检测功能
    uart.c_cflag &= ~CSTOPB;   //设置1停止位
    uart.c_cflag &= ~CSIZE;    //清除数据位设置
    uart.c_cflag |=  CS8;      //设置为8数据位
    uart.c_cflag &= ~CRTSCTS;  //禁止硬件流控
    uart.c_cc[VTIME] = 0;      //指定所要读取字符的最小数量
    uart.c_cc[VMIN] = 1;       //指定读取第一个字符的等待时间，时间的单位为n*100ms
                                //假设设置VTIME=0，则无字符输入时read（）操作无限期的堵塞
    tcflush(fd, TCIFLUSH);     //清空终端未完毕的输入/输出请求及数据。

    if (tcsetattr(fd, TCSANOW, &uart) != 0) //激活配置
    {
        perror("tcsetattr Error!\n");
        return -1;
    }
    baude_changed(fd,baude);
    return 1;
}

int Uart::setCustomBaudRate(int &fd,int baude){
    // struct serial_struct currentSerialInfo;
    // if (ioctl(fd, TIOCGSERIAL, &currentSerialInfo) == -1)
    //     perror("\n");
    // printf("baud_base :%d\n",currentSerialInfo.baud_base);
    // if (currentSerialInfo.baud_base % baude != 0)
    //     // perror("\n");

    // currentSerialInfo.flags &= ~ASYNC_SPD_MASK;
    // currentSerialInfo.flags |= (ASYNC_SPD_CUST /* | ASYNC_LOW_LATENCY*/);
    // currentSerialInfo.custom_divisor = currentSerialInfo.baud_base / baude;
    // if (currentSerialInfo.custom_divisor == 0)
    //     // perror("\n");
    // if (ioctl(fd, TIOCSSERIAL, &currentSerialInfo) == -1)
    //     perror("\n");
    // cfsetispeed(&uart, B38400);
    // cfsetispeed(&uart, B38400);
}

int Uart::serial_set_speci_baud(int &fd,int baude){
    // struct serial_struct ss, ss_set;
    // tcgetattr(fd, &uart);
    // cfsetispeed(&uart, B38400);
    // cfsetospeed(&uart, B38400);
    // tcflush(fd, TCIFLUSH);/*handle unrecevie char*/
    // tcsetattr(fd, TCSANOW, &uart);
    // if((ioctl(fd, TIOCGSERIAL, &ss)) < 0)
    // {
    //     printf("BAUD: error to get the serial_struct info:%s\n", strerror(errno));
    //     return -1;
    // }
    // ss.flags |= ASYNC_SPD_CUST| ASYNC_LOW_LATENCY;
    // ss.flags &= ~ASYNC_SPD_MASK;
    // ss.custom_divisor = ss.baud_base / baude;
    // if((ioctl(fd, TIOCSSERIAL, &ss)) < 0)
    // {
    //     printf("BAUD: error to set serial_struct:%s\n", strerror(errno));
    //     return -2;
    // }
    // ioctl(fd, TIOCGSERIAL, &ss_set);
    // printf("BAUD: success set baud to %d,custom_divisorSerialInit(int &fd, const char* devname, int baude)完成串口打开与波特率自定义。
}

/*先使用标准波特率打开串口，再使用该接口修该波特率*/
int Uart::baude_changed(int &fd,int speed)
{
	struct termios2 tio = {0};
	tio.c_cflag = BOTHER | CS8 | CLOCAL | CREAD;
	tio.c_iflag = IGNPAR;
	tio.c_oflag = 0;
	tio.c_ispeed = speed;
	tio.c_ospeed = speed;
	return ioctl(fd, TCSETS2, &tio);
}
