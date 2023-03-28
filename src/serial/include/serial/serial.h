/**
 * @file serial.h
 * @brief 串口类
 * @author lihuagit (3190995951@qq.com)
 * @version 1.0
 * @date 2022-05-16
 * 
 */
#ifndef _SERIAL_H_
#define _SERIAL_H_

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class Serial {
private:
    int fd;
    int nSpeed;
    char nEvent;
    int nBits;
    int nStop;
	bool wait_uart;

    int set_opt(int fd, int nSpeed, char nEvent, int nBits, int nStop);

public:
    Serial(int nSpeed = 115200, char nEvent = 'N', int nBits = 8, int nStop = 1);
    ~Serial();

    bool InitPort(int nSpeed = 115200, char  nEvent = 'N', int nBits = 8, int nStop = 1);
//    int GetBytesInCOM() const ;
    bool WriteData(const unsigned char* pData, unsigned int length);
    bool ReadData(unsigned char* buffer, unsigned int length);
};

#endif /* _SERIAL_H_ */
