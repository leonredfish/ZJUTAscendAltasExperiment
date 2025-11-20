#ifndef SERIAL_LISTENER_H
#define SERIAL_LISTENER_H

#include <termios.h>  // 提供 speed_t 类型（波特率常量）

/**
 * 串口持续监听函数（核心对外接口）
 * @param serial_dev 串口设备路径（如 "/dev/ttyS0"、"/dev/ttyUSB0"）
 * @param baudrate 波特率（Linux 系统常量，如 B115200、B9600、B19200）
 * @return 成功返回1（正常启动循环），失败返回0（初始化失败）
 */
int serial_listen_loop(const char* serial_dev, speed_t baudrate);

#endif  // SERIAL_LISTENER_H