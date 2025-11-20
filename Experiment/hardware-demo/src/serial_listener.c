#include "serial_listener.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <termios.h>
#include <time.h>

/**
 * 串口初始化函数（内部辅助函数，不对外暴露）
 * @param serial_dev 串口设备路径
 * @param baudrate 波特率
 * @return 成功返回串口文件描述符，失败返回 -1
 */
static int serial_init(const char* serial_dev, speed_t baudrate) {
    if (!serial_dev || strlen(serial_dev) == 0) {
        fprintf(stderr, "Error: serial device path is empty\n");
        return -1;
    }

    // 打开串口（读写+非控制终端+非阻塞）
    int fd = open(serial_dev, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd < 0) {
        perror("serial_init: open serial port failed");
        return -1;
    }

    // 获取并配置串口属性
    struct termios old_tio, new_tio;
    if (tcgetattr(fd, &old_tio) != 0) {
        perror("serial_init: tcgetattr failed");
        close(fd);
        return -1;
    }

    bzero(&new_tio, sizeof(new_tio));
    // 核心配置：波特率 + 8数据位 + 无校验 + 1停止位 + 禁用流控
    new_tio.c_cflag = baudrate | CS8 | CLOCAL | CREAD;
    new_tio.c_cflag &= ~PARENB;  // 无校验位
    new_tio.c_cflag &= ~CSTOPB;  // 1位停止位
    new_tio.c_cflag &= ~CRTSCTS; // 禁用硬件流控

    // 原始模式：禁用终端回显、规范模式等（避免串口数据被干扰）
    new_tio.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    new_tio.c_iflag &= ~(IXON | IXOFF | IXANY); // 禁用软件流控
    new_tio.c_oflag &= ~OPOST;                  // 禁用输出处理

    // 读取超时配置（0：无数据立即返回；5：0.5秒超时）
    new_tio.c_cc[VMIN] = 0;
    new_tio.c_cc[VTIME] = 5;

    // 刷新缓冲区并应用配置
    tcflush(fd, TCIFLUSH);
    if (tcsetattr(fd, TCSANOW, &new_tio) != 0) {
        perror("serial_init: tcsetattr failed");
        close(fd);
        return -1;
    }

    printf("serial_init: %s initialized successfully (baudrate: %d 8N1)\n",
           serial_dev, baudrate);
    return fd;
}

/**
 * 串口发送+监听循环（每1秒发送指定字节数组，持续监听响应）
 */
int serial_listen_loop(const char* serial_dev, speed_t baudrate) {
    // 初始化串口
    int serial_fd = serial_init(serial_dev, baudrate);
    if (serial_fd < 0) {
        fprintf(stderr, "serial_listen_loop: serial init failed\n");
        return 0; // 失败返回0
    }

    // 配置发送数据
    static const unsigned char send_buf[] = {0x16, 0x54, 0x0d, 0x21};
    const size_t send_len = sizeof(send_buf); // 发送长度：4字节

    // 监听配置
    const int BUFFER_SIZE = 128;
    char recv_buffer[BUFFER_SIZE];
    time_t last_send_time = time(NULL); // 记录上次发送时间


    // 死循环：发送+监听
    while (1) {
        // 1. 检查是否到达发送间隔（每2秒发送一次）
        time_t current_time = time(NULL);
        if (current_time - last_send_time >= 2) { // 间隔≥2秒则发送
            // 发送指定字节数组
            ssize_t send_ret = write(serial_fd, send_buf, send_len);
            if (send_ret == send_len) {
                printf("[Serial Sent] 0x16 0x54 0x0D 0x21\n");
            } else if (send_ret < 0) {
                perror("serial_listen_loop: send failed");
            } else {
                fprintf(stderr, "serial_listen_loop: send incomplete (sent %zd/%zu bytes)\n",
                        send_ret, send_len);
            }
            last_send_time = current_time; // 更新上次发送时间
        }

        // 2. 监听串口数据（非阻塞，超时1秒）
        ssize_t recv_len = read(serial_fd, recv_buffer, sizeof(recv_buffer) - 1);
        if (recv_len > 0) {  // 成功读取到响应数据

            printf("[Serial Received] ");
            for (int i=0; i<recv_len; i++) {
                printf("0x%02X ", (unsigned char)recv_buffer[i]);
            }
            if (recv_len == 3) { 
                unsigned char low4bit = recv_buffer[0] & 0x0F;  // 0x0F 是二进制 00001111，保留低4位
                int number = low4bit; 
                printf("=> Extracted Number: %d", number);
                char cmd[256]; 
                const char* python_script = "/home/HwHiAiUser/Experiment/hardware-demo/code/main.py";  
                snprintf(cmd, sizeof(cmd), "/home/HwHiAiUser/.conda/envs/embedded/bin/python3.8 %s %d", python_script, number);

                int ret = system(cmd);  // ret 是脚本执行返回值（0 表示成功）
                if (ret == -1) {
                    printf("[Error] Failed to call Python script\n");
                } else {
                    printf("[Success] Python script executed, return code: %d\n", ret);
                }

            }
        

            printf("\n");
        } else if (recv_len == 0) {
            continue;
        } else {
            if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                perror("serial_listen_loop: read failed");
            }
        }
        usleep(10000);
    }

    close(serial_fd); // 理论上不会执行到这里，仅作冗余保护
    return 1;
}