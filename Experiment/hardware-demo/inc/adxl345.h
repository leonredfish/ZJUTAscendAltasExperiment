#ifndef __ADXL345_H_
#define __ADXL345_H_

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <stdbool.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <string.h>


#define ADXL345_REG_DEVID                 0x00        /**< device id register */
#define ADXL345_REG_THRESH_TAP            0x1D        /**< thresh tap register */
#define ADXL345_REG_OFSX                  0x1E        /**< x offset register */
#define ADXL345_REG_OFSY                  0x1F        /**< y offset register */
#define ADXL345_REG_OFSZ                  0x20        /**< z offset register */
#define ADXL345_REG_DUR                   0x21        /**< duration time register */
#define ADXL345_REG_LATENT                0x22        /**< latent register */
#define ADXL345_REG_WINDOW                0x23        /**< window register */
#define ADXL345_REG_THRESH_ACT            0x24        /**< act thresh register */
#define ADXL345_REG_THRESH_INACT          0x25        /**< inact thresh register */
#define ADXL345_REG_TIME_INACT            0x26        /**< inact time register */
#define ADXL345_REG_ACT_INACT_CTL         0x27        /**< act inact ctl register */
#define ADXL345_REG_THRESH_FF             0x28        /**< free fall thresh register */
#define ADXL345_REG_TIME_FF               0x29        /**< free fall time register */
#define ADXL345_REG_TAP_AXES              0x2A        /**< tap axes register */
#define ADXL345_REG_ACT_TAP_STATUS        0x2B        /**< act tap status register */
#define ADXL345_REG_BW_RATE               0x2C        /**< bandwidth rate register */
#define ADXL345_REG_POWER_CTL             0x2D        /**< power control register */
#define ADXL345_REG_INT_ENABLE            0x2E        /**< interrupt enable register */
#define ADXL345_REG_INT_MAP               0x2F        /**< interrupt map register */
#define ADXL345_REG_INT_SOURCE            0x30        /**< interrrupt source register */
#define ADXL345_REG_DATA_FORMAT           0x31        /**< data format register */
#define ADXL345_REG_DATAX0                0x32        /**< data X0 register */
#define ADXL345_REG_DATAX1                0x33        /**< data X1 register */
#define ADXL345_REG_DATAY0                0x34        /**< data Y0 register */
#define ADXL345_REG_DATAY1                0x35        /**< data Y1 register */
#define ADXL345_REG_DATAZ0                0x36        /**< data Z0 register */
#define ADXL345_REG_DATAZ1                0x37        /**< data Z1 register */
#define ADXL345_REG_FIFO_CTL              0x38        /**< fifo control register */
#define ADXL345_REG_FIFO_STATUS           0x39        /**< fifo status register */




void Adxl345Init(); 
void Adxl345GetData(float* data);
void Adxl345Release();
bool Adxl345WriteCmd(unsigned char cmd,unsigned char data);
bool Adxl345ReadCmd(unsigned char cmd,unsigned char* data);

#endif