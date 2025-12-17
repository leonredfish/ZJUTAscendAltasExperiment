#ifndef __TASK_H_
#define __TASK_H_

#include "key.h"
#include "led.h"
#include "adxl345.h"
#include "ssd1306.h"
#include "sht20.h"
#include "pca9557.h"
#include "ds1339u.h"
#include "v4l2.h"
#include "lcd_gui.h"
#include "serial_listener.h"
typedef uint8_t(*TPin_level)();



#ifdef ASCEND310

#define LED1    2



#else

#define LED1        2
#define LED2        74
#define LED3        73
#define LED4        38
#define KEY1        147
#define KEY2        235
#define KEY3        3

#define I2C_DEV         "/dev/i2c-7"
#define SPI_DEV_0         "/dev/spidev0.0"
#define SPI_DEV_1         "/dev/spidev7.0"

#endif






void Led_Test(void);
void Led_Pwm_Test(void);
void Key_Test(void);
void Esp32_Ble_Test(void);
void Esp32_UdpClient_Test(void);
void Esp32_HttpClient_Test(void);
void Adxl345_Test(void);
void Oled_Test(void);
void Sht20_Test(void);
void Pca9557_Test(void);
void Ds1399u_Test(void);
void Lcd_test(void);
void Measure(void);
#endif