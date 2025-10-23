#include "unistd.h"
#include "linux/spi/spidev.h"
#include "sys/ioctl.h"
#include "fcntl.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include "memory.h"


unsigned char tx_buf[64];
unsigned char rx_buf[64];


int spifd;
unsigned char spi_mode = SPI_MODE_3;
unsigned char spi_bits = 8;
unsigned int spi_speed = 500000;


char SpiTransfers(unsigned char *tx, unsigned char *rx, unsigned char len)
{
	int ret;
	if(len > 64)
		return -1;
	struct spi_ioc_transfer tr={0};
	tr.tx_buf = (unsigned long)tx;
	tr.rx_buf = (unsigned long)rx;

	tr.len = len;
	tr.delay_usecs = 1;
    	tr.speed_hz = spi_speed;
    	tr.bits_per_word = spi_bits;


	ret = ioctl(spifd, SPI_IOC_MESSAGE(1), &tr);
	if(ret < 0)
	{
		printf("can't send spi message %d\n", ret);
        	return -1;
	}
	else
	{
		printf("rx = 0x%02x\r\n", rx[0]);
		return ret;
	}
}

char Adxl345WriteRead(unsigned char reg, unsigned char val)
{
	int ret;
	memset(tx_buf, 0xFF, 64);
	memset(rx_buf,0xFF,64);
	tx_buf[0]=reg;
	tx_buf[1]=val;
	ret = SpiTransfers(tx_buf,rx_buf,2);
}



int main(void)
{
	int ret;


	spifd = open("/dev/spidev0.0", O_RDWR);
	if(spifd < 0)
	{
		printf("open spi dev error \r\n");
		return -1;
	}
	printf("open spi dev success \r\n");

	ret = ioctl(spifd, SPI_IOC_WR_MODE32, &spi_mode);
	if(ret == -1)
	{
		printf("open spi ioctl  error \r\n");
                goto end;
	}
	
	ret = ioctl(spifd,SPI_IOC_RD_MODE, &spi_mode);
	
	printf("set spi mode= 0x%d  \r\n", spi_mode);

	ret = ioctl(spifd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
        if(ret == -1)
        {
                printf("open spi ioctl  error \r\n");
                goto end;
        }
	ret = ioctl(spifd,SPI_IOC_RD_BITS_PER_WORD, &spi_bits);

        printf("set spi bit= %d  \r\n", spi_bits);


	ret = ioctl(spifd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);
        if(ret == -1)
        {
                printf("open si ioctl  error \r\n");
                goto end;
        }
        ret = ioctl(spifd,SPI_IOC_RD_MAX_SPEED_HZ, &spi_speed);

        printf("set spi speed= %d  \r\n", spi_speed);	


	


	Adxl345WriteRead(0x80, 0xff);
	Adxl345WriteRead(0x27, 0x08);
	Adxl345WriteRead(0x31, 0x0B);
	Adxl345WriteRead(0x2C, 0x0F);
	Adxl345WriteRead(0x2E, 0x00);
	Adxl345WriteRead(0x2F, 0x00);
	Adxl345WriteRead(0x38, 0x00);


	Adxl345WriteRead(0x31|0x80, 0xFF);
	Adxl345WriteRead(0x27|0x80, 0xFF);
        Adxl345WriteRead(0x31|0x80, 0xFF);
        Adxl345WriteRead(0x2C|0x80, 0xFF);
        Adxl345WriteRead(0x2E|0x80, 0xFF);
        Adxl345WriteRead(0x2F|0x80, 0xFF);
        Adxl345WriteRead(0x38|0x80, 0xFF);









end:
	close(spifd);
	return 0;
}



