#include "adxl345.h"

#define Pi 3.14
#define GPIO_PATH(num) "/sys/class/gpio/gpio" #num "/value"

static const char* cs_dev = "/sys/class/gpio/gpio90/value";
static const char* spi_dev =  "/dev/spidev0.0";

static unsigned char spi_mode = SPI_MODE_3;
static unsigned char spi_bits = 8;
static unsigned int spi_speed = 500000;
static int spifd;
static int csfd;
static float angle_x;
static float angle_y;
static float angle_z;

static void Spi_CS_High()
{
    write(csfd,"1", 2);
}
static void Spi_CS_Low()
{
    write(csfd,"0", 2);
}

static bool ReadDeviceID()
{
    unsigned char device_id;
    bool read_id = false;
    for(int i=0;i<10;i++)
    {
        Adxl345ReadCmd(0x00,&device_id);
        if( device_id !=0xE5)
            continue;
        else
        {   read_id = true;
            break;
        }
    }
    if(!read_id)
    {
        return false;
    }
    return true;
}

static void ConfigAdxl345()
{
    Adxl345WriteCmd(ADXL345_REG_POWER_CTL,0x08);   /*< Configure */
    // Adxl345ReadCmd(ADXL345_REG_POWER_CTL,NULL);
    Adxl345WriteCmd(ADXL345_REG_DATA_FORMAT,0x0B);    /*< Configure full_res=1 16g*/
    // Adxl345ReadCmd(ADXL345_REG_DATA_FORMAT,NULL);
    Adxl345WriteCmd(ADXL345_REG_BW_RATE,0x0A);    /* rate 3200Hz*/
    // Adxl345ReadCmd(ADXL345_REG_BW_RATE,NULL);
    Adxl345WriteCmd(ADXL345_REG_INT_ENABLE,0x00);
    // Adxl345ReadCmd(ADXL345_REG_INT_ENABLE,NULL);
    Adxl345WriteCmd(ADXL345_REG_INT_MAP,0x80);
    // Adxl345ReadCmd(ADXL345_REG_INT_MAP,NULL);
    Adxl345WriteCmd(ADXL345_REG_FIFO_CTL,0x00);
    // Adxl345ReadCmd(ADXL345_REG_FIFO_CTL,NULL);
}

void Adxl345Init()
{   
    int ret;

    spifd = open(spi_dev, O_RDWR);
    if(spifd < 0)
	{
		printf("open spi dev error \r\n");
        exit(1);
	}
    ret = ioctl(spifd, SPI_IOC_WR_MODE32, &spi_mode);
    ret = ioctl(spifd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
    ret = ioctl(spifd, SPI_IOC_WR_MAX_SPEED_HZ, &spi_speed);

    

    csfd = open(cs_dev, O_RDWR);
    if (csfd < 0)
	{
		printf("Cannot open GPIO value  ,%s\n",cs_dev);
		exit(1);
	}
    Spi_CS_High();

    if(!ReadDeviceID())
        return ;
    ConfigAdxl345();
    printf("adxl345 device init success\n");
    return ;
}

bool Adxl345WriteCmd(unsigned char cmd,unsigned char data)
{
    int ret;
    unsigned char txbuff[4] = {cmd,data};
    unsigned char rxbuff[4];
    struct spi_ioc_transfer tr={0};
    tr.tx_buf = (unsigned long)txbuff;
	tr.rx_buf = (unsigned long)rxbuff;
    tr.len = 4;
	tr.delay_usecs = 1;
    tr.speed_hz = spi_speed;
    tr.bits_per_word = spi_bits;
    Spi_CS_Low();
    ret = ioctl(spifd, SPI_IOC_MESSAGE(1), &tr);
    Spi_CS_High();
	if(ret < 0)
	{
		printf("can't send spi message %d\n", ret);
        	return -1;
	}
	else
	{
		//printf("Write rx = 0x%02x\r\n", rxbuff[0]);
		return ret;
	}
}
bool Adxl345ReadCmd(unsigned char cmd,unsigned char* data)
{
    int ret;
    unsigned char txbuff[2] = {cmd|0x80,0x00};
    unsigned char rxbuff[2] = {};
    struct spi_ioc_transfer tr={0};
    tr.tx_buf = (unsigned long)txbuff;
	tr.rx_buf = (unsigned long)rxbuff;
    tr.len = 2;
	tr.delay_usecs = 1;
    tr.speed_hz = spi_speed;
    tr.bits_per_word = spi_bits;
    
    Spi_CS_Low();
    ret = ioctl(spifd, SPI_IOC_MESSAGE(1), &tr);
    Spi_CS_High();
    if(data!=NULL)
        *data = rxbuff[1];
	if(ret < 0)
	{
		printf("can't send spi message %d\n", ret);
        	return -1;
	}
	else
	{
		//printf("Read rx = 0x%02x  0x%02x \r\n",rxbuff[0],rxbuff[1]);
		return ret;
	}
    return 0;
}

void Adxl345GetData(float* data)
{
    unsigned char acc[6];
    short xyz[3];
    float x,y,z;
    float Angle[3];
    int i;
    int time = 10;
    for(i=0;i<time;i++)
    {
        Adxl345ReadCmd(ADXL345_REG_DATAX0,&acc[0]);
        Adxl345ReadCmd(ADXL345_REG_DATAX1,&acc[1]);
        Adxl345ReadCmd(ADXL345_REG_DATAY0,&acc[2]);
        Adxl345ReadCmd(ADXL345_REG_DATAY1,&acc[3]);
        Adxl345ReadCmd(ADXL345_REG_DATAZ0,&acc[4]);
        Adxl345ReadCmd(ADXL345_REG_DATAZ1,&acc[5]);
        xyz[0] = (short)(acc[0] + ((unsigned short)acc[1] << 8));
        xyz[1] = (short)(acc[2] + ((unsigned short)acc[3] << 8));
        xyz[2] = (short)(acc[4] + ((unsigned short)acc[5] << 8));
        x += xyz[0];
        y += xyz[1];
        z += xyz[2];

    }

    x = (float)xyz[0]*0.0039;
    y = (float)xyz[1]*0.0039;
    z = (float)xyz[2]*0.0039;

    x = x/10.0;
    y = y/10.0;
    z = z/10.0;

    Angle[0] = (atan(x / sqrt(y*y + z * z))) * 180/Pi;
    Angle[1] = (atan(y / sqrt(x*x + z * z))) * 180/Pi;
    Angle[2] = (atan(z / sqrt(x*x + y * y))) * 180/Pi;
    

    memcpy(&data[0],&Angle[0],sizeof(float));
    memcpy(&data[1],&Angle[1],sizeof(float));
    memcpy(&data[2],&Angle[2],sizeof(float));

    usleep(500000);
}

void Adxl345Release()
{
    close(csfd);
    close(spifd);
}
