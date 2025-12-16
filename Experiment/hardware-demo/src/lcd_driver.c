#include "lcd_driver.h"

#define LCD_SPI_DEV			"/dev/spidev7.0"
#define LCD_SPI_MODE 		0x00
#define LCD_SPI_BITS		8
#define LCD_SPI_SPEED		25000000

static int8_t isInit=0;
static int lcdspifd;
static int lcdrstfd;
static int lcddcfd;
static int lcdcsfd;

LCD_DIS sLCD_DIS;

void LCD_DC_SET() // DC_LOW write data
{
    write(lcddcfd,"1",2);
}
void LCD_DC_CLR()  // DC_LOW write reg
{
    write(lcddcfd,"0",2);
}
void LCD_CS_CLR()
{
    write(lcdcsfd,"0",2);
}
void LCD_CS_SET()
{
    write(lcdcsfd,"1",2);
}


static void LCD_Reset(void)
{
    write(lcdrstfd,"1",2);
    usleep(200000);
    write(lcdrstfd,"0",2);
    usleep(200000);
    write(lcdrstfd,"1",2);
    usleep(200000);
}

static void LCD_InitResource(void)
{
    int ret;
    uint8_t mode = LCD_SPI_MODE;
    uint8_t bits = LCD_SPI_BITS;
    unsigned int speed = 1000000;
    lcdspifd = open(LCD_SPI_DEV,O_RDWR);
    if(lcdspifd<0)
    {
        printf("Open spi dev error\r\n");
        return ;
    }
    if(ioctl(lcdspifd,SPI_IOC_WR_MODE,&mode)<0)
    {
        printf("Failed to set SPI mode\r\n");
        close(lcdspifd);
        return ;
    }
    if(ioctl(lcdspifd,SPI_IOC_WR_BITS_PER_WORD,&bits) <0)
    {
        printf("Failed to set SPI bits per word\r\n");
        close(lcdspifd);
        return ;
    }
    ret = ioctl(lcdspifd,SPI_IOC_WR_MAX_SPEED_HZ,&speed);
    lcddcfd = open("/sys/class/gpio/gpio81/value",O_RDWR);
    if(lcddcfd <0)
    {
        printf("Open gpio81 error\r\n");
        close(lcdspifd);
        return ;
    }
    lcdrstfd = open("/sys/class/gpio/gpio82/value",O_RDWR);
    if( lcdrstfd<0)
    {
        printf("Open gpio82 error\r\n");
        close(lcdspifd);
        close(lcddcfd);
        return ;
    }
    lcdcsfd = open("/sys/class/gpio/gpio226/value",O_RDWR);
    if(lcdcsfd< 0)
    {
        printf("Open gpio226 error\r\n");
        return ;
    }
}
static void LCD_ReleaseResource(void)
{
    close(lcdcsfd);
    close(lcddcfd);
    close(lcdrstfd);
    close(lcdspifd);
}


static void LCD_SetBackLight(uint16_t value)
{
//	PWM_SetValue(value);
}

int8_t SpiWriteByte(uint8_t data)
{
    uint8_t tx_buffer[2] = {0};
    uint8_t rx_buffer[2] = {0};
    tx_buffer[0] = data;
    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)rx_buffer,
        .len = 1,
        .delay_usecs = 0,
        .speed_hz = LCD_SPI_SPEED,  // SPI speed in Hz
        .bits_per_word = LCD_SPI_BITS,
    };
    if (ioctl(lcdspifd, SPI_IOC_MESSAGE(1), &transfer) < 0) 
    {
        printf("Failed to perform SPI transfer  line:%d \r\n",__LINE__);
        return -1;
    }
}
int8_t SpiWriteRead(uint8_t* tx, uint8_t* rx, int len)
{
    uint8_t tx_buffer[2] = {0};
    uint8_t rx_buffer[2] = {0};
    memcpy(tx_buffer,tx,len);
    struct spi_ioc_transfer transfer = {
        .tx_buf = (unsigned long)tx_buffer,
        .rx_buf = (unsigned long)rx_buffer,
        .len = len,
        .delay_usecs = 0,
        .speed_hz = LCD_SPI_SPEED,  // SPI speed in Hz
        .bits_per_word = LCD_SPI_BITS,
    };
    if (ioctl(lcdspifd, SPI_IOC_MESSAGE(1), &transfer) < 0) 
    {
        printf("Failed to perform SPI transfer  line:%d \r\n",__LINE__);
        return -1;
    }
    memcpy(rx,rx_buffer,len);
} 

uint8_t lcd_write_bytes(const uint8_t* data, uint32_t len)
{
    uint8_t tx_buf[2048];
    struct spi_ioc_transfer transfer = {0};
    uint16_t i=0;
    LCD_CS_CLR();
    LCD_DC_SET();
    while(len)
    {
        if(!(len/2048))
        {
            memcpy(tx_buf,data,len);
            transfer.tx_buf = (unsigned long)tx_buf;
            transfer.len = len;
            len -= len;
        }
        else
        {
            memcpy(tx_buf,data,2048);
            transfer.tx_buf = (unsigned long)tx_buf;
            transfer.len = 2048;
            len = len-2048;
            data = data+2048;
        }

        transfer.speed_hz = LCD_SPI_SPEED;
        transfer.bits_per_word = LCD_SPI_BITS;
        if (ioctl(lcdspifd, SPI_IOC_MESSAGE(1), &transfer) < 0) 
        {
            printf("Failed to perform SPI transfer  line:%d \r\n",__LINE__);
            return -1;
        }
    }
    LCD_CS_SET();
}


void LCD_WriteReg(uint8_t Reg)
{
    LCD_DC_CLR();      
    LCD_CS_CLR();
    SpiWriteByte(Reg);
    LCD_CS_SET();
}

void LCD_WriteData(uint16_t Data)
{
    LCD_DC_SET();      
    LCD_CS_CLR();
    SpiWriteByte(Data>>8);
    SpiWriteByte(Data&0xFF);
    LCD_CS_SET();
}

static void LCD_Write_AllData(uint16_t Data, uint32_t DataLen)
{
    uint32_t i;
    uint8_t *buf = NULL;
    buf = malloc(DataLen*2);
    if(buf)
    {
        for (i = 0; i < DataLen; i++)
        {
            buf[i * 2] =  Data>>8;
            buf[i * 2 + 1] =  Data & 0xFF;
        }
        lcd_write_bytes(buf,DataLen*2);
        free(buf);
    }
    else
    {
        LCD_DC_SET();  
        LCD_CS_CLR();
        for(i = 0; i < DataLen; i++) {
            SpiWriteByte(Data >> 8);
            SpiWriteByte(Data & 0XFF);
        }
        LCD_CS_SET();
    }

}

static void LCD_InitReg(void)
{
    LCD_WriteReg(0x21);

    LCD_WriteReg(0xC2);	//Normal mode, increase can change the display quality, while increasing power consumption
    LCD_WriteData(0x33);

    LCD_WriteReg(0XC5);
    LCD_WriteData(0x00);
    LCD_WriteData(0x1e);//VCM_REG[7:0]. <=0X80.
    LCD_WriteData(0x80);

    LCD_WriteReg(0xB1);//Sets the frame frequency of full color normal mode
    LCD_WriteData(0xB0);//0XB0 =70HZ, <=0XB0.0xA0=62HZ

    LCD_WriteReg(0x36);
    LCD_WriteData(0x28); //2 DOT FRAME MODE,F<=70HZ.

    LCD_WriteReg(0XE0);
    LCD_WriteData(0x0);
    LCD_WriteData(0x13);
    LCD_WriteData(0x18);
    LCD_WriteData(0x04);
    LCD_WriteData(0x0F);
    LCD_WriteData(0x06);
    LCD_WriteData(0x3a);
    LCD_WriteData(0x56);
    LCD_WriteData(0x4d);
    LCD_WriteData(0x03);
    LCD_WriteData(0x0a);
    LCD_WriteData(0x06);
    LCD_WriteData(0x30);
    LCD_WriteData(0x3e);
    LCD_WriteData(0x0f);		

    LCD_WriteReg(0XE1);
    LCD_WriteData(0x0);
    LCD_WriteData(0x13);
    LCD_WriteData(0x18);
    LCD_WriteData(0x01);
    LCD_WriteData(0x11);
    LCD_WriteData(0x06);
    LCD_WriteData(0x38);
    LCD_WriteData(0x34);
    LCD_WriteData(0x4d);
    LCD_WriteData(0x06);
    LCD_WriteData(0x0d);
    LCD_WriteData(0x0b);
    LCD_WriteData(0x31);
    LCD_WriteData(0x37);
    LCD_WriteData(0x0f);

    LCD_WriteReg(0X3A);	//Set Interface Pixel Format
    LCD_WriteData(0x55);

    LCD_WriteReg(0x11);//sleep out
    usleep(120000);
    LCD_WriteReg(0x29);//Turn on the LCD display
}


void LCD_SetGramScanWay(LCD_SCAN_DIR Scan_dir)
{
    uint16_t MemoryAccessReg_Data = 0; //addr:0x36
    uint16_t DisFunReg_Data = 0; //addr:0xB6

    //Pico-ResTouch-LCD-3.5
    // Gets the scan direction of GRAM
    switch (Scan_dir) {
    case L2R_U2D:
        /* Memory access control: MY = 0, MX = 0, MV = 0, ML = 0 */
        /* Display Function control: NN = 0, GS = 0, SS = 1, SM = 0	*/
        MemoryAccessReg_Data = 0x08;
        DisFunReg_Data = 0x22;
        break;
    case R2L_D2U: 
        /* Memory access control: MY = 0, MX = 0, MV = 0, ML = 0 */
        /* Display Function control: NN = 0, GS = 1, SS = 0, SM = 0	*/
        MemoryAccessReg_Data = 0x08;
        DisFunReg_Data = 0x42;
        break;
    case U2D_R2L: //0X6
        /* Memory access control: MY = 0, MX = 0, MV = 1, ML = 0 	X-Y Exchange*/
        /* Display Function control: NN = 0, GS = 0, SS = 0, SM = 0	*/
        MemoryAccessReg_Data = 0x28;
        DisFunReg_Data = 0x02;
        break;
    case D2U_L2R: //0XA
        /* Memory access control: MY = 0, MX = 0, MV = 1, ML = 0 	X-Y Exchange*/
        /* Display Function control: NN = 0, GS = 1, SS = 1, SM = 0	*/
        MemoryAccessReg_Data = 0x28;
        DisFunReg_Data = 0x62;
        break;
    }

    //Get the screen scan direction
    sLCD_DIS.LCD_Scan_Dir = Scan_dir;

    //Get GRAM and LCD width and height
    //480*320,horizontal default
    if(Scan_dir == L2R_U2D || Scan_dir == R2L_D2U) {
        sLCD_DIS.LCD_Dis_Column	= LCD_3_5_HEIGHT ;
        sLCD_DIS.LCD_Dis_Page = LCD_3_5_WIDTH ;
    } else {
        sLCD_DIS.LCD_Dis_Column	= LCD_3_5_WIDTH ;
        sLCD_DIS.LCD_Dis_Page = LCD_3_5_HEIGHT ;
    }

    // Set the read / write scan direction of the frame memory
    LCD_WriteReg(0xB6);
    LCD_WriteData(0X00);
    LCD_WriteData(DisFunReg_Data);

    LCD_WriteReg(0x36);
    LCD_WriteData(MemoryAccessReg_Data);
}


void BMP_SetGramScanWay(LCD_SCAN_DIR Scan_dir)
{
    uint16_t MemoryAccessReg_Data = 0; //addr:0x36
    uint16_t DisFunReg_Data = 0; //addr:0xB6
    //Pico-ResTouch-LCD-3.5
    // Gets the scan direction of GRAM
    switch (Scan_dir) {
    case L2R_U2D:
        /* Memory access control: MY = 0, MX = 1, MV = 0, ML = 0 */
        /* Display Function control: NN = 0, GS = 0, SS = 1, SM = 0	*/
        MemoryAccessReg_Data = 0x48;
        DisFunReg_Data = 0x22;
        break;
    case R2L_D2U: 
        /* Memory access control: MY = 0, MX = 1, MV = 0, ML = 0 */
        /* Display Function control: NN = 0, GS = 1, SS = 0, SM = 0	*/
        MemoryAccessReg_Data = 0x48;
        DisFunReg_Data = 0x42;
        break;
    case U2D_R2L: 
        /* Memory access control: MY = 1, MX = 0, MV = 1, ML = 0 	X-Y Exchange*/
        /* Display Function control: NN = 0, GS = 0, SS = 0, SM = 0	*/
        MemoryAccessReg_Data = 0xA8;
        DisFunReg_Data = 0x02;
        break;
    case D2U_L2R: 
        /* Memory access control: MY = 1, MX = 0, MV = 1, ML = 0 	X-Y Exchange*/
        /* Display Function control: NN = 0, GS = 1, SS = 1, SM = 0	*/
        MemoryAccessReg_Data = 0xA8;
        DisFunReg_Data = 0x62;
        break;
    }

    //Get the screen scan direction
    sLCD_DIS.LCD_Scan_Dir = Scan_dir;

    //Get GRAM and LCD width and height
    //480*320,horizontal default
    if(Scan_dir == L2R_U2D || Scan_dir == R2L_D2U) {
        sLCD_DIS.LCD_Dis_Column	= LCD_3_5_HEIGHT ;
        sLCD_DIS.LCD_Dis_Page = LCD_3_5_WIDTH ;
    } else {
        sLCD_DIS.LCD_Dis_Column	= LCD_3_5_WIDTH ;
        sLCD_DIS.LCD_Dis_Page = LCD_3_5_HEIGHT ;
    }

    // Set the read / write scan direction of the frame memory
    LCD_WriteReg(0xB6);
    LCD_WriteData(0X00);
    LCD_WriteData(DisFunReg_Data);

    LCD_WriteReg(0x36);
    LCD_WriteData(MemoryAccessReg_Data);
}


void LCD_Init(LCD_SCAN_DIR LCD_ScanDir, uint16_t LCD_BLval)
{
    if(isInit)
        return;
    LCD_InitResource();

    LCD_Reset();//Hardware reset

    LCD_InitReg();//Set the initialization register
	
	if(LCD_BLval > 1000)
		LCD_BLval = 1000;
	
	LCD_SetGramScanWay(LCD_ScanDir);//Set the display scan and color transfer modes
    
	usleep(200000);
    isInit = 1;
}

void LCD_Exit()
{
    if(!isInit)
        return;
    LCD_ReleaseResource();
    isInit = 0;
}
void LCD_SetWindow(POINT Xstart, POINT Ystart,	POINT Xend, POINT Yend)
{	

	//set the X coordinates
	LCD_WriteReg(0x2A);
	LCD_WriteData(Xstart >> 8);	 		//Set the horizontal starting point to the high octet
	LCD_WriteData(Xstart & 0xff);	 	//Set the horizontal starting point to the low octet
	LCD_WriteData((Xend - 1) >> 8);		//Set the horizontal end to the high octet
	LCD_WriteData((Xend - 1) & 0xff);	//Set the horizontal end to the low octet

	//set the Y coordinates
	LCD_WriteReg(0x2B);
	LCD_WriteData(Ystart >> 8);
	LCD_WriteData(Ystart & 0xff );
	LCD_WriteData((Yend - 1) >> 8);
	LCD_WriteData((Yend - 1) & 0xff);

    LCD_WriteReg(0x2C);
}

void LCD_SetCursor(POINT Xpoint, POINT Ypoint)
{
	LCD_SetWindow(Xpoint, Ypoint, Xpoint, Ypoint);
}

void LCD_SetColor(COLOR Color , POINT Xpoint, POINT Ypoint)
{
    LCD_Write_AllData(Color , (uint32_t)Xpoint * (uint32_t)Ypoint);
}

void LCD_SetPointlColor( POINT Xpoint, POINT Ypoint, COLOR Color)
{
    if ((Xpoint <= sLCD_DIS.LCD_Dis_Column) && (Ypoint <= sLCD_DIS.LCD_Dis_Page)) {
        LCD_SetCursor(Xpoint, Ypoint);
        LCD_SetColor(Color, 1, 1);
    }
}

void LCD_SetArealColor(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,	COLOR Color)
{
    if((Xend > Xstart) && (Yend > Ystart)) {
        LCD_SetWindow(Xstart , Ystart , Xend , Yend);
        LCD_SetColor( Color , Xend - Xstart, Yend - Ystart);
    }
}
void LCD_SetLocalArea(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,uint8_t* buffer, uint32_t len)
{
    LCD_SetWindow(Xstart , Ystart , Xend , Yend);
    if(buffer)
    {
        lcd_write_bytes(buffer,len);
    }
}

void LCD_Clear(COLOR  Color)
{
    LCD_SetArealColor(0, 0, sLCD_DIS.LCD_Dis_Column , sLCD_DIS.LCD_Dis_Page , Color);
}

uint8_t LCD_Read_Id(void)
{
	uint8_t reg = 0xDC;
	uint8_t tx_val = 0x00;
	uint8_t rx_val;
    LCD_CS_CLR();
    LCD_DC_CLR();
    SpiWriteByte(reg);
	SpiWriteRead(&tx_val,&rx_val,1);
    LCD_CS_SET();
    printf("LCD id = 0x%02x\r\n",rx_val);
	return rx_val;
}