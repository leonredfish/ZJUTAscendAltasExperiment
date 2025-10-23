#ifdef __cplusplus
extern "C"{
#endif

#include "task.h"
#include "unistd.h" 
#include <getopt.h>

// #include "demo_ili9488.h"

#ifdef __cplusplus
}
#endif


static int optIdx=0;
static struct option longOpt[] = {
    {"led",no_argument,0,'l'},
    {"pwm",no_argument,0,'w'},
    {"key",no_argument,0, 'k'},
    {"lcd",no_argument,0,'c'},
    {"oled",no_argument,0,'o'},
    {"gyroscope",no_argument,0,'g'},
    {"rtc",no_argument,0,'r'},
    {"t&h",no_argument,0,'t'},
    {"digital",no_argument,0,'d'},
    {"video",no_argument,0,'v'},
    {"help",no_argument,0,'h'}
};

void GetHelp()
{
    printf("\033[0;33;34m--help         -h    实验内容 \033[m \n");
    printf("\033[0;33;34m--led          -l    Led 控制实验 \033[m \r\n");
    printf("\033[0;33;34m--pwm          -w    呼吸灯实验  \033[m \r\n");
    printf("\033[0;33;34m--key          -k    按键实验   \033[m \r\n");
    printf("\033[0;33;34m--gyroscope    -g    Spi驱动和Adxl345三轴加速度计实验 \033[m \r\n");
    printf("\033[0;33;34m--lcd          -c    Spi驱动和LCD实验 \033[m\r\n");  
    printf("\033[0;33;34m--oled         -o    I2C驱动和Oled屏实验 \033[m \r\n");
    printf("\033[0;33;34m--t&h          -t    I2C驱动和温度传感器实验 \033[m\r\n");
    printf("\033[0;33;34m--digital      -d    I2C驱动和数码管实验 \033[m\r\n");   
    printf("\033[0;33;34m--video        -v    v4l2摄像头实验 \033[m\r\n");  
}



int main(int argc,char* argv[])
{
    int c;
    c = getopt_long(argc, argv,"hlwkgcotdv",longOpt,&optIdx);
    switch(c)
    {
        case 'l':
        {
            Led_Test();
        }break;
        case 'w':
        {
            Led_Pwm_Test();
        }break;
        case 'k':
        {
            Key_Test();
        }break;
        case 'c':
        {
            Lcd_test();
        }break;
        case 'o':
        {
            Oled_Test();
        }break;
        case 'g':
        {
            Adxl345_Test();
        }break;
        case 't':
        {
            Sht20_Test();
        }break;
        case 'd':
        {
            Pca9557_Test();
        }break;
        case 'v':
        {
            v4l2_test();
        }break;
        case 'h':
        {
            GetHelp();
        }break;
        default :{
            GetHelp();
        }
    }
    return 0;
}