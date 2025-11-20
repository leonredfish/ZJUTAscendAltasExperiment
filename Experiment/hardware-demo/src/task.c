#include "task.h"
static pthread_t taskid = 0,taskid2 = 0,delay_thread = 0,pca_thread = 0;
static volatile int LED4_time = 0;
extern uint8_t framebuffer[320*480*2];
static volatile uint8_t tens_t = 0;
static volatile uint8_t ones_t = 0;
void Led_Pwm_Test(void)
{
    int flag = 0;
    int time = 0; 
    Ledhandler led;
    Led_Init(LED1,&led);
    while(1)
    {
      time = 0;
      while(!flag)
      {
        Led_High(&led);
        usleep(time*30);
        Led_Low(&led);
        usleep((255-time)*30);
        if((time++)==255)
          flag = 1;
      }
      time = 0;
      while(flag)
      {
        Led_High(&led);
        usleep((255-time)*30);
        Led_Low(&led);
        usleep(time*30);
        if((time++)==255)
          flag = 0;
      }
    }
}

void Led_Test(void)
{
    int time = 1000000; 
    Ledhandler led;
    Led_Init(LED1,&led);
    while(1)
    {
        Led_Low(&led);
        usleep(time);
        Led_High(&led);
        usleep(time);
    }
}

void LED1_Blink_1s(void){
    int time = 1000000;
    Ledhandler led;
    Led_Init(LED1,&led);
    while(1)
    {
        Led_Low(&led);
        usleep(time);
        Led_High(&led);
        usleep(time);
    }
}
void LED4_Blink(void){
    Ledhandler led;
    Led_Init(LED4,&led);
    while(1)
    {
        Led_Low(&led);
        usleep(LED4_time);
        Led_High(&led);
        usleep(LED4_time);
    }
}
static uint8_t ReadKeyS2Status(void)
{
    uint8_t res1 = 0;
    Keyhandler key1;
    Key_Init(KEY3,&key1);
    res1 =  Key_Status(&key1);
    Key_close(&key1);
    return res1 ;
}
static uint8_t ReadKeyS3Status(void)
{
    uint8_t res2 = 0;
    Keyhandler key2;
    Key_Init(KEY3,&key2);
    res2 =  Key_Status(&key2);
    Key_close(&key2);
    return res2 ;
}
static void* S2_LongPressStart_callback_thread(void* arg)
{
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    LED1_Blink_1s();
    return NULL;
}
static void* S3_PressDown_callback_thread(void* arg)
{
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    LED4_Blink();
    return NULL;
}
static void CallBackS2_LongPressStart(void* args)
{
    printf("############### S2 Key LONGPRESS###############\n");
    if (taskid == 0) {
        int ret = pthread_create(&taskid,0,S2_LongPressStart_callback_thread,NULL);
        if (ret != 0) {
            printf("Failed to create thread: %d\n", ret);
            taskid = 0;
        }
    }
}
static void CallBackS2_PressUp(void* args)
{
    printf("############### S2 Key RELEASED###############\n");
    if (taskid != 0) {
        pthread_cancel(taskid);
        pthread_join(taskid, NULL);
        taskid = 0;
    }
}
static void* delay_terminate_thread(void* arg) {
    pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL);
    pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);
    sleep(5);  // 等待5秒
    if (taskid2 != 0) {
        pthread_cancel(taskid2);
        pthread_join(taskid2, NULL);
        taskid2 = 0;
    }

    return NULL;
}

static void CallBackS3_PressDown(void* args)
{
    printf("S3 Key Pressed\n");

    LED4_time = 250000;

    if (delay_thread != 0) {
        pthread_cancel(delay_thread);
        pthread_join(delay_thread, NULL);
        delay_thread = 0;
    }

    if (taskid2 == 0) {  // 确保线程没有在运行
        int ret = pthread_create(&taskid2, NULL, S3_PressDown_callback_thread, NULL);
        if (ret != 0) {
            printf("Failed to create thread: %d\n", ret);
            taskid2 = 0;
        }
    }
}

static void CallBackS3_PressUp(void* args)
{
    printf("S3 Key Released\n");

    LED4_time = 1000000;

    // 创建延迟结束线程
    if (delay_thread != 0) {
        pthread_cancel(delay_thread);
        pthread_join(delay_thread, NULL);
    }

    int ret = pthread_create(&delay_thread, NULL, delay_terminate_thread, NULL);
    if (ret != 0) {
        printf("Failed to create delay thread: %d\n", ret);
        // 立即结束线程
        if (taskid2 != 0) {
            pthread_cancel(taskid2);
            pthread_join(taskid2, NULL);
            taskid2 = 0;
        }
    }
}
/**
 * @brief 按键测试函数
 *
 * 该函数用于初始化和测试两个按钮，设置按钮的回调函数，并在循环中处理按钮事件。
 */
void Key_Test()
{
    // 分配并初始化第一个按钮的内存
    Button*  button1 = (Button*)malloc(sizeof(Button));
    memset(button1,0,sizeof(Button));
    // 分配并初始化第二个按钮的内存
    Button*  button2 = (Button*)malloc(sizeof(Button));
    memset(button2,0,sizeof(Button));
    TPin_level  pin_levels2 = ReadKeyS2Status;
    TPin_level  pin_levels3 = ReadKeyS3Status;
    button_init(button1,pin_levels2,0,1);
    button_init(button2,pin_levels3,0,1);
    button_attach(button1,LONG_PRESS_START,CallBackS2_LongPressStart);
    button_attach(button1,PRESS_UP,CallBackS2_PressUp);
    button_attach(button2,PRESS_DOWN,CallBackS3_PressDown);
    button_attach(button2,PRESS_UP,CallBackS3_PressUp);
    button_start(button1);
    button_start(button2);
    while(1)
    {
        button_ticks();
        usleep(5000);//5ms
    }
    button_stop(button1);
    button_stop(button2);
    free(button1);
    free(button2);
}


void Adxl345_Test()
{
    float buff[3];
    //oled_init();
    Adxl345Init();
    LCD_Init(L2R_U2D,1000);
    LCD_Clear(RED);
    while (1){
        Adxl345GetData(buff);
        GUI_Clear(WHITE);
        //转化为字符串
        char str[10];
        sprintf(str, "%.2f", buff[0]);
        GUI_DisString_EN(40, 80, "AngleX", &Font20, LCD_BACKGROUND, BLUE);
        GUI_DisString_EN(40, 120, str, &Font20, LCD_BACKGROUND, BLUE);
        sprintf(str, "%.2f", buff[1]);
        GUI_DisString_EN(40, 160, "AngleY", &Font20, LCD_BACKGROUND, BLUE);
        GUI_DisString_EN(40, 200, str, &Font20, LCD_BACKGROUND, BLUE);
        sprintf(str, "%.2f", buff[2]);
        GUI_DisString_EN(40, 240, "AngleZ", &Font20, LCD_BACKGROUND, BLUE);
        GUI_DisString_EN(40, 280, str, &Font20, LCD_BACKGROUND, BLUE);
        LCD_SetLocalArea(0, 0, 320, 480, framebuffer, 320*480*2);
        //Refresh();
        printf("AngleX:%0.3f AngleY:%0.3f AngleZ:%0.3f\n",buff[0],buff[1],buff[2]);    // {
    }
}



void Oled_Test(void)
{
    oled_init();
    while (1)
    {
        ColorTurn(TURNOVER_COLOR);//反色显示
        DisplayTurn(NORMAL_DISPLAY);//正常显示
 
        ShowString(0,0,"abcdefg",size1206);//显示ASCII字符    
        ShowString(0,12,"1234567",size1608);//显示ASCII字符    
        ShowString(0,28,"ABCD",size2412);//显示ASCII字符    
        DrawCircle(40,32,20);
        DrawLine(0,0,60,60);
        Refresh();
    }
}

void Oled_Show(float humidity)
{
    ColorTurn(TURNOVER_COLOR);//反色显示
    DisplayTurn(NORMAL_DISPLAY);//正常显示
    char str[10];
    sprintf(str, "%.2f", humidity);
    ShowString(0,0,"Humidity:",size1206);//显示ASCII字符
    ShowString(0,12,str,size1608);//显示ASCII字符
    Refresh();
}

static void* Pca9557_show(void* arg)
{
    pca9557_show();
    return NULL;
}
void Measure(){
    serial_listen_loop("/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0",B9600);
}
/**
 * @brief SHT20温湿度传感器测试函数
 * 该函数初始化SHT20传感器，循环读取温湿度数据并通过OLED显示
 */
void Sht20_Test(void)
{

    Sht20Init("/dev/i2c-7",0x40);
    pca9557_init("/dev/i2c-7");
    LCD_Init(L2R_U2D,1000);
    LCD_Clear(RED);
    if (pca_thread == 0) {
        int ret = pthread_create(&pca_thread,0,Pca9557_show,NULL);
        if (ret != 0) {
            printf("Failed to create thread: %d\n", ret);
            pca_thread = 0;
        }
    }
    //oled_init();
    float buff[2];
    while (1)
    {
        Sht20GetData(&buff);
        GUI_Clear(WHITE);
        //Oled_Show(buff[1]);
        //转化为字符串
        char str[10];
        sprintf(str, "%.2f", buff[0]);
        GUI_DisString_EN(40, 180, "Humidity", &Font20, LCD_BACKGROUND, BLUE);
        GUI_DisString_EN(40, 220, str, &Font20, LCD_BACKGROUND, BLUE);
        LCD_SetLocalArea(0, 0, 320, 480, framebuffer, 320*480*2);
        int temperature = (int)(buff[1] + 0.5f);  // 四舍五入取整
        tens_t = temperature / 10;    // 取十位数字
        ones_t = temperature % 10;   // 取个位数字
        pca9557_setnum(10,10,tens_t,ones_t);
        //Refresh();
        usleep(2000000);
    }
    Sht20Close();
}


void Pca9557_Test(void)
{
    pca9557_init("/dev/i2c-7");
    pca9557_setnum(1,2,3,4);
    pca9557_show();
}

void Ds1399u_Test()
{
    rtems_time_of_day tod={
        .year=2024,
        .month=6,
        .day=30,
        .hour=12,
        .minute=30,
        .second=30,
        .ticks =0
    };
    rtc_set_time(&tod);
    memset(&tod,0,sizeof(tod));
    rtc_get_time(&tod);
    printf("%d-%d-%d %d:%d:%d\n",tod.year,tod.month,tod.day,tod.hour,tod.minute,tod.second);

    while(1)
    {
        sleep(1);
        rtc_get_time(&tod);
        printf("%d-%d-%d %d:%d:%d\n",tod.year,tod.month,tod.day,tod.hour,tod.minute,tod.second);
    }
}

void Lcd_test(void)
{
    LCD_Init(L2R_U2D,1000);
    LCD_Clear(RED);
    GUI_Show();
    LCD_Exit();
}