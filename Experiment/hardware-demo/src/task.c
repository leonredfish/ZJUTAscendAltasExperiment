#include "task.h"

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




static uint8_t ReadKeyS4Status(void)
{
    uint8_t res = 0;
    Keyhandler key;
    Key_Init(KEY3,&key);
    res =  Key_Status(&key);
    Key_close(&key);
    return res ;
}
static void* S4_pressDown_callback_thread(void* args)
{
    printf("############### S4 Key  PRESS_DOWN ###############\n");
    Led_Test();
    return 0;
    
}
static void CallBackS4_PressDown(void* args)
{
    pthread_t taskid;
    pthread_create(&taskid,0,S4_pressDown_callback_thread,NULL);
    //pthread_join(taskid,NULL);
}
void Key_Test()
{
    Button*  button = (Button*)malloc(sizeof(Button));
    memset(button,0,sizeof(Button));
    TPin_level  pin_levels2 = ReadKeyS4Status;

    button_init(button,pin_levels2,0,1);
    button_attach(button,PRESS_DOWN,CallBackS4_PressDown);
    button_start(button);
    while(1)
    {
        button_ticks();
        usleep(5000);//5ms
    }
    button_stop(button);
    free(button);
}


void Adxl345_Test()
{
    float buff[3];
    Adxl345Init();
    while (1){
        Adxl345GetData(buff);
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



void Sht20_Test(void)
{

    Sht20Init("/dev/i2c-7",0x40);
    float buff[2];
    while (1)
    {
        Sht20GetData(buff);
        printf("温度为:%f \t湿度为:%f\n",buff[0],buff[1]);
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
        .ticks =1
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