#include "key.h"

void Key_Init(int gpioNum,Keyhandler* key)
{
    char buff[100] = {0};
    int tempfd = open("/sys/class/gpio/export", O_WRONLY);
    if (tempfd < 0)
	{
		printf("Cannot open GPIO to export it\n");
		exit(1);
	}
    memset(buff,0,100);
    sprintf(buff,"%d",gpioNum);
    write(tempfd,buff,4);
    close(tempfd);


    memset(buff,0,100);
    sprintf(buff,"/sys/class/gpio/gpio%d/direction",gpioNum);
    tempfd = open(buff,O_RDWR);
    if (tempfd < 0)
	{
		printf("Cannot open GPIO direction it\n");
		exit(1);
	}
    write(tempfd, "in", 3);
    close(tempfd);


    memset(buff,0,100);
    sprintf(buff,"/sys/class/gpio/gpio%d/value",gpioNum);
    key->fd = open(buff,O_RDWR);
    if(key->fd < 0)
    {
        printf("Open /sys/class/gpio/gpio%d/value Error\n",gpioNum);
    }
}

char Key_Status(Keyhandler* key)
{
    int res;
    char  status[4]= {0};
    res = read(key->fd,&status,sizeof(status));
    if(res < 0)
    {
        printf("Read status error\n");
    }
    close(key->fd);
    if(strchr(status,'1') !=NULL )
        return 1;
    else return 0;   
}

void Key_close(Keyhandler* key)
{
    memset(key,0,sizeof(Keyhandler));
}