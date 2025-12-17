#include "lcd_gui.h"
#include "image_circle.h"
#include "image_square.h"
#include "image_triangle.h"
#include <stdio.h>
#include <string.h>

extern LCD_DIS sLCD_DIS;

uint8_t framebuffer[320*480*2]; 

// 默认的测量结果文件路径，可根据部署位置调整。
#define MEASURE_FILE_PATH "../code/detection_logs/measurement_data.txt"

// 读取测量文件的首行，格式：<class> <width> <distance>
static int load_measurement_data(int *cls, float *distance, float *width)
{
    FILE *fp = fopen(MEASURE_FILE_PATH, "r");
    if (!fp) {
        perror("fopen measurement file");
        return -1;
    }

    char line[128] = {0};
    if (fgets(line, sizeof(line), fp) == NULL) {
        fclose(fp);
        return -1;
    }

    fclose(fp);

    if (sscanf(line, "%d %f %f", cls, distance, width) != 3) {
        return -1;
    }
    return 0;
}
/******************************************************************************
function:	Coordinate conversion
******************************************************************************/
void GUI_Swop(POINT Point1, POINT Point2)
{
    POINT Temp;
    Temp = Point1;
    Point1 = Point2;
    Point2 = Temp;
}

/******************************************************************************
function:	Coordinate conversion
******************************************************************************/
void GUI_Clear(COLOR Color)
{
    for(uint32_t i=0;i<320*480*2;i+=2)
    {
        framebuffer[i] = Color >> 8;
        framebuffer[i+1] = Color & 0xFF;
    }
    LCD_SetLocalArea(0, 0, 320, 480, framebuffer, 320*480*2);
}

static void GUI_SetPointlColor(uint16_t x, uint16_t y, COLOR Color)
{
    if(x >= sLCD_DIS.LCD_Dis_Column  || y>= sLCD_DIS.LCD_Dis_Page)
        return ;
    // printf("[INFO] %hd %hd\r\n", x, y);
    framebuffer[2*(320*y + x)] = Color >> 8;
    framebuffer[2*(320*y + x)+1] = Color & 0xFF;

}



/******************************************************************************
function:	Draw Point(Xpoint, Ypoint) Fill the color
parameter:
	Xpoint		:   The x coordinate of the point
	Ypoint		:   The y coordinate of the point
	Color		:   Set color
	Dot_Pixel	:	point size
******************************************************************************/
void GUI_DrawPoint(POINT Xpoint, POINT Ypoint, COLOR Color,
                   DOT_PIXEL Dot_Pixel, DOT_STYLE DOT_STYLE)
{
    if(Xpoint > sLCD_DIS.LCD_Dis_Column || Ypoint > sLCD_DIS.LCD_Dis_Page) {
        //DEBUG("GUI_DrawPoint Input exceeds the normal display range\r\n");
        return;
    }

    
    uint16_t XDir_Num , YDir_Num;
    if(DOT_STYLE == DOT_FILL_AROUND) {
        for(XDir_Num = 0; XDir_Num < 2 * Dot_Pixel; XDir_Num++) {
            for(YDir_Num = 0; YDir_Num < 2 * Dot_Pixel; YDir_Num++) {
                GUI_SetPointlColor(Xpoint + XDir_Num - Dot_Pixel, Ypoint + YDir_Num - Dot_Pixel, Color);
            }
        }
    } else {
        for(XDir_Num = 0; XDir_Num <  Dot_Pixel; XDir_Num++) {
            for(YDir_Num = 0; YDir_Num <  Dot_Pixel; YDir_Num++) {
                GUI_SetPointlColor(Xpoint + XDir_Num - 1, Ypoint + YDir_Num - 1, Color);
            }
        }
    }
}

/******************************************************************************
function:	Draw a line of arbitrary slope
parameter:
	Xstart ：Starting x point coordinates
	Ystart ：Starting x point coordinates
	Xend   ：End point x coordinate
	Yend   ：End point y coordinate
	Color  ：The color of the line segment
******************************************************************************/
void GUI_DrawLine(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,
                  COLOR Color, DOT_PIXEL Dot_Pixel)
{
    if(Xstart > sLCD_DIS.LCD_Dis_Column || Ystart > sLCD_DIS.LCD_Dis_Page ||
       Xend > sLCD_DIS.LCD_Dis_Column || Yend > sLCD_DIS.LCD_Dis_Page) {
        //DEBUG("GUI_DrawLine Input exceeds the normal display range\r\n");
        return;
    }

    if(Xstart > Xend)
        GUI_Swop(Xstart, Xend);
    if(Ystart > Yend)
        GUI_Swop(Ystart, Yend);

    if(Xstart == Xend)
    {
        for(uint32_t i=Ystart;i<Yend;i++)
        {
            GUI_DrawPoint(Xstart, i, Color, Dot_Pixel, DOT_FILL_AROUND);
        }
    }
    else if(Ystart == Yend)
    {
        for(uint32_t i=Xstart;i<Xend;i++)
        {
            GUI_DrawPoint(i, Ystart, Color, Dot_Pixel, DOT_FILL_AROUND);
        }
    }
}

/******************************************************************************
function:	Draw a rectangle
parameter:
	Xstart ：Rectangular  Starting x point coordinates
	Ystart ：Rectangular  Starting x point coordinates
	Xend   ：Rectangular  End point x coordinate
	Yend   ：Rectangular  End point y coordinate
	Color  ：The color of the Rectangular segment
	Filled : Whether it is filled--- 1 solid 0：empty
******************************************************************************/
void GUI_DrawRectangle(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,
                       COLOR Color, DRAW_FILL Filled, DOT_PIXEL Dot_Pixel)
{
    if(Xstart > sLCD_DIS.LCD_Dis_Column || Ystart > sLCD_DIS.LCD_Dis_Page ||
       Xend > sLCD_DIS.LCD_Dis_Column || Yend > sLCD_DIS.LCD_Dis_Page) {
        //DEBUG("Input exceeds the normal display range\r\n");
        return;
    }

    if(Xstart > Xend)
        GUI_Swop(Xstart, Xend);
    if(Ystart > Yend)
        GUI_Swop(Ystart, Yend);

    
    if(Filled ) {
	//#if LOW_Speed_Show
		POINT Ypoint;
        for(Ypoint = Ystart; Ypoint < Yend; Ypoint++) {
            GUI_DrawLine(Xstart, Ypoint, Xend, Ypoint, Color , Dot_Pixel);
        }
	//#elif HIGH_Speed_Show
	//	LCD_SetArealColor( Xstart, Ystart, Xend, Yend, Color);
	//#endif
    } else {
        GUI_DrawLine(Xstart, Ystart, Xend, Ystart, Color , Dot_Pixel);
        GUI_DrawLine(Xstart, Ystart, Xstart, Yend, Color , Dot_Pixel);
        GUI_DrawLine(Xstart, Yend, Xend, Yend, Color , Dot_Pixel);
        GUI_DrawLine(Xend, Ystart, Xend, Yend, Color , Dot_Pixel);
    }
}

/******************************************************************************
function:	Use the 8-point method to draw a circle of the
				specified size at the specified position.
parameter:
	X_Center  ：Center X coordinate
	Y_Center  ：Center Y coordinate
	Radius    ：circle Radius
	Color     ：The color of the ：circle segment
	Filled    : Whether it is filled: 1 filling 0：Do not
******************************************************************************/
void GUI_DrawCircle(POINT X_Center, POINT Y_Center, LENGTH Radius,
                    COLOR Color, DRAW_FILL  Draw_Fill , DOT_PIXEL Dot_Pixel)
{
    if(X_Center > sLCD_DIS.LCD_Dis_Column || Y_Center >= sLCD_DIS.LCD_Dis_Page) {
        //DEBUG("GUI_DrawCircle Input exceeds the normal display range\r\n");
        return;
    }

    //Draw a circle from(0, R) as a starting point
    int16_t XCurrent, YCurrent;
    XCurrent = 0;
    YCurrent = Radius;

    //Cumulative error,judge the next point of the logo
    int16_t Esp = 3 - (Radius << 1 );

    int16_t sCountY;
    if(Draw_Fill == DRAW_FULL) {
        while(XCurrent <= YCurrent ) { //Realistic circles
            for(sCountY = XCurrent; sCountY <= YCurrent; sCountY ++ ) {
                GUI_DrawPoint(X_Center + XCurrent, Y_Center + sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT );//1
                GUI_DrawPoint(X_Center - XCurrent, Y_Center + sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT );//2
                GUI_DrawPoint(X_Center - sCountY, Y_Center + XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT );//3
                GUI_DrawPoint(X_Center - sCountY, Y_Center - XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT );//4
                GUI_DrawPoint(X_Center - XCurrent, Y_Center - sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT );//5
                GUI_DrawPoint(X_Center + XCurrent, Y_Center - sCountY, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT );//6
                GUI_DrawPoint(X_Center + sCountY, Y_Center - XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT );//7
                GUI_DrawPoint(X_Center + sCountY, Y_Center + XCurrent, Color, DOT_PIXEL_DFT, DOT_STYLE_DFT );
            }
            if(Esp < 0 )
                Esp += 4 * XCurrent + 6;
            else {
                Esp += 10 + 4 * (XCurrent - YCurrent );
                YCurrent --;
            }
            XCurrent ++;
        }
    } else { //Draw a hollow circle
        while(XCurrent <= YCurrent ) {
            GUI_DrawPoint(X_Center + XCurrent, Y_Center + YCurrent, Color, Dot_Pixel, DOT_STYLE_DFT );//1
            GUI_DrawPoint(X_Center - XCurrent, Y_Center + YCurrent, Color, Dot_Pixel, DOT_STYLE_DFT );//2
            GUI_DrawPoint(X_Center - YCurrent, Y_Center + XCurrent, Color, Dot_Pixel, DOT_STYLE_DFT );//3
            GUI_DrawPoint(X_Center - YCurrent, Y_Center - XCurrent, Color, Dot_Pixel, DOT_STYLE_DFT );//4
            GUI_DrawPoint(X_Center - XCurrent, Y_Center - YCurrent, Color, Dot_Pixel, DOT_STYLE_DFT );//5
            GUI_DrawPoint(X_Center + XCurrent, Y_Center - YCurrent, Color, Dot_Pixel, DOT_STYLE_DFT );//6
            GUI_DrawPoint(X_Center + YCurrent, Y_Center - XCurrent, Color, Dot_Pixel, DOT_STYLE_DFT );//7
            GUI_DrawPoint(X_Center + YCurrent, Y_Center + XCurrent, Color, Dot_Pixel, DOT_STYLE_DFT );//0

            if(Esp < 0 )
                Esp += 4 * XCurrent + 6;
            else {
                Esp += 10 + 4 * (XCurrent - YCurrent );
                YCurrent --;
            }
            XCurrent ++;
        }
    }
}

/******************************************************************************
function:	Show English characters
parameter:
	Xpoint           ：X coordinate
	Ypoint           ：Y coordinate
	Acsii_Char       ：To display the English characters
	Font             ：A structure pointer that displays a character size
	Color_Background : Select the background color of the English character
	Color_Foreground : Select the foreground color of the English character
******************************************************************************/
void GUI_DisChar(POINT Xpoint, POINT Ypoint, const char Acsii_Char,
                 sFONT* Font, COLOR Color_Background, COLOR Color_Foreground)
{
    POINT Page, Column;

    if(Xpoint > sLCD_DIS.LCD_Dis_Column || Ypoint > sLCD_DIS.LCD_Dis_Page) {
        //DEBUG("GUI_DisChar Input exceeds the normal display range\r\n");
        return;
    }

    uint32_t Char_Offset = (Acsii_Char - ' ') * Font->Height * (Font->Width / 8 + (Font->Width % 8 ? 1 : 0));
    const unsigned char *ptr = &Font->table[Char_Offset];

    for(Page = 0; Page < Font->Height; Page ++ ) {
        for(Column = 0; Column < Font->Width; Column ++ ) {

            //To determine whether the font background color and screen background color is consistent
            if(FONT_BACKGROUND == Color_Background) { //this process is to speed up the scan
                if(*ptr & (0x80 >> (Column % 8)))
                    GUI_DrawPoint(Xpoint + Column, Ypoint + Page, Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
            } else {
                if(*ptr & (0x80 >> (Column % 8))) {
                    GUI_DrawPoint(Xpoint + Column, Ypoint + Page, Color_Foreground, DOT_PIXEL_DFT, DOT_STYLE_DFT);
                } else {
                    GUI_DrawPoint(Xpoint + Column, Ypoint + Page, Color_Background, DOT_PIXEL_DFT, DOT_STYLE_DFT);
                }
            }
            //One pixel is 8 bits
            if(Column % 8 == 7)
                ptr++;
        }/* Write a line */
        if(Font->Width % 8 != 0)
            ptr++;
    }/* Write all */
}

/******************************************************************************
function:	Display the string
parameter:
	Xstart           ：X coordinate
	Ystart           ：Y coordinate
	pString          ：The first address of the English string to be displayed
	Font             ：A structure pointer that displays a character size
	Color_Background : Select the background color of the English character
	Color_Foreground : Select the foreground color of the English character
******************************************************************************/
void GUI_DisString_EN(POINT Xstart, POINT Ystart, const char * pString,
                      sFONT* Font, COLOR Color_Background, COLOR Color_Foreground )
{
    POINT Xpoint = Xstart;
    POINT Ypoint = Ystart;

    if(Xstart > sLCD_DIS.LCD_Dis_Column || Ystart > sLCD_DIS.LCD_Dis_Page) {
        //DEBUG("GUI_DisString_EN Input exceeds the normal display range\r\n");
        return;
    }

    while(* pString != '\0') {
        //if X direction filled , reposition to(Xstart,Ypoint),Ypoint is Y direction plus the height of the character
        if((Xpoint + Font->Width ) > sLCD_DIS.LCD_Dis_Column ) {
            Xpoint = Xstart;
            Ypoint += Font->Height;
        }

        // If the Y direction is full, reposition to(Xstart, Ystart)
        if((Ypoint  + Font->Height ) > sLCD_DIS.LCD_Dis_Page ) {
            Xpoint = Xstart;
            Ypoint = Ystart;
        }
        GUI_DisChar(Xpoint, Ypoint, * pString, Font, Color_Background, Color_Foreground);

        //The next character of the address
        pString ++;

        //The next word of the abscissa increases the font of the broadband
        Xpoint += Font->Width;
    }
}

/******************************************************************************
function:	Display the string
parameter:
	Xstart           ：X coordinate
	Ystart           : Y coordinate
	Nummber          : The number displayed
	Font             ：A structure pointer that displays a character size
	Color_Background : Select the background color of the English character
	Color_Foreground : Select the foreground color of the English character
******************************************************************************/
#define  ARRAY_LEN 255
void GUI_DisNum(POINT Xpoint, POINT Ypoint, int32_t Nummber,
                sFONT* Font, COLOR Color_Background, COLOR Color_Foreground )
{

    int16_t Num_Bit = 0, Str_Bit = 0;
    uint8_t Str_Array[ARRAY_LEN] = {0}, Num_Array[ARRAY_LEN] = {0};
    uint8_t *pStr = Str_Array;

    if(Xpoint > sLCD_DIS.LCD_Dis_Column || Ypoint > sLCD_DIS.LCD_Dis_Page) {
        //DEBUG("GUI_DisNum Input exceeds the normal display range\r\n");
        return;
    }

    //Converts a number to a string
    while(Nummber) {
        Num_Array[Num_Bit] = Nummber % 10 + '0';
        Num_Bit++;
        Nummber /= 10;
    }

    //The string is inverted
    while(Num_Bit > 0) {
        Str_Array[Str_Bit] = Num_Array[Num_Bit - 1];
        Str_Bit ++;
        Num_Bit --;
    }

    //show
    GUI_DisString_EN(Xpoint, Ypoint, (const char*)pStr, Font, Color_Background, Color_Foreground );
}



/******************************************************************************
function:	Display the bit map,1 byte = 8bit = 8 points
parameter:
	Xpoint ：X coordinate
	Ypoint : Y coordinate
	pMap   : Pointing to the picture
	Width  ：Bitmap Width
	Height : Bitmap Height
note:
	This function is suitable for bitmap, because a 16-bit data accounted for 16 points
******************************************************************************/
void GUI_Disbitmap(POINT Xpoint, POINT Ypoint, const unsigned char *pMap,
                   POINT Width, POINT Height)
{
    POINT i, j, byteWidth = (Width + 7) / 8;
    for(j = 0; j < Height; j++) {
        for(i = 0; i < Width; i ++) {
            if(*(pMap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
                GUI_DrawPoint(Xpoint + i, Ypoint + j, BLACK, DOT_PIXEL_DFT, DOT_STYLE_DFT);
            }
        }
    }
    LCD_SetLocalArea(0, 0, 320, 480, framebuffer, 320*480*2);
}

/******************************************************************************
function:	Display the Gray map,1 byte = 8bit = 2 points
parameter:
	Xpoint ：X coordinate
	Ypoint : Y coordinate
	pMap   : Pointing to the picture
	Width  ：Bitmap Width
	Height : Bitmap Height
note:
	This function is suitable for bitmap, because a 4-bit data accounted for 1 points
	Please use the Image2lcd generated array
******************************************************************************/
void GUI_DisGrayMap(POINT Xpoint, POINT Ypoint, const unsigned char *pBmp)
{
    //Get the Map header Gray, width, height
    char Gray;
    Gray = *(pBmp + 1);
    POINT Height, Width;
    Width = (*(pBmp + 3) << 8) | (*(pBmp + 2));
    Height = (*(pBmp + 5) << 8) | (*(pBmp + 4));

    POINT i, j;
    if(Gray == 0x04) { //Sixteen gray levels
        pBmp = pBmp + 6;
        for(j = 0; j < Height; j++)
            for(i = 0; i < Width / 2; i++) {
                GUI_DrawPoint(Xpoint + i * 2, Ypoint + j, ~(*pBmp >> 4), DOT_PIXEL_DFT, DOT_STYLE_DFT);
                GUI_DrawPoint(Xpoint + i * 2 + 1, Ypoint + j, ~*pBmp , DOT_PIXEL_DFT, DOT_STYLE_DFT);
                pBmp++;
            }
    } else {
        //DEBUG("Does not support type\r\n");
        return;
    }
}

sFONT *GUI_GetFontSize(POINT Dx, POINT Dy)
{
    sFONT *Font;
    if (Dx > Font24.Width && Dy > Font24.Height) {
        Font = &Font24;
    } else if ((Dx > Font20.Width && Dx < Font24.Width) &&
               (Dy > Font20.Height && Dy < Font24.Height)) {
        Font = &Font20;
    } else if ((Dx > Font16.Width && Dx < Font20.Width) &&
               (Dy > Font16.Height && Dy < Font20.Height)) {
        Font = &Font16;
    } else if ((Dx > Font12.Width && Dx < Font16.Width) &&
               (Dy > Font12.Height && Dy < Font16.Height)) {
        Font = &Font12;
    } else if ((Dx > Font8.Width && Dx < Font12.Width) &&
               (Dy > Font8.Height && Dy < Font12.Height)) {
        Font = &Font8;
    } else {
        //DEBUG("Please change the display area size, or add a larger font to modify\r\n");
    }
    return Font;
}
/******************************************************************************
  function:	According to the display area adaptive display time
  parameter:
		xStart :   X direction Start coordinates
		Ystart :   Y direction Start coordinates
		Xend   :   X direction end coordinates
		Yend   :   Y direction end coordinates
		pTime  :   Pointer to the definition of the structure
		Color  :   Set show color
  note:
******************************************************************************/
void GUI_Showtime(POINT Xstart, POINT Ystart, POINT Xend, POINT Yend,
                  DEV_TIME *pTime, COLOR Color)
{
    uint8_t value[10] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9'};
    sFONT *Font;

    //According to the display area adaptive font size
    POINT Dx = (Xend - Xstart) / 7;//Determine the spacing between characters
    POINT Dy = Yend - Ystart;      //determine the font size
    Font = GUI_GetFontSize(Dx, Dy);
	
	if ((pTime->Sec % 10) < 10 && (pTime->Sec % 10) > 0) {
		LCD_SetArealColor(Xstart + Dx * 6, Ystart, Xend, Yend, WHITE);// xx:xx:x0
	} else {
		if ((pTime->Sec / 10) < 6 && (pTime->Sec / 10) > 0) {
			LCD_SetArealColor(Xstart + Dx * 5, Ystart, Xend, Yend, WHITE);// xx:xx:00
		} else {//sec = 60
			pTime->Min = pTime->Min + 1;
			pTime->Sec = 0;
			if ((pTime->Min % 10) < 10 && (pTime->Min % 10) > 0) {
				LCD_SetArealColor(Xstart + Dx * 3 + Dx / 2, Ystart, Xend, Yend, WHITE);// xx:x0:00
			} else {
				if ((pTime->Min / 10) < 6 && (pTime->Min / 10) > 0) {
					LCD_SetArealColor(Xstart + Dx * 2 + Dx / 2, Ystart, Xend, Yend, WHITE);// xx:00:00
				} else {//min = 60
					pTime->Hour =  pTime->Hour + 1;
					pTime->Min = 0;
					if ((pTime->Hour % 10) < 4 && (pTime->Hour % 10) > 0 && pTime->Hour < 24) {// x0:00:00
						LCD_SetArealColor(Xstart + Dx, Ystart, Xend, Yend, WHITE);
					} else {
						pTime->Hour = 0;
						pTime->Min = 0;
						pTime->Sec = 0;
						LCD_SetArealColor(Xstart, Ystart, Xend, Yend, WHITE);// 00:00:00
					}
				}
			}
		}
	}
    
    //Write data into the cache
    GUI_DisChar(Xstart                           , Ystart, value[pTime->Hour / 10], Font, FONT_BACKGROUND, Color);
    GUI_DisChar(Xstart + Dx                      , Ystart, value[pTime->Hour % 10], Font, FONT_BACKGROUND, Color);
    GUI_DisChar(Xstart + Dx  + Dx / 4 + Dx / 2   , Ystart, ':'                    , Font, FONT_BACKGROUND, Color);
    GUI_DisChar(Xstart + Dx * 2 + Dx / 2         , Ystart, value[pTime->Min / 10] , Font, FONT_BACKGROUND, Color);
    GUI_DisChar(Xstart + Dx * 3 + Dx / 2         , Ystart, value[pTime->Min % 10] , Font, FONT_BACKGROUND, Color);
    GUI_DisChar(Xstart + Dx * 4 + Dx / 2 - Dx / 4, Ystart, ':'                    , Font, FONT_BACKGROUND, Color);
    GUI_DisChar(Xstart + Dx * 5                  , Ystart, value[pTime->Sec / 10] , Font, FONT_BACKGROUND, Color);
    GUI_DisChar(Xstart + Dx * 6                  , Ystart, value[pTime->Sec % 10] , Font, FONT_BACKGROUND, Color);
}


/******************************************************************************
function:	GUI_Show
note:
	Clear,
	Draw Line,
	Draw Rectangle,
	Draw Rings,
	Draw Olympic Rings,
	Display String,
	Show Pic
******************************************************************************/

// 测量工具界面函数
void GUI_Show(void)
{
    printf("LCD_Dis_Column = %d\r\n", sLCD_DIS.LCD_Dis_Column);
    printf("LCD_Dis_Page = %d\r\n", sLCD_DIS.LCD_Dis_Page);

    int cls = -1;
    float obj_width = 0.0f;
    float obj_distance = 0.0f;
    int has_measure = (load_measurement_data(&cls, &obj_distance, &obj_width) == 0);
    printf(cls, obj_distance, obj_width);

    char line_cls[32] = {0};
    char line_width[48] = {0};
    char line_distance[48] = {0};

    if (has_measure) {
        snprintf(line_cls, sizeof(line_cls), "Class: %d", cls);
        snprintf(line_distance, sizeof(line_distance), "%.1f cm", obj_distance);
        snprintf(line_width, sizeof(line_width), "%.1f cm", obj_width);
    } else {
        strncpy(line_cls, "Class: N/A", sizeof(line_cls) - 1);
        strncpy(line_width, "N/A", sizeof(line_width) - 1);
        strncpy(line_distance, "N/A", sizeof(line_distance) - 1);
    }

    // 清屏为白色背景
    GUI_Clear(WHITE);

    // 绘制标题栏
    GUI_DrawRectangle(0, 0, 320, 35, PURPLE, DRAW_FULL, DOT_PIXEL_1X1);
    GUI_DisString_EN(30, 6, "Measurement Tool", &Font24, PURPLE, WHITE);

    // 绘制图形显示区域
    GUI_DrawRectangle(20, 50, 300, 250, GRAY, DRAW_EMPTY, DOT_PIXEL_1X1);

    // 绘制测试图形 (70x99像素)
    POINT graphicX = 125;  // 居中位置: (320-70)/2 = 125
    POINT graphicY = 100;  // (250-50-99)/2 + 50 ≈ 100
    switch (cls) {
        case 1: // 例: 三角形
        GUI_Disbitmap(graphicX, graphicY, icon_1_bitmap, 70, 99);
            break;
        case 2: // 例: 方形
            GUI_Disbitmap(graphicX, graphicY, icon_2_bitmap, 70, 99);
            break;
        case 0: // 例: 圆形
            GUI_Disbitmap(graphicX, graphicY, icon_3_bitmap, 70, 99);
            break;
        default:
            // 未知类别，绘制空白框
            GUI_DrawRectangle(graphicX, graphicY, graphicX + 70, graphicY + 99, BLUE, DRAW_EMPTY, DOT_PIXEL_1X1);
            break;
    }
    GUI_DrawRectangle(graphicX, graphicY, graphicX + 70, graphicY + 99, BLUE, DRAW_EMPTY, DOT_PIXEL_1X1);

    // 图形标签
    // GUI_DisString_EN(graphicX + 15, graphicY + 40, "70x99", &Font16, BLUE, WHITE);
    GUI_DisString_EN(80, 260, "Object Type", &Font20, WHITE, BLACK);

    // 摄像头图标 (简单绘制)
    // GUI_DrawCircle(285, 65, 8, BLACK, DRAW_EMPTY, DOT_PIXEL_1X1);
    // GUI_DrawRectangle(280, 55, 290, 60, BLACK, DRAW_FULL, DOT_PIXEL_1X1);

    // 测量数据显示区域
    GUI_DrawRectangle(0, 290, 320, 480, LIGHTBLUE, DRAW_FULL, DOT_PIXEL_1X1);

    GUI_DisString_EN(30, 310, "Distance to Camera", &Font20, LIGHTBLUE, BLACK);
    GUI_DisString_EN(95, 340, line_distance, &Font20, LIGHTBLUE, BLACK);
    GUI_DisString_EN(70, 380, "Object width", &Font20, LIGHTBLUE, BLACK);
    GUI_DisString_EN(95, 410, line_width, &Font20, LIGHTBLUE, BLACK);
    // 状态指示器
    // GUI_DisString_EN(30, 360, "System Ready", &Font12, WHITE, BLACK);
    // GUI_DrawCircle(20, 365, 3, GREEN, DRAW_FULL, DOT_PIXEL_1X1);

    // 测量按钮
    // GUI_DrawRectangle(80, 390, 240, 430, PURPLE, DRAW_FULL, DOT_PIXEL_1X1);
    // GUI_DisString_EN(103, 400, "MEASURE", &Font24, PURPLE, WHITE);

    // 更新显示
    LCD_SetLocalArea(0, 0, 320, 480, framebuffer, 320*480*2);
}

// 开始等待界面函数

void GUI_Waiting(void)
{
    // 清屏为白色背景
    GUI_Clear(WHITE);

    // 绘制标题栏
    GUI_DrawRectangle(0, 0, 320, 35, PURPLE, DRAW_FULL, DOT_PIXEL_1X1);
    GUI_DisString_EN(30, 6, "Measurement Tool", &Font24, PURPLE, WHITE);

    // 显示等待信息
    GUI_DisString_EN(80, 200, "System Initializing...", &Font20, WHITE, BLACK);
    GUI_DisString_EN(100, 240, "Please Wait", &Font20, WHITE, BLACK);

    // 更新显示
    LCD_SetLocalArea(0, 0, 320, 480, framebuffer, 320*480*2);
}



