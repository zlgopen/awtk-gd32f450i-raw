/*********************************************************************************************************
*
* File                : TouchPanel.c
* Hardware Environment: 
* Build Environment   : RealView MDK-ARM  Version: 4.20
* Version             : V1.0
* By                  : 
*
*                                  (c) Copyright 2005-2011, WaveShare
*                                       http://www.waveshare.net
*                                          All Rights Reserved
*
*********************************************************************************************************/

/* Includes ------------------------------------------------------------------*/
#include "TouchPanel.h"
#include "gd32f450i_eval.h"
#include "math.h"
#include <stdlib.h>
#include "gd32f4xx.h"
/* Private variables ---------------------------------------------------------*/
Matrix matrix;
val_XY g_tTP;

Coordinate  display;
__IO uint8_t Line_Flag=0,line_cnt=0;

Coordinate ScreenSample[5];

Coordinate DisplaySample[5] = {
                                 { LCD_PIXEL_WIDTH*0.1,LCD_PIXEL_HEIGHT*0.1},
                                 { LCD_PIXEL_WIDTH*0.9,LCD_PIXEL_HEIGHT*0.1},
                                 { LCD_PIXEL_WIDTH*0.1,LCD_PIXEL_HEIGHT*0.9},
                                 { LCD_PIXEL_WIDTH*0.9,LCD_PIXEL_HEIGHT*0.9},
                                 { LCD_PIXEL_WIDTH*0.5,LCD_PIXEL_HEIGHT*0.5}

                              };

/* Private define ------------------------------------------------------------*/
#define THRESHOLD 8

#define XPT2046_READ_TIMES 5        /* Number of reads */
#define XPT2046_LOST_VAL 1          /* Value to be lost */
uint8_t ADC_ERR_RANGE = 6;          /* Error range of ADC value */                 

/*******************************************************************************
* Function Name  : delay_ms
* Description    : Delay Time
* Input          : - cnt: Delay Time
* Output         : None
* Return         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
//void delay_ms(uint16_t ms)    
//{ 
//    uint16_t i,j; 
//    for( i = 0; i < ms; i++ )
//    {
//        for( j = 0; j < 0xffff; j++ );
//    }
//} 

/* Private functions ---------------------------------------------------------*/
/**
  * @brief  Touch screen CS.
  * @param  a.
  * @retval None
  */
void SPI_CS(uint8_t a)
{
    if (a)
        gpio_bit_set(Open_TP_CS_PORT, Open_TP_CS_PIN);
    else
        gpio_bit_reset(Open_TP_CS_PORT, Open_TP_CS_PIN);
}

/**
  * @brief  Set or Clear the SPI_MOSI.
  * @param  a
  * @retval None.
  */
void SPI_DIN(uint8_t a)
{
    if (a)
        gpio_bit_set(Open_SPI_MOSI_GPIO_PORT, Open_SPI_MOSI_PIN);
    else
        gpio_bit_reset(Open_SPI_MOSI_GPIO_PORT, Open_SPI_MOSI_PIN);
}

/**
  * @brief  Set the touch screen clock by changing SPI_SCK value.
  * @param  a: 0 or 1
  * @retval None
  */
void SPI_CLK(uint8_t a)
{
    if (a)
        gpio_bit_set(Open_SPI_SCK_GPIO_PORT, Open_SPI_SCK_PIN);
    else
        gpio_bit_reset(Open_SPI_SCK_GPIO_PORT, Open_SPI_SCK_PIN);
}

/**
  * @brief  Read the data SPI received( PA6, SPI1_MISO).
  * @param  None.
  * @retval PA6 value.
  */
uint8_t SPI_DOUT(void)
{
    return gpio_input_bit_get(Open_SPI_MISO_GPIO_PORT, Open_SPI_MISO_PIN);
}

/**
  * @brief  SPI delay function
  * @param  i
  * @retval None
  */
void SPI_delay(uint16_t i)
{
    uint16_t k;
    for (k=0;k<i;k++);
}

/**
  * @brief  Touch start
  * @param  None
  * @retval None
  */
void Touch_Start(void)
{
    SPI_CLK(0);
    SPI_CS(1);
    SPI_DIN(1);
    SPI_CLK(1);
    SPI_CS(0);
}

/**
  * @brief  Write data to Touch screen
  * @param  d: the data to be writed
  * @retval None
  */
void Touch_Write(uint8_t d)
{
    uint8_t buf, i ;
    SPI_CLK(0);
    for( i = 0; i < 8; i++ )
    {
        buf = (d >> (7-i)) & 0x1 ;
        SPI_DIN(buf);
        SPI_CLK(0);
        SPI_CLK(1);
        SPI_CLK(0);
    }
}

/**
  * @brief  Read the touch AD value by SPI.
  * @param  None
  * @retval Touch AD value
  */
uint16_t  Touch_Read(void)
{
    uint16_t buf ;
    uint8_t i ;
    
    buf=0;
    for( i = 0; i < 12; i++ )
    {
        buf = buf << 1 ;
        SPI_CLK(1);
        SPI_CLK(0);
        if ( SPI_DOUT() )
        {
            buf = buf + 1 ;
        }
    }
    return( buf );
}

/**
  * @brief  Touch_PenIRQ
  * @param  None
  * @retval The input port pin(PE5) value. If touch pen active, PE5 will be Low.
  */
uint8_t  Touch_PenIRQ(void)
{
    return gpio_input_bit_get(Open_TP_IRQ_PORT, Open_TP_IRQ_PIN);
}

/**
  * @brief  Config LCD touch interrupt request pin (PE5).
  * @param  None
  * @retval None
  */
void Touch_PenIRQ_Initializtion(void)
{
    rcu_periph_clock_enable(Open_TP_IRQ_CLK);
    gpio_mode_set(Open_TP_IRQ_PORT, GPIO_MODE_INPUT, GPIO_PUPD_NONE, Open_TP_IRQ_PIN);

}

/**
  * @brief  Get the physical touch location at X coordinate  
  * @param  None
  * @retval Channel X+ AD sample value
  */
uint16_t Touch_GetPhyX(void)
{
    if (Touch_PenIRQ()) return 0;
    Touch_Start();
    Touch_Write(0x00);
    Touch_Write(CHX);
    return (Touch_Read());
}

/**
  * @brief  Get the physical touch location at Y coordinate
  * @param  None
  * @retval Channel Y+ AD sample value
  */
uint16_t Touch_GetPhyY(void)
{
    if (Touch_PenIRQ()) return 0;
    Touch_Start();
    Touch_Write(0x00);
    Touch_Write(CHY);
    return (Touch_Read());
}

/**
  * @brief  Get channel X+ AD average sample value.
  * @param  None
  * @retval Channel X+ AD average sample value
  */
uint16_t Touch_MeasurementX(void)
{
    uint8_t i;
    uint16_t p=0;
    for (i=0;i<8;i++)
    {
        p+=Touch_GetPhyX();
        SPI_delay(1000);
    }
    p>>=3;
    
    return ( p );
}

/**
  * @brief  Get channel Y+ AD average sample value.
  * @param  None
  * @retval Channel Y+ AD average sample value
  */
uint16_t Touch_MeasurementY(void)
{
    uint8_t i;
    uint16_t p=0;
    for (i=0;i<8;i++)
    {
        p+=Touch_GetPhyY();
        SPI_delay(1000);
    }
    p>>=3;

    return ( p );
}

/**
  * @brief  Get X coordinate value of touch point on LCD.
  * @param  adx : channel X+ AD average sample value.
  * @retval X coordinate value of touch point.
  */
uint16_t _AD2X(uint16_t adx)
{
    uint16_t sx = 0;
    uint32_t
    r = adx - AD_Left;
    r *= LCD_X - 1;
    sx =  r / (AD_Right - AD_Left);
    if (sx <= 0 || sx > LCD_X)
        return 0;
    return sx;
}

/**
  * @brief  AD Converer
  * @param  ady 
  * @retval Y coordinate value of touch point.
  */
uint16_t _AD2Y(uint16_t ady) 
{
    uint16_t sy = 0;
    uint32_t
    r = ady - AD_Top;
    r *= LCD_Y - 1;
    sy =  r / (AD_Bottom - AD_Top);
    if (sy <= 0 || sy > LCD_Y)
        return 0;
    return sy;  
}

/**
  * @brief  Read a value (x or y) for several times. Order these values, 
  *         remove the lowest and highest and obtain the average value
  * @param  _ucCh, CHX or CHY 
  * @retval The data to be read.
  */
uint16_t TOUCH_DataFilter(uint8_t _ucCh)
{
    uint16_t i, j; 
    uint16_t buf[XPT2046_READ_TIMES]; 
    uint16_t usSum; 
    uint16_t usTemp;
    /* Read data in XPT2046_READ_TIMES times */
    for(i=0; i < XPT2046_READ_TIMES; i++)
    {
        if (_ucCh == CHX)
            buf[i] = Touch_GetPhyX();
        else if (_ucCh == CHY)
            buf[i] = Touch_GetPhyY();
    }
    /* Sort in ascending sequence */
    for(i = 0; i < XPT2046_READ_TIMES - 1; i++)
    {
        for(j = i + 1; j < XPT2046_READ_TIMES; j++)
        {
            if(buf[i] > buf[j])
            {
                usTemp = buf[i]; 
                buf[i] = buf[j]; 
                buf[j] = usTemp;
            }
        }
    }
    usSum = 0;
    for(i = XPT2046_LOST_VAL; i < XPT2046_READ_TIMES - XPT2046_LOST_VAL; i++)
    {
        usSum += buf[i];
    }
    usTemp = usSum / (XPT2046_READ_TIMES - 2 * XPT2046_LOST_VAL);
    return usTemp;
}

/**
  * @brief  Continuous read x and y value two times. If the deviation of two values 
  *         is less than ADC_ERR_RANGE, the data is correct, otherwise the data is wrong.
  * @param  *_usX, *_usY, is the X,Y value be read 
  * @retval 0, fail; 1, success
  */
uint8_t TOUCH_ReadAdcXY(int16_t *_usX, int16_t *_usY)
{
    uint16_t iX1, iY1; 
    uint16_t iX2, iY2; 
    uint16_t iX, iY;
    iX1 = TOUCH_DataFilter(CHX); 
    iY1 = TOUCH_DataFilter(CHY); 
    iX2 = TOUCH_DataFilter(CHX); 
    iY2 = TOUCH_DataFilter(CHY);
    
    iX = abs(iX1 - iX2); 
    iY = abs(iY1 - iY2);
    if ((iX <= ADC_ERR_RANGE) && (iY <= ADC_ERR_RANGE))
    {
        *_usX = (iX1 + iX2) / 2; 
        *_usY = (iY1 + iY2) / 2; 
        return 1;
    }
    else
        return 0;
}

/**
  * @brief  The detection program of touch event
  * @param  void 
  * @retval void
  */
void TOUCH_SCAN(void)
{
    uint8_t s_invalid_count = 0;
    if (Touch_PenIRQ()== 0)
    {
        while(!TOUCH_ReadAdcXY(&g_tTP.usAdcNowX, &g_tTP.usAdcNowY)&&s_invalid_count < 20)  
        { 
            s_invalid_count++; 
        }
        if(s_invalid_count >= 20) 
        { 
            g_tTP.usAdcNowX = -1; 
            g_tTP.usAdcNowY = -1;
        }
    }
    else
    { 
        g_tTP.usAdcNowX = -1; 
        g_tTP.usAdcNowY = -1; 
    }
}

/*******************************************************************************
* Function Name  : TP_Init
* Description    : 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
void TP_Init(void) 
{
    rcu_periph_clock_enable(Open_SPI_SCK_GPIO_CLK);
    rcu_periph_clock_enable(Open_SPI_MISO_GPIO_CLK);
    rcu_periph_clock_enable(Open_SPI_MOSI_GPIO_CLK);
    rcu_periph_clock_enable(Open_TP_CS_CLK); 
    rcu_periph_clock_enable(Open_TP_IRQ_CLK);
    rcu_periph_clock_enable(Open_RCC_SPI);

    gpio_mode_set(Open_SPI_SCK_GPIO_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,Open_SPI_SCK_PIN|Open_SPI_MOSI_PIN);
    gpio_output_options_set(Open_SPI_SCK_GPIO_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,Open_SPI_SCK_PIN|Open_SPI_MOSI_PIN);
    
    gpio_mode_set(Open_SPI_MISO_GPIO_PORT,GPIO_MODE_INPUT,GPIO_PUPD_NONE,Open_SPI_MISO_PIN);

    /* TP_CS (SPI_NSS) */
    gpio_mode_set(Open_TP_CS_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,Open_TP_CS_PIN);
    gpio_output_options_set(Open_TP_CS_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,Open_TP_CS_PIN);

    /* TP_IRQ (LCD_Touch_PENIRQ) */
    gpio_mode_set(Open_TP_IRQ_PORT,GPIO_MODE_INPUT,GPIO_PUPD_NONE,Open_TP_IRQ_PIN);

    TP_CS(1); 
} 


void TP_IRQ_EXTI_Config(void)
{
    rcu_periph_clock_enable(Open_TP_IRQ_CLK);
    gpio_mode_set(Open_TP_IRQ_PORT,GPIO_MODE_INPUT,GPIO_PUPD_NONE,Open_TP_IRQ_PIN);

    exti_init(TP_IRQ_EXTI_LINE, EXTI_INTERRUPT, EXTI_TRIG_FALLING);
    exti_interrupt_enable(TP_IRQ_EXTI_LINE);

    nvic_irq_enable(TP_IRQ_EXTI_IRQ, 0, 1);
}


/*******************************************************************************
* Function Name  : DelayUS
* Description    : 
* Input          : - cnt:
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/

static void DelayUS(uint32_t cnt)
{
  uint16_t i;
  for(i = 0;i<cnt;i++)
  {
     uint8_t us = 10;
     while (us--)
     {
       ;   
     }
  }
}

/*******************************************************************************
* Function Name  : Read_X
* Description    : Read ADS7843 ADC X 
* Input          : None
* Output         : None
* Return         : 
* Attention		 : None
*******************************************************************************/
int Read_X(void)  
{  
  int i; 
//  TP_CS(0); 
  Touch_Start();
//  DelayUS(1); 
  Touch_Write(0x00);
  Touch_Write(CHX); 
//  DelayUS(1); 
  i=Touch_Read(); 
  TP_CS(1); 
  return i;     
} 

/*******************************************************************************
* Function Name  : Read_Y
* Description    : Read ADS7843 ADC Y
* Input          : None
* Output         : None
* Return         : 
* Attention		 : None
*******************************************************************************/
int Read_Y(void)  
{  
  int i; 
  Touch_Start();
  Touch_Write(0x00);
  Touch_Write(CHY); 
//  DelayUS(1); 
  i=Touch_Read(); 
  TP_CS(1); 
  return i;   
} 


/*******************************************************************************
* Function Name  : TP_GetAdXY
* Description    : Read ADS7843
* Input          : None
* Output         : None
* Return         : 
* Attention		 : None
*******************************************************************************/
void TP_GetAdXY(int *x,int *y)  
{ 
  int adx,ady; 
  adx=Read_X(); 
  DelayUS(1); 
  ady=Read_Y(); 
  *x=adx; 
  *y=ady; ; 
} 

/*******************************************************************************
* Function Name  : TP_DrawPoint
* Description    : 
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
//void TP_DrawPoint(uint16_t Xpos,uint16_t Ypos)
//{
//    lcd_point_set(Xpos,Ypos,LCD_COLOR_BLUE);
//    lcd_point_set(Xpos+1,Ypos,LCD_COLOR_BLUE);
//    lcd_point_set(Xpos,Ypos+1,LCD_COLOR_BLUE);
//    lcd_point_set(Xpos+1,Ypos+1,LCD_COLOR_BLUE);
//}

/*******************************************************************************
* Function Name  : DrawCross
* Description    : 
* Input          : - Xpos: Row Coordinate
*                  - Ypos: Line Coordinate 
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
//void DrawCross(uint16_t Xpos,uint16_t Ypos)
//{
//    lcd_line_draw(Xpos-13, Ypos, 10, LCD_LINEDIR_HORIZONTAL);
//    lcd_line_draw(Xpos+4, Ypos, 10, LCD_LINEDIR_HORIZONTAL);
//    lcd_line_draw(Xpos, Ypos-13, 10, LCD_LINEDIR_VERTICAL);
//    lcd_line_draw(Xpos, Ypos+4, 10, LCD_LINEDIR_VERTICAL);
//}

/*******************************************************************************
* Function Name  : Read_Ads7846
* Description    : Get TouchPanel X Y
* Input          : None
* Output         : None
* Return         : Coordinate *
* Attention		 : None
*******************************************************************************/
Coordinate *Read_Ads7846(void)
{
  static Coordinate  screen;    //,screen_p
  int m0,m1,m2,temp[3];
	int TP_X[1],TP_Y[1];
  uint8_t count=0,i,j;
  int buffer[2][32]={{0},{0}},data=0;  

//   if((TP_INT_IN)) 
// 		{line_cnt=0;Line_Flag=0;}	
//   if(!TP_INT_IN )  //&& (!Line_Flag)
// 		DelayUS(100);
// 	else
// 		return 0; 

	
  do
  {		   
		TP_GetAdXY(TP_X,TP_Y);  
		buffer[0][count]=TP_X[0];  
		buffer[1][count]=TP_Y[0];
		count++;  
		//DelayUS(1);
  }
  while(!TP_INT_IN&& count<16);  /* TP_INT_IN  */
  if(count==16)   /* X Y  */ 
  {

		for(i=0;i<16;i++)
		{ 
			for(j=0;j<16-i;j++)
			{
				if(buffer[0][j]>buffer[0][j+1]) ///////X
				{
					data=buffer[0][j];
					buffer[0][j]=buffer[0][j+1];
					buffer[0][j+1]=data;
				}
				if(buffer[1][j]>buffer[1][j+1])   //Y
				{
					data=buffer[1][j];
					buffer[1][j]=buffer[1][j+1];
					buffer[1][j+1]=data;
				}
			}

 		}
		/* Average X  */
    temp[0]=(buffer[0][4]+buffer[0][5]+buffer[0][6])/3;
	  temp[1]=(buffer[0][7]+buffer[0][8]+buffer[0][9])/3;
	  temp[2]=(buffer[0][10]+buffer[0][11]+buffer[0][12])/3;	
  	m0=temp[0]-temp[1];
	  m1=temp[1]-temp[2];
	  m2=temp[2]-temp[0];

	  m0=m0>0?m0:(-m0);
    m1=m1>0?m1:(-m1);
	  m2=m2>0?m2:(-m2);
  	if( m0>THRESHOLD  &&  m1>THRESHOLD  &&  m2>THRESHOLD ) return 0;		
	  /* Average Y  */
    temp[0]=(buffer[1][4]+buffer[1][5]+buffer[1][6])/3;
	  temp[1]=(buffer[1][7]+buffer[1][8]+buffer[1][9])/3;
	  temp[2]=(buffer[1][10]+buffer[1][11]+buffer[1][12])/3;
	  m0=temp[0]-temp[1];
	  m1=temp[1]-temp[2];
	  m2=temp[2]-temp[0];
	  m0=m0>0?m0:(-m0);
	  m1=m1>0?m1:(-m1);
	  m2=m2>0?m2:(-m2);
	  if(m0>THRESHOLD && m1>THRESHOLD && m2>THRESHOLD) return 0;

	
		screen.x=(buffer[0][4]+buffer[0][5]+buffer[0][6]+buffer[0][7]+buffer[0][8]+buffer[0][9]+buffer[0][10]+buffer[0][11])/8;
		screen.y=(buffer[1][4]+buffer[1][5]+buffer[1][6]+buffer[1][7]+buffer[1][8]+buffer[1][9]+buffer[1][10]+buffer[1][11])/8;		
		
// 		screen.x=(buffer[0][12]+buffer[0][13]+buffer[0][14]+buffer[0][15]+buffer[0][16]+buffer[0][17]+buffer[0][18]+buffer[0][19])/8;
// 		screen.y=(buffer[1][12]+buffer[1][13]+buffer[1][14]+buffer[1][15]+buffer[1][16]+buffer[1][17]+buffer[1][18]+buffer[1][19])/8;	

		return &screen;		
  }  
  return 0; 
}
	 


Coordinate *Read_Ads7846_Cal(void)
{
	static Coordinate  screen;
	int TP_X[1],TP_Y[1];
  uint8_t count=0,i,j;
  int buffer[2][32]={{0},{0}},data=0;  


  do
  {		   
		TP_GetAdXY(TP_X,TP_Y);  
		buffer[0][count]=TP_X[0];  
		buffer[1][count]=TP_Y[0];
		count++;  
		DelayUS(10);
  }
  while(!TP_INT_IN&& count<32);  /* TP_INT_IN  */
  if(count==32)   /* X Y  */ 
  {

		for(i=0;i<32;i++)
		{ 
			for(j=0;j<32-i;j++)
			{
				if(buffer[0][j]>buffer[0][j+1]) ///////X
				{
					data=buffer[0][j];
					buffer[0][j]=buffer[0][j+1];
					buffer[0][j+1]=data;
				}
				if(buffer[1][j]>buffer[1][j+1])   //Y
				{
					data=buffer[1][j];
					buffer[1][j]=buffer[1][j+1];
					buffer[1][j+1]=data;
				}
			}

		}
// 		screen.x=(buffer[0][4]+buffer[0][5]+buffer[0][6]+buffer[0][7]+buffer[0][8]+buffer[0][9]+buffer[0][10]+buffer[0][11])/8;
// 		screen.y=(buffer[1][4]+buffer[1][5]+buffer[1][6]+buffer[1][7]+buffer[1][8]+buffer[1][9]+buffer[1][10]+buffer[1][11])/8;		
		
		screen.x=(buffer[0][12]+buffer[0][13]+buffer[0][14]+buffer[0][15]+buffer[0][16]+buffer[0][17]+buffer[0][18]+buffer[0][19])/8;
		screen.y=(buffer[1][12]+buffer[1][13]+buffer[1][14]+buffer[1][15]+buffer[1][16]+buffer[1][17]+buffer[1][18]+buffer[1][19])/8;	


		return &screen;		
  }  
  return 0; 
}
/*******************************************************************************
* Function Name  : setCalibrationMatrix
* Description    : Calculate K A B C D E F
* Input          : None
* Output         : None
* Return         : 
* Attention		 : None
*******************************************************************************/
ControlStatus setCalibrationMatrix( Coordinate * displayPtr,
                          Coordinate * screenPtr,
                          Matrix * matrixPtr)
{
	
	ControlStatus retTHRESHOLD = ENABLE ;
  long double a,b,c,d,e,f,g,h,i,j,k,l;
	a=screenPtr[0].x + screenPtr[1].x + screenPtr[2].x + screenPtr[3].x + screenPtr[4].x;
	b=screenPtr[0].y + screenPtr[1].y + screenPtr[2].y + screenPtr[3].y + screenPtr[4].y;
	c=5;
	d=displayPtr[0].x + displayPtr[1].x + displayPtr[2].x + displayPtr[3].x + displayPtr[4].x;
	 
	e=screenPtr[0].x*screenPtr[0].x + screenPtr[1].x*screenPtr[1].x + screenPtr[2].x*screenPtr[2].x + screenPtr[3].x*screenPtr[3].x + screenPtr[4].x*screenPtr[4].x;
	f=screenPtr[0].x*screenPtr[0].y + screenPtr[1].x*screenPtr[1].y + screenPtr[2].x*screenPtr[2].y + screenPtr[3].x*screenPtr[3].y + screenPtr[4].x*screenPtr[4].y;
	g=screenPtr[0].x + screenPtr[1].x + screenPtr[2].x + screenPtr[3].x + screenPtr[4].x;
  h=displayPtr[0].x*screenPtr[0].x + displayPtr[1].x*screenPtr[1].x + displayPtr[2].x*screenPtr[2].x + displayPtr[3].x*screenPtr[3].x + displayPtr[4].x*screenPtr[4].x;
	
	i=screenPtr[0].x*screenPtr[0].y + screenPtr[1].x*screenPtr[1].y + screenPtr[2].x*screenPtr[2].y + screenPtr[3].x*screenPtr[3].y + screenPtr[4].x*screenPtr[4].y;
	j=screenPtr[0].y*screenPtr[0].y + screenPtr[1].y*screenPtr[1].y + screenPtr[2].y*screenPtr[2].y + screenPtr[3].y*screenPtr[3].y + screenPtr[4].y*screenPtr[4].y;
	k=screenPtr[0].y + screenPtr[1].y + screenPtr[2].y + screenPtr[3].y + screenPtr[4].y;
	l=displayPtr[0].x*screenPtr[0].y + displayPtr[1].x*screenPtr[1].y + displayPtr[2].x*screenPtr[2].y + displayPtr[3].x*screenPtr[3].y + displayPtr[4].x*screenPtr[4].y;
	
	matrixPtr->An =(b*g*l - b*h*k - c*f*l + c*h*j + d*f*k - d*g*j)/(a*f*k - a*g*j - b*e*k + b*g*i + c*e*j - c*f*i);
	matrixPtr->Bn =(-a*g*l + a*h*k + c*e*l - c*h*i - d*e*k + d*g*i)/(a*f*k - a*g*j - b*e*k + b*g*i + c*e*j - c*f*i);
	matrixPtr->Cn =(a*f*l - a*h*j - b*e*l + b*h*i + d*e*j - d*f*i)/(a*f*k - a*g*j - b*e*k + b*g*i + c*e*j - c*f*i);
	
	a=screenPtr[0].x + screenPtr[1].x + screenPtr[2].x + screenPtr[3].x + screenPtr[4].x;
	b=screenPtr[0].y + screenPtr[1].y + screenPtr[2].y + screenPtr[3].y + screenPtr[4].y;
	c=5;
	d=displayPtr[0].y + displayPtr[1].y + displayPtr[2].y + displayPtr[3].y + displayPtr[4].y;
	 
	e=screenPtr[0].x*screenPtr[0].x + screenPtr[1].x*screenPtr[1].x + screenPtr[2].x*screenPtr[2].x + screenPtr[3].x*screenPtr[3].x + screenPtr[4].x*screenPtr[4].x;
	f=screenPtr[0].x*screenPtr[0].y + screenPtr[1].x*screenPtr[1].y + screenPtr[2].x*screenPtr[2].y + screenPtr[3].x*screenPtr[3].y + screenPtr[4].x*screenPtr[4].y;
	g=screenPtr[0].x + screenPtr[1].x + screenPtr[2].x + screenPtr[3].x + screenPtr[4].x;
  h=displayPtr[0].y*screenPtr[0].x + displayPtr[1].y*screenPtr[1].x + displayPtr[2].y*screenPtr[2].x + displayPtr[3].y*screenPtr[3].x + displayPtr[4].y*screenPtr[4].x;
	
	i=screenPtr[0].x*screenPtr[0].y + screenPtr[1].x*screenPtr[1].y + screenPtr[2].x*screenPtr[2].y + screenPtr[3].x*screenPtr[3].y + screenPtr[4].x*screenPtr[4].y;
	j=screenPtr[0].y*screenPtr[0].y + screenPtr[1].y*screenPtr[1].y + screenPtr[2].y*screenPtr[2].y + screenPtr[3].y*screenPtr[3].y + screenPtr[4].y*screenPtr[4].y;
	k=screenPtr[0].y + screenPtr[1].y + screenPtr[2].y + screenPtr[3].y + screenPtr[4].y;
	l=displayPtr[0].y*screenPtr[0].y + displayPtr[1].y*screenPtr[1].y + displayPtr[2].y*screenPtr[2].y + displayPtr[3].y*screenPtr[3].y + displayPtr[4].y*screenPtr[4].y;
	
	matrixPtr->Dn =(b*g*l - b*h*k - c*f*l + c*h*j + d*f*k - d*g*j)/(a*f*k - a*g*j - b*e*k + b*g*i + c*e*j - c*f*i);
	matrixPtr->En =(-a*g*l + a*h*k + c*e*l - c*h*i - d*e*k + d*g*i)/(a*f*k - a*g*j - b*e*k + b*g*i + c*e*j - c*f*i);
	matrixPtr->Fn =(a*f*l - a*h*j - b*e*l + b*h*i + d*e*j - d*f*i)/(a*f*k - a*g*j - b*e*k + b*g*i + c*e*j - c*f*i);
	
  return( retTHRESHOLD ) ;
}

/*******************************************************************************
* Function Name  : getDisplayPoint
* Description    : Touch panel X Y to display X Y
* Input          : None
* Output         : None
* Return         : 
* Attention		 : None
*******************************************************************************/
ControlStatus getDisplayPoint(Coordinate * displayPtr,
                     Coordinate * screenPtr,
                     Matrix * matrixPtr )
{
  ControlStatus retTHRESHOLD =ENABLE ;
  /*
	An=168
	*/
  if(!(screenPtr == 0))	
	{		
    /* XD = AX+BY+C */        
    displayPtr->x = ( (matrixPtr->An * screenPtr->x) + 
                      (matrixPtr->Bn * screenPtr->y) + 
                       matrixPtr->Cn 
                    ) ;
	/* YD = DX+EY+F */        
    displayPtr->y = ( (matrixPtr->Dn * screenPtr->x) + 
                      (matrixPtr->En * screenPtr->y) + 
                       matrixPtr->Fn 
                    );
		

  return(retTHRESHOLD);
	}
	else 
		return DISABLE;
} 



/*******************************************************************************
* Function Name  : TouchPanel_Calibrate
* Description    : 
* Input          : None
* Output         : None
* Return         : None
* Attention		 : None
*******************************************************************************/
//void TouchPanel_Calibrate(void)
//{
//    uint8_t i,flag=1;
//    Coordinate * Ptr;
//    __IO uint16_t AB,AC,CD,BD,AD,BC;
//    uint32_t tem1,tem2;
////    float fac; 
//    while(flag==1)
//    {
//        for(i=0;i<5;i++)
//        {
//            lcd_font_set(&Font12x12);
//            lcd_clear(LCD_COLOR_BLACK);
//            lcd_text_color_set(LCD_COLOR_BLACK);
//            lcd_background_color_set(LCD_COLOR_RED);
//            lcd_string_display(LINE(0), (uint8_t*)"Touch crosshair to calibrate");
//            delay_ms(250);
//            DrawCross(DisplaySample[i].x,DisplaySample[i].y);
//            do
//            {
//                if(!TP_INT_IN)
//                {
//                    delay_ms(100);
//                    Ptr=Read_Ads7846_Cal();
//                    while(!TP_INT_IN); 
//                }
//                else 
//                    Ptr = (void*)0;
////              if(!TP_INT_IN)
////              Ptr=Read_Ads7846_Cal();
//            }
//            while( Ptr == (void*)0 );
//            ScreenSample[i].x= Ptr->x; ScreenSample[i].y= Ptr->y;
//        }
//        
//        tem1=(ScreenSample[0].x-ScreenSample[1].x)*(ScreenSample[0].x-ScreenSample[1].x);
//        tem2=(ScreenSample[0].y-ScreenSample[1].y)*(ScreenSample[0].y-ScreenSample[1].y);
//        AB=sqrt(tem1+tem2);

//        tem1=(ScreenSample[0].x-ScreenSample[2].x)*(ScreenSample[0].x-ScreenSample[2].x);
//        tem2=(ScreenSample[0].y-ScreenSample[2].y)*(ScreenSample[0].y-ScreenSample[2].y);
//        AC=sqrt(tem1+tem2);

//        tem1=(ScreenSample[3].x-ScreenSample[1].x)*(ScreenSample[3].x-ScreenSample[1].x);
//        tem2=(ScreenSample[3].y-ScreenSample[1].y)*(ScreenSample[3].y-ScreenSample[1].y);
//        BD=sqrt(tem1+tem2);        

//        tem1=(ScreenSample[2].x-ScreenSample[3].x)*(ScreenSample[2].x-ScreenSample[3].x);
//        tem2=(ScreenSample[2].y-ScreenSample[3].y)*(ScreenSample[2].y-ScreenSample[3].y);
//        CD=sqrt(tem1+tem2);

//        tem1=(ScreenSample[0].x-ScreenSample[3].x)*(ScreenSample[0].x-ScreenSample[3].x);
//        tem2=(ScreenSample[0].y-ScreenSample[3].y)*(ScreenSample[0].y-ScreenSample[3].y);
//        AD=sqrt(tem1+tem2);        

//        tem1=(ScreenSample[2].x-ScreenSample[1].x)*(ScreenSample[2].x-ScreenSample[1].x);
//        tem2=(ScreenSample[2].y-ScreenSample[1].y)*(ScreenSample[2].y-ScreenSample[1].y);
//        BC=sqrt(tem1+tem2);    
//        
////		fac=(float)AD/BC;
////		if(fac<(float)0.95||fac>(float)2.05)
////		continue;		
////		
////		fac=(float)AB/CD;
////		if(fac<(float)0.95||fac>(float)2.05)
////		continue;
////		
////		fac=(float)AC/BD;
////		if(fac<(float)0.95||fac>(float)2.05)
////		continue;
//		
//        flag=0;

//    }
//    setCalibrationMatrix( &DisplaySample[0],&ScreenSample[0],&matrix );
//    lcd_background_color_set(LCD_COLOR_WHITE); 
//    lcd_clear(LCD_COLOR_WHITE);
//} 
//void Xpos_Ypos(uint16_t Xpos, uint16_t Ypos)
//{
//    uint32_t XAD; 
//    lcd_font_set(&Font16x24);
//    //LCD_SetTextColor(LCD_COLOR_RED);
//    lcd_string_display(LINE(1), (uint8_t*)"Xpos:");
//    lcd_string_display(LINE(2), (uint8_t*)"Ypos:");
//    XAD=16*5;
//    /*x??*/
//    lcd_char_display(24,XAD,48+Xpos/1000);             XAD=XAD+16;
//    lcd_char_display(24,XAD,48+Xpos%1000/100);   XAD=XAD+16; 
//    lcd_char_display(24,XAD,48+Xpos%100/10);       XAD=XAD+16;
//    lcd_char_display(24,XAD,48+Xpos%10);             XAD=XAD+16;
//    /*y??*/
//    XAD=16*5;
//    lcd_char_display(48,XAD,48+Ypos/1000);           XAD=XAD+16;
//    lcd_char_display(48,XAD,48+Ypos%1000/100);   XAD=XAD+16;
//    lcd_char_display(48,XAD,48+Ypos%100/10);     XAD=XAD+16;
//    lcd_char_display(48,XAD,48+Ypos%10);         XAD=XAD+16;
//}

/**
  * @brief  Configure touch screen SPI GPIO.
  * @param  None
  * @retval None
  */
void SPI_Touch_GPIO_Configuration(void)
{
    rcu_periph_clock_enable(Open_SPI_SCK_GPIO_CLK);
    rcu_periph_clock_enable(Open_SPI_MISO_GPIO_CLK);
    rcu_periph_clock_enable(Open_SPI_MOSI_GPIO_CLK);
    rcu_periph_clock_enable(Open_TP_CS_CLK); 
    rcu_periph_clock_enable(Open_TP_IRQ_CLK);
    rcu_periph_clock_enable(Open_RCC_SPI);

    gpio_mode_set(Open_SPI_SCK_GPIO_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,Open_SPI_SCK_PIN|Open_SPI_MOSI_PIN);
    gpio_output_options_set(Open_SPI_SCK_GPIO_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,Open_SPI_SCK_PIN|Open_SPI_MOSI_PIN);
    
    gpio_mode_set(Open_SPI_MISO_GPIO_PORT,GPIO_MODE_INPUT,GPIO_PUPD_NONE,Open_SPI_MISO_PIN);

    /* TP_CS (SPI_NSS) */
    gpio_mode_set(Open_TP_CS_PORT,GPIO_MODE_OUTPUT,GPIO_PUPD_NONE,Open_TP_CS_PIN);
    gpio_output_options_set(Open_TP_CS_PORT,GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ,Open_TP_CS_PIN);
    
    TP_CS(1);
}
/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
