/*********************************************************************************************************
*
* File                : TouchPanel.h
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

#ifndef _TOUCHPANEL_H_
#define _TOUCHPANEL_H_

/* Includes ------------------------------------------------------------------*/
#include "gd32f4xx.h"


#define LCD_PIXEL_WIDTH              ((uint16_t)320)
#define LCD_PIXEL_HEIGHT             ((uint16_t)480)

#define Open_TP_CS_PIN                  GPIO_PIN_7
#define Open_TP_CS_PORT                 GPIOF
#define Open_TP_CS_CLK                  RCU_GPIOF

#define Open_TP_IRQ_PIN                 GPIO_PIN_7
#define Open_TP_IRQ_PORT                GPIOD
#define Open_TP_IRQ_CLK                 RCU_GPIOD

#define TP_IRQ_EXTI_LINE                EXTI_7
#define TP_IRQ_EXTI_IRQ                 EXTI5_9_IRQn


#define TP_CS(x)    x ? gpio_bit_set(Open_TP_CS_PORT,Open_TP_CS_PIN): gpio_bit_reset(Open_TP_CS_PORT,Open_TP_CS_PIN)
#define TP_INT_IN   gpio_input_bit_get(Open_TP_IRQ_PORT,Open_TP_IRQ_PIN)

/**
 * @brief Definition for TouchPanel  SPI
 */
 /* Configure TouchPanel pins: */
#define Open_RCC_SPI                    RCU_SPI4

#define Open_SPI_SCK_PIN                GPIO_PIN_8
#define Open_SPI_SCK_GPIO_PORT          GPIOF
#define Open_SPI_SCK_GPIO_CLK           RCU_GPIOF

#define Open_SPI_MISO_PIN               GPIO_PIN_6
#define Open_SPI_MISO_GPIO_PORT         GPIOD
#define Open_SPI_MISO_GPIO_CLK          RCU_GPIOD

#define Open_SPI_MOSI_PIN               GPIO_PIN_9
#define Open_SPI_MOSI_GPIO_PORT         GPIOF
#define Open_SPI_MOSI_GPIO_CLK          RCU_GPIOF


/* Private typedef -----------------------------------------------------------*/
typedef	struct POINT 
{
   uint16_t x;
   uint16_t y;
}Coordinate;


typedef struct Matrix 
{
long double An,  
            Bn,     
            Cn,   
            Dn,    
            En,    
            Fn,
            Divider ;
} Matrix;

typedef struct val_XY
{
    int16_t usAdcNowX;
    int16_t usAdcNowY;
} val_XY;

/* Private variables ---------------------------------------------------------*/
extern Coordinate ScreenSample[5];
extern Coordinate DisplaySample[5];
extern Matrix matrix ;
extern Coordinate  display ;

/* Private define ------------------------------------------------------------*/
#define  AD_Left   300
#define  AD_Right  3850
#define  AD_Top    220
#define  AD_Bottom 3850

#define  LCD_X     480
#define  LCD_Y     272

//#define	CHX 	0x90
//#define	CHY 	0xd0
#define CHX     0xD0
#define CHY     0x90

/* Private function prototypes -----------------------------------------------*/
void TP_Init(void);
Coordinate *Read_Ads7846(void);
//void TouchPanel_Calibrate(void);
void TP_IRQ_EXTI_Config(void) ;
//void DrawCross(uint16_t Xpos,uint16_t Ypos);
//void TP_DrawPoint(uint16_t Xpos,uint16_t Ypos);
ControlStatus setCalibrationMatrix( Coordinate * displayPtr,Coordinate * screenPtr,Matrix * matrixPtr);
ControlStatus getDisplayPoint(Coordinate * displayPtr,Coordinate * screenPtr,Matrix * matrixPtr );
void delay_ms(uint16_t ms);    
//void Xpos_Ypos(uint16_t Xpos, uint16_t Ypos);

void Touch_Write(uint8_t d);
void SPI_Touch_GPIO_Configuration(void);
void Touch_PenIRQ_Initializtion(void);
uint16_t  Touch_GetPhyX(void);
uint16_t  Touch_GetPhyY(void);
uint16_t  Touch_MeasurementX(void);
uint16_t  Touch_MeasurementY(void);
uint8_t  Touch_PenIRQ(void);
uint16_t _AD2X(uint16_t adx);
uint16_t _AD2Y(uint16_t ady);

uint16_t TOUCH_DataFilter(uint8_t _ucCh);
uint8_t TOUCH_ReadAdcXY(int16_t *_usX, int16_t *_usY);
void TOUCH_SCAN(void);
#endif

/*********************************************************************************************************
      END FILE
*********************************************************************************************************/
