/*!
    \file  main.c
    \brief TLI_IPA demo 
*/

/*
    Copyright (C) 2016 GigaDevice

    2016-10-19, V1.0.0, demo for GD32F4xx
*/

#include "FreeRTOS.h"
#include "gd32f4xx.h"
#include <stdio.h>
#include "gd32f450i_eval.h"
#include "exmc_sdram.h"
#include "systick.h"
#include "task.h"

#include "string.h"

uint8_t *online_fb_addr = (uint8_t *)(SDRAM_DEVICE0_ADDR);
uint8_t *offline_fb_addr = (uint8_t *)(SDRAM_DEVICE0_ADDR + (480 * 272 * 2));


static void tli_config(void);
static void tli_gpio_config(void);
static void lcd_config(void);

extern int gui_app_start(int lcd_w, int lcd_h);
static void awtk_task(void *pvParameters) 
{
    memset(online_fb_addr, 0xFF, 480 * 272 * 4);
    gui_app_start(480, 272);
    while (1)
            ;
}

static void led_task1(void *pvParameters) 
{
    gd_eval_led_init(LED1);
    while(1){
        /* turn on LED1 */
        gd_eval_led_on(LED1);
        /* insert 200 ms delay */
        vTaskDelay(250);

        gd_eval_led_off(LED1);
        /* insert 200 ms delay */
        vTaskDelay(250);
    }
}


/*!
    \brief      main program
    \param[in]  none
    \param[out] none
    \retval     none
*/

HeapRegion_t xHeapRegions[] =
{
//	{ ( uint8_t * ) 0x10000000, 0x10000 },
	{ ( uint8_t * ) 0x20010000, 0x20000 },
	{ NULL, 0 }
};



int main(void)
{
    systick_config();
    exmc_synchronous_dynamic_ram_init(EXMC_SDRAM_DEVICE0);
    
    lcd_config();
    tli_config();
    tli_layer_enable(LAYER0);
    tli_reload_config(TLI_FRAME_BLANK_RELOAD_EN);
    tli_enable();
    tli_reload_config(TLI_REQUEST_RELOAD_EN);
    
    
    vPortDefineHeapRegions( xHeapRegions );
    if (xTaskCreate(led_task1, "led_task1", 10, NULL, 3, NULL) != pdPASS)
    {
        while (1)
            ;
    }
    if (xTaskCreate(awtk_task, "awtk_task", 20 * 1024, NULL, 0, NULL) != pdPASS)
    {
        while (1)
            ;
    }
    vTaskStartScheduler();
    while(1){
    }
}

/*!
    \brief      LCD Configuration
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void lcd_config(void)
{
    /* configure the GPIO of TLI */
    tli_gpio_config();
}


/*!
    \brief      configure TLI peripheral
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void tli_config(void)
{
    tli_parameter_struct               tli_init_struct;
    tli_layer_parameter_struct         tli_layer_init_struct;

    rcu_periph_clock_enable(RCU_TLI);
    tli_gpio_config();

#ifdef __CC_ARM
    /* configure the PLLSAI clock to generate lcd clock */
    if(ERROR == rcu_pllsai_config(192, 2, 3, 3)){
        while(1);
    }
#endif /* __CC_ARM */
#ifdef __IAR_SYSTEMS_ICC__
    /* configure the PLLSAI clock to generate lcd clock */
    if(ERROR == rcu_pllsai_config(192, 2, 3, 6)){
        while(1);
    }
#endif /* __IAR_SYSTEMS_ICC__ */
    rcu_tli_clock_div_config(RCU_PLLSAIR_DIV8);
    rcu_osci_on(RCU_PLLSAI_CK);
    if(ERROR == rcu_osci_stab_wait(RCU_PLLSAI_CK)){
        while(1);
    }
    
    /* TLI initialization */
    tli_init_struct.signalpolarity_hs = TLI_HSYN_ACTLIVE_LOW;
    tli_init_struct.signalpolarity_vs = TLI_VSYN_ACTLIVE_LOW;
    tli_init_struct.signalpolarity_de = TLI_DE_ACTLIVE_LOW;
    tli_init_struct.signalpolarity_pixelck = TLI_PIXEL_CLOCK_TLI;
    
    /* LCD display timing configuration */
    tli_init_struct.synpsz_hpsz = 40;
    tli_init_struct.synpsz_vpsz = 9;
    tli_init_struct.backpsz_hbpsz = 42; 
    tli_init_struct.backpsz_vbpsz = 11;  
    tli_init_struct.activesz_hasz = 522;
    tli_init_struct.activesz_vasz = 283;
    tli_init_struct.totalsz_htsz = 524; 
    tli_init_struct.totalsz_vtsz = 285;
    /* LCD background color configure*/
    tli_init_struct.backcolor_red = 0xFF;
    tli_init_struct.backcolor_green = 0xFF;
    tli_init_struct.backcolor_blue = 0xFF; 
    tli_init(&tli_init_struct);

    /* TLI layer1 configuration */
    tli_layer_init_struct.layer_window_leftpos = 43;
    tli_layer_init_struct.layer_window_rightpos = (480 + 43 - 1); 
    tli_layer_init_struct.layer_window_toppos = 12 ;
    tli_layer_init_struct.layer_window_bottompos =(272 + 12 - 1) ;
    tli_layer_init_struct.layer_ppf = LAYER_PPF_RGB565;
    tli_layer_init_struct.layer_sa = 0xFF;
    tli_layer_init_struct.layer_default_blue = 0xFF;
    tli_layer_init_struct.layer_default_green = 0xFF;
    tli_layer_init_struct.layer_default_red = 0xFF;
    tli_layer_init_struct.layer_default_alpha = 0xFF;
    tli_layer_init_struct.layer_acf1 = LAYER_ACF1_PASA;
    tli_layer_init_struct.layer_acf2 = LAYER_ACF2_PASA;
    tli_layer_init_struct.layer_frame_bufaddr = (uint32_t)online_fb_addr;
    tli_layer_init_struct.layer_frame_line_length = ((480 * 2) + 3); 
    tli_layer_init_struct.layer_frame_buf_stride_offset = (480 * 2);
    tli_layer_init_struct.layer_frame_total_line_number = 272; 
    tli_layer_init(LAYER0, &tli_layer_init_struct);
}

/*!
    \brief      configure TLI GPIO  
    \param[in]  none
    \param[out] none
    \retval     none
*/
static void tli_gpio_config(void)
{
    /* GPIO clock enable */
    rcu_periph_clock_enable(RCU_GPIOB);
    rcu_periph_clock_enable(RCU_GPIOE);
    rcu_periph_clock_enable(RCU_GPIOH);
    rcu_periph_clock_enable(RCU_GPIOI);
    rcu_periph_clock_enable(RCU_GPIOG);
    rcu_periph_clock_enable(RCU_GPIOF);

    /* configure HSYNC(PI10), VSYNC(PI9), PCLK(PG7) */
    /* configure LCD_R7(PG6), LCD_R6(PH12), LCD_R5(PH11), LCD_R4(PH10), LCD_R3(PH9),LCD_R2(PH8), 
                 LCD_R1(PH3), LCD_R0(PH2), LCD_G7(PI2), LCD_G6(PI1), LCD_G5(PI0), LCD_G4(PH15), 
                 LCD_G3(PH14), LCD_G2(PH13),LCD_G1(PE6), LCD_G0(PE5),LCD_B7(PI7), LCD_B6(PI6), 
                 LCD_B5(PI5), LCD_B4(PI4), LCD_B3(PG11),LCD_B2(PG10), LCD_B1(PG12), LCD_B0(PE4) */    
    /* TLI pins AF configure */
    gpio_af_set(GPIOE,GPIO_AF_14,GPIO_PIN_5); 
    gpio_af_set(GPIOE,GPIO_AF_14,GPIO_PIN_6);  
    gpio_af_set(GPIOE,GPIO_AF_14,GPIO_PIN_4);
    
    gpio_af_set(GPIOH,GPIO_AF_14,GPIO_PIN_2);  
    gpio_af_set(GPIOH,GPIO_AF_14,GPIO_PIN_3);
    gpio_af_set(GPIOH,GPIO_AF_14,GPIO_PIN_8);
    gpio_af_set(GPIOH,GPIO_AF_14,GPIO_PIN_9);  
    gpio_af_set(GPIOH,GPIO_AF_14,GPIO_PIN_10); 
    gpio_af_set(GPIOH,GPIO_AF_14,GPIO_PIN_11);  
    gpio_af_set(GPIOH,GPIO_AF_14,GPIO_PIN_12); 
    gpio_af_set(GPIOH,GPIO_AF_14,GPIO_PIN_13);  
    gpio_af_set(GPIOH,GPIO_AF_14,GPIO_PIN_14);     
    gpio_af_set(GPIOH,GPIO_AF_14,GPIO_PIN_15); 
    
    gpio_af_set(GPIOI,GPIO_AF_14,GPIO_PIN_0);  
    gpio_af_set(GPIOI,GPIO_AF_14,GPIO_PIN_1); 
    gpio_af_set(GPIOI,GPIO_AF_14,GPIO_PIN_2);     
    gpio_af_set(GPIOI,GPIO_AF_14,GPIO_PIN_4); 
    gpio_af_set(GPIOI,GPIO_AF_14,GPIO_PIN_5);  
    gpio_af_set(GPIOI,GPIO_AF_14,GPIO_PIN_6);     
    gpio_af_set(GPIOI,GPIO_AF_14,GPIO_PIN_7);
    gpio_af_set(GPIOI,GPIO_AF_14,GPIO_PIN_9);     
    gpio_af_set(GPIOI,GPIO_AF_14,GPIO_PIN_10);
    
    gpio_af_set(GPIOG,GPIO_AF_14,GPIO_PIN_6);     
    gpio_af_set(GPIOG,GPIO_AF_14,GPIO_PIN_7);
    gpio_af_set(GPIOG,GPIO_AF_14,GPIO_PIN_10);
    gpio_af_set(GPIOG,GPIO_AF_14,GPIO_PIN_11);     
    gpio_af_set(GPIOG,GPIO_AF_14,GPIO_PIN_12);    

    gpio_af_set(GPIOF,GPIO_AF_14,GPIO_PIN_10);  

    /* TLI GPIO configure */
    gpio_mode_set(GPIOE, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);
    gpio_output_options_set(GPIOE, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6);

    gpio_mode_set(GPIOH, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                 |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);
    gpio_output_options_set(GPIOH, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10
                           |GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15);

    gpio_mode_set(GPIOI, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                 |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10);
    gpio_output_options_set(GPIOI, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2
                           |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_9|GPIO_PIN_10);
    
    gpio_mode_set(GPIOG, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);
    gpio_output_options_set(GPIOG, GPIO_OTYPE_PP,GPIO_OSPEED_50MHZ, GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12);
    gpio_mode_set(GPIOB, GPIO_MODE_OUTPUT, GPIO_PUPD_PULLUP, GPIO_PIN_15);
    gpio_output_options_set(GPIOB, GPIO_OTYPE_PP, GPIO_OSPEED_50MHZ,GPIO_PIN_15);
    gpio_bit_set(GPIOB,GPIO_PIN_15);
}


#define MAX_CNT_DATA 5

static int acmp(const void *a,const void *b)
{
	return(*(int *)a - *(int *)b);
}

static int
readxy(int *x, int *y)
{
	int a[MAX_CNT_DATA],b[MAX_CNT_DATA];
	int i = 0;
    int num_x,num_y;
	unsigned short int datx[3];
	unsigned short int daty[3];
	unsigned short int temp_x,temp_y;

	*x = 0;
	*y = 0;
	
	memset(a,0,MAX_CNT_DATA);
	memset(b,0,MAX_CNT_DATA);
	
	for(i = 0; i < MAX_CNT_DATA; i ++){
		num_x = Touch_GetPhyX();
        num_y = Touch_GetPhyY();
        if(num_x && num_y == 0) {
            return 3;
        } else {
            a[i] = num_x;
            b[i] = num_y;
        }
	}
	

	qsort (a,MAX_CNT_DATA,sizeof(a[0]),acmp);		//快速排序，就是中值滤波
	qsort (b,MAX_CNT_DATA,sizeof(b[0]),acmp);
	
	
	datx[0] = a[MAX_CNT_DATA / 2 - 1];
	datx[1] = a[MAX_CNT_DATA / 2];
	datx[2] = a[MAX_CNT_DATA / 2 + 1];
	
	daty[0] = b[MAX_CNT_DATA / 2 - 1];
	daty[1] = b[MAX_CNT_DATA / 2];
	daty[2] = b[MAX_CNT_DATA / 2 + 1];
	
	if(
		abs(datx[0] - datx[1]) > 20 || 
		abs(datx[1] - datx[2]) > 20 ||
		abs(daty[0] - daty[1]) > 20 ||
		abs(daty[1] - daty[2]) > 20
		)return 2;

 	temp_x = (datx[0] + datx[1] + datx[2]) / 3;
 	temp_y = (daty[0] + daty[1] + daty[2]) / 3;
	
	*x = temp_x;
	*y = temp_y;

	return 0;
}

int BOARD_Touch_Poll(int *pX, int *pY, int *pPressFlg)
{    
    *pX = 1;
    *pY = 1;
    static int x=190,y=173;
    static int x1=3901,y1=4100;
    int touch_x,touch_y;
    if (!gpio_input_bit_get(GPIOD, GPIO_PIN_7)) {
        if(readxy(&touch_x, &touch_y) == 0) {
            *pX = 320-((touch_x-x)/((x1 - x)/320));
            *pY = 480-((4150 - touch_y - y)/((y1-y)/480));
            
            *pPressFlg = 1;
        } else {
            return 1;
        }
    }
    return 1;
}
