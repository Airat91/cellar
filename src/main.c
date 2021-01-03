
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"
#include "dcts.h"
#include "dcts_config.h"
#include "pin_map.h"
#include "buttons.h"
#include "LCD.h"
#include "adc.h"
#include "portable.h"
#include "am2302.h"
#include "menu.h"
#include "flash.h"
#include "uart.h"
#include "modbus.h"

/**
  * @defgroup MAIN
  */

#define FEEDER 0
#define DEFAULT_TASK_PERIOD 100
#define RELEASE 0

typedef enum{
    READ_FLOAT_SIGNED = 0,
    READ_FLOAT_UNSIGNED,
}read_float_bkp_sign_t;


/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
osThreadId defaultTaskHandle;
osThreadId buttonsTaskHandle;
osThreadId displayTaskHandle;
osThreadId menuTaskHandle;
osThreadId controlTaskHandle;
osThreadId adcTaskHandle;
osThreadId am2302TaskHandle;
osThreadId navigationtTaskHandle;
osThreadId uartTaskHandle;


/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
//static void MX_GPIO_Init(void);
static void MX_IWDG_Init(void);
static void MX_RTC_Init(void);
//static void MX_ADC1_Init(void);
//static void MX_USART1_UART_Init(void);
static void tim2_init(void);
static void main_page_print(void);
static void main_menu_print(void);
static void error_page_print(menu_page_t page);
static void save_page_print (void);
static void info_print (void);
static void meas_channels_print(void);
static void calib_print(uint8_t start_channel);
static void mdb_print(void);
static void display_print(void);
static void save_params(void);
static void restore_params(void);
static void save_to_bkp(u8 bkp_num, u8 var);
static void save_float_to_bkp(u8 bkp_num, float var);
static u8 read_bkp(u8 bkp_num);
static float read_float_bkp(u8 bkp_num, u8 sign);
static void led_lin_init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

uint32_t us_cnt_H = 0;
navigation_t navigation_style = MENU_NAVIGATION;
edit_val_t edit_val = {0};
saved_to_flash_t config;
static const uint16_t def_lvl_calib_table[6] = {
    375,
    738,
    1102,
    1466,
    1829,
    2193,
};
static const uint16_t def_tmpr_calib_table[11] = {
    3137,
    2727,
    2275,
    1826,
    1421,
    1082,
    813,
    607,
    454,
    341,
    258,
};
static const uart_bitrate_t bitrate_array[14] = {
    BITRATE_600,
    BITRATE_1200,
    BITRATE_2400,
    BITRATE_4800,
    BITRATE_9600,
    BITRATE_14400,
    BITRATE_19200,
    BITRATE_28800,
    BITRATE_38400,
    BITRATE_56000,
    BITRATE_57600,
    BITRATE_115200,
    BITRATE_128000,
    BITRATE_256000,
};
static uint16_t bitrate_array_pointer = 0;


/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void){

    HAL_Init();
    SystemClock_Config();
    tim2_init();
    dcts_init();
    restore_params();
    led_lin_init();
#if RELEASE
    MX_IWDG_Init();
#endif //RELEASE
    /*
    MX_RTC_Init();
    */
    /*
    osThreadDef(own_task, default_task, osPriorityNormal, 0, 364);
    defaultTaskHandle = osThreadCreate(osThread(own_task), NULL);
    */

    osThreadDef(display_task, display_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*4);
    displayTaskHandle = osThreadCreate(osThread(display_task), NULL);

    osThreadDef(adc_task, adc_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*2);
    adcTaskHandle = osThreadCreate(osThread(adc_task), NULL);

    osThreadDef(buttons_task, buttons_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    buttonsTaskHandle = osThreadCreate(osThread(buttons_task), NULL);

    osThreadDef(am2302_task, am2302_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    am2302TaskHandle = osThreadCreate(osThread(am2302_task), NULL);

    osThreadDef(navigation_task, navigation_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    navigationtTaskHandle = osThreadCreate(osThread(navigation_task), NULL);

    osThreadDef(uart_task, uart_task, osPriorityHigh, 0, configMINIMAL_STACK_SIZE*4);
    uartTaskHandle = osThreadCreate(osThread(uart_task), NULL);


    /* Start scheduler */
    osKernelStart();

    while (1)  {

    }

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.LSIState = RCC_LSI_ON;
    RCC_OscInitStruct.LSEState = RCC_LSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Initializes the CPU, AHB and APB busses clocks
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
            |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC;
    PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
    PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    /**Configure the Systick interrupt time
    */
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick
    */
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

    /* SysTick_IRQn interrupt configuration */
    HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* RTC init function */
static void MX_RTC_Init(void){
    RTC_TimeTypeDef sTime = {0};
    RTC_DateTypeDef sDate = {0};
    __HAL_RCC_BKP_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_RTC_ENABLE();
    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    u32 data;
    const  u32 data_c = 0x1234;
    data = BKP->DR1;
    if(data!=data_c){   // set default values
        HAL_PWR_EnableBkUpAccess();
        BKP->DR1 = data_c;
        HAL_PWR_DisableBkUpAccess();

        sTime.Hours = dcts.dcts_rtc.hour;
        sTime.Minutes = dcts.dcts_rtc.minute;
        sTime.Seconds = dcts.dcts_rtc.second;

        sDate.Date = dcts.dcts_rtc.day;
        sDate.Month = dcts.dcts_rtc.month;
        sDate.Year = (uint8_t)(dcts.dcts_rtc.year - 2000);

        HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
        HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

    }else{  // read data from bkpram
        HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
        HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

        dcts.dcts_rtc.hour = sTime.Hours;
        dcts.dcts_rtc.minute = sTime.Minutes;
        dcts.dcts_rtc.second = sTime.Seconds;

        dcts.dcts_rtc.day = sDate.Date;
        dcts.dcts_rtc.month = sDate.Month;
        dcts.dcts_rtc.year = sDate.Year + 2000;
        dcts.dcts_rtc.weekday = sDate.WeekDay;
    }
}


/**
 * @brief default_task
 * @param argument - None
 * @todo add group
 */
void default_task(void const * argument){

    (void)argument;
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;
    uint32_t last_wake_time = osKernelSysTick();

    //HAL_IWDG_Refresh(&hiwdg);
    while(1){
        HAL_RTC_GetDate(&hrtc,&date,RTC_FORMAT_BIN);
        HAL_RTC_GetTime(&hrtc,&time,RTC_FORMAT_BIN);

        dcts.dcts_rtc.hour = time.Hours;
        dcts.dcts_rtc.minute = time.Minutes;
        dcts.dcts_rtc.second = time.Seconds;

        dcts.dcts_rtc.day = date.Date;
        dcts.dcts_rtc.month = date.Month;
        dcts.dcts_rtc.year = date.Year + 2000;
        dcts.dcts_rtc.weekday = date.WeekDay;

        //HAL_IWDG_Refresh(&hiwdg);
        osDelayUntil(&last_wake_time, DEFAULT_TASK_PERIOD);
    }
}

/**
 * @brief display_task
 * @param argument
 */
#define display_task_period 500
void display_task(void const * argument){
    (void)argument;
    menu_init();
    LCD_init();
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
#if RELEASE
        HAL_IWDG_Refresh();
#endif //RELEASE
        LCD_clr();
        switch (selectedMenuItem->Page){
        case MAIN_PAGE:
            main_page_print();
            break;
        case MAIN_MENU:
        case COMMON_INFO:
        case MEAS_CHANNELS:
        case LVL_CALIB:
        case TMPR_CALIB:
        case CONNECTION:
        case DISPLAY:
            main_menu_print();
            break;
        case INFO:
            info_print();
            break;
        case MEAS_CH_0:
        case MEAS_CH_1:
        case MEAS_CH_2:
        case MEAS_CH_3:
        case MEAS_CH_4:
        case MEAS_CH_5:
        case MEAS_CH_6:
        case MEAS_CH_7:
        case MEAS_CH_8:
        case MEAS_CH_9:
        case MEAS_CH_10:
        case MEAS_CH_11:
        case MEAS_CH_12:
            meas_channels_print();
            break;
        case LVL_0:
        case LVL_20:
        case LVL_40:
        case LVL_60:
        case LVL_80:
        case LVL_100:
            calib_print(LVL_0);
            break;
        case ADC_0:
        case ADC_10:
        case ADC_20:
        case ADC_30:
        case ADC_40:
        case ADC_50:
        case ADC_60:
        case ADC_70:
        case ADC_80:
        case ADC_90:
        case ADC_100:
            calib_print(ADC_0);
            break;
        case MDB_ADDR:
        case MDB_BITRATE:
        case MDB_OVERRUN_ERR:
        case MDB_PARITY_ERR:
        case MDB_FRAME_ERR:
        case MDB_NOISE_ERR:
            mdb_print();
            break;
        case LIGHT_LVL:
        case AUTO_OFF:
            display_print();
            break;
        case SAVE_CHANGES:
            save_page_print();
            break;
        default:
            error_page_print(selectedMenuItem->Page);
        }

        LCD_update();
        if((LCD.auto_off != 0)&&(LCD.backlight == LCD_BACKLIGHT_ON)){
            LCD.auto_off_timeout += display_task_period;
            if(LCD.auto_off_timeout > (uint32_t)LCD.auto_off * 10000){
                LCD.auto_off_timeout = 0;
                LCD_backlight_shutdown();
            }
        }
        osDelayUntil(&last_wake_time, display_task_period);
    }
}

#define navigation_task_period 20
void navigation_task (void const * argument){
    (void)argument;
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        switch (navigation_style){
        case MENU_NAVIGATION:
            if((pressed_time[BUTTON_UP].pressed > 0)&&(pressed_time[BUTTON_UP].pressed < navigation_task_period)){
                menuChange(selectedMenuItem->Previous);
            }
            if((pressed_time[BUTTON_DOWN].pressed > 0)&&(pressed_time[BUTTON_DOWN].pressed < navigation_task_period)){
                menuChange(selectedMenuItem->Next);
            }
            if((pressed_time[BUTTON_LEFT].pressed > 0)&&(pressed_time[BUTTON_LEFT].pressed < navigation_task_period)){
                menuChange(selectedMenuItem->Parent);
            }
            if((pressed_time[BUTTON_RIGHT].pressed > 0)&&(pressed_time[BUTTON_RIGHT].pressed < navigation_task_period)){
                menuChange(selectedMenuItem->Child);
            }
            if((pressed_time[BUTTON_OK].pressed > 0)&&(pressed_time[BUTTON_OK].pressed < navigation_task_period)){
                menuChange(selectedMenuItem->Child);
            }
            break;
        case DIGIT_EDIT:
            if((pressed_time[BUTTON_UP].pressed > 0)&&(pressed_time[BUTTON_UP].pressed < navigation_task_period)){
                if(*edit_val.p_val < edit_val.val_max){
                    *edit_val.p_val += uint16_pow(10, (uint16_t)edit_val.digit);
                }
                if((*edit_val.p_val > edit_val.val_max)||(*edit_val.p_val < edit_val.val_min)){ //if out of range
                    *edit_val.p_val = edit_val.val_max;
                }
            }
            if((pressed_time[BUTTON_DOWN].pressed > 0)&&(pressed_time[BUTTON_DOWN].pressed < navigation_task_period)){
                if(*edit_val.p_val > edit_val.val_min){
                    *edit_val.p_val -= uint16_pow(10, (uint16_t)edit_val.digit);
                }
                if((*edit_val.p_val > edit_val.val_max)||(*edit_val.p_val < edit_val.val_min)){ //if out of range
                    *edit_val.p_val = edit_val.val_min;
                }
            }
            if((pressed_time[BUTTON_LEFT].pressed > 0)&&(pressed_time[BUTTON_LEFT].pressed < navigation_task_period)){
                if(edit_val.digit < edit_val.digit_max){
                    edit_val.digit++;
                }
            }
            if((pressed_time[BUTTON_RIGHT].pressed > 0)&&(pressed_time[BUTTON_RIGHT].pressed < navigation_task_period)){
                if(edit_val.digit > 0){
                    edit_val.digit--;
                }
            }
            if((pressed_time[BUTTON_OK].pressed > navigation_task_period)){
                while(pressed_time[BUTTON_OK].last_state == BUTTON_PRESSED){
                }
                navigation_style = MENU_NAVIGATION;
            }

            break;
        }
        if((pressed_time[BUTTON_BREAK].pressed > 0)&&(pressed_time[BUTTON_BREAK].pressed < navigation_task_period)){
            if(LCD.auto_off == 0){
                LCD_backlight_toggle();
            }
        }
        if((pressed_time[BUTTON_SET].pressed > 0)&&(pressed_time[BUTTON_SET].pressed < navigation_task_period)){
            save_params();
        }
        osDelayUntil(&last_wake_time, navigation_task_period);
    }
}

static void error_page_print(menu_page_t page){
    char string[100];

    LCD_set_xy(25,45);
    sprintf(string, "СТРАНИЦА НЕ");
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_set_xy(25,35);
    sprintf(string, "НАЙДЕНА: %d", page);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, "<назад");
    LCD_set_xy(0,0);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,0,42,11);
}

static void main_page_print(void){
    char string[100];
    const float vmax = 114.0;
    uint8_t high_lev = 0;

    // print water tank
    LCD_fill_area(0,0,50,63,LCD_COLOR_BLACK);
    LCD_fill_area(1,1,49,62,LCD_COLOR_WHITE);

    // print values
    sprintf(string, "%3.1f%s", dcts_meas[WTR_TMPR].value, dcts_meas[WTR_TMPR].unit_cyr);
    LCD_set_xy(align_text_center(string, Font_7x10)-38,45);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "%3.1f%s", dcts_meas[WTR_LVL].value, dcts_meas[WTR_LVL].unit_cyr);
    LCD_set_xy(align_text_center(string, Font_7x10)-38,5);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "Горячая");
    LCD_set_xy(align_text_center(string, Font_7x10)-38,30);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "вода");
    LCD_set_xy(align_text_center(string, Font_7x10)-38,20);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);


    // fill water level
    high_lev = (uint8_t)(dcts_meas[WTR_LVL].value/vmax*62);
    if(high_lev > 61){
        high_lev = 61;
    }
    LCD_invert_area(1,1,49,high_lev+1);

    sprintf(string, "Предбанник");
    LCD_set_xy(align_text_center(string, Font_7x10)+27,52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(52,53,127,63);
    if(dcts_meas[PREDBANNIK_HUM].valid){
        sprintf(string, "%.1f%s/%.0f%s", dcts_meas[PREDBANNIK_TMPR].value, dcts_meas[PREDBANNIK_TMPR].unit_cyr, dcts_meas[PREDBANNIK_HUM].value, dcts_meas[PREDBANNIK_HUM].unit_cyr);
    }else{
        sprintf(string, "Нет связи");
    }
    LCD_set_xy(align_text_center(string, Font_7x10)+27,41);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, "Моечная");
    LCD_set_xy(align_text_center(string, Font_7x10)+27,31);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(52,32,127,42);
    if(dcts_meas[MOYKA_HUM].valid){
        sprintf(string, "%.1f%s/%.0f%s", dcts_meas[MOYKA_TMPR].value, dcts_meas[MOYKA_TMPR].unit_cyr, dcts_meas[MOYKA_HUM].value, dcts_meas[MOYKA_HUM].unit_cyr);
    }else{
        sprintf(string, "Нет связи");
    }
    LCD_set_xy(align_text_center(string, Font_7x10)+27,20);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, "Парная");
    LCD_set_xy(align_text_center(string, Font_7x10)+27,10);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(52,12,127,21);
    if(dcts_meas[MOYKA_HUM].valid){
        sprintf(string, "%.1f%s", dcts_meas[PARILKA_TMPR].value, dcts_meas[PARILKA_TMPR].unit_cyr);
    }else{
        sprintf(string, "Нет связи");
    }
    LCD_set_xy(align_text_center(string, Font_7x10)+27,0);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
}

static void main_menu_print (void){
    char string[100];
    menuItem* temp = selectedMenuItem->Parent;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,53,127,63);

    temp = selectedMenuItem->Previous;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),39);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, selectedMenuItem->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(5,26,120,38);
    LCD_invert_area(6,27,119,37);

    temp = selectedMenuItem->Next;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, "<назад      выбор>");
    LCD_set_xy(0,0);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,0,42,11);
    LCD_invert_area(83,0,127,11);
}

static void info_print (void){
    char string[100];
    menuItem* temp = selectedMenuItem->Parent;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,53,127,63);

    sprintf(string, "Имя: %s",dcts.dcts_name_cyr);
    LCD_set_xy(2,40);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "Тип: %d",dcts.dcts_id);
    LCD_set_xy(2,30);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "Адрес: %d",dcts.dcts_address);
    LCD_set_xy(2,20);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "Версия: %s",dcts.dcts_ver);
    LCD_set_xy(2,10);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, "<назад");
    LCD_set_xy(0,0);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,0,42,11);
}

static void meas_channels_print(void){
    char string[100];
    uint8_t channel = 0;
    menuItem* temp = selectedMenuItem->Parent;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,53,127,63);

    temp = selectedMenuItem;
    for(uint8_t i = 0; i < 2; i++){
        channel = (uint8_t)temp->Page - MEAS_CH_0;
        sprintf(string, "%s:",dcts_meas[channel].name_cyr);
        LCD_set_xy(2,41-21*i);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        sprintf(string, "%.2f(%s) ", dcts_meas[channel].value, dcts_meas[channel].unit_cyr);
        LCD_set_xy(align_text_right(string,Font_7x10),31-21*i);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        temp = temp->Next;
    }

    sprintf(string, "<назад");
    LCD_set_xy(0,0);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,0,42,11);

}

static void calib_print (uint8_t start_channel){
    char string[100];
    menuItem* temp = selectedMenuItem->Parent;
    uint16_t* calib_table;
    sprintf(string, temp->Text);
    LCD_set_xy(2,52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,53,127,63);
    if(temp->Page == LVL_CALIB){
        sprintf(string, "%.0f",dcts_meas[WTR_LVL_ADC].value);
        LCD_set_xy(align_text_right(string,Font_7x10),52);
        LCD_print(string,&Font_7x10,LCD_COLOR_WHITE);
        calib_table = config.params.lvl_calib_table;
    }else if(temp->Page == TMPR_CALIB){
        sprintf(string, "%.0f",dcts_meas[WTR_TMPR_ADC].value);
        LCD_set_xy(align_text_right(string,Font_7x10),52);
        LCD_print(string,&Font_7x10,LCD_COLOR_WHITE);
        calib_table = config.params.tmpr_calib_table;
    }

    temp = selectedMenuItem->Previous;
    sprintf(string, temp->Text);
    LCD_set_xy(1,39);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "%d",calib_table[(uint8_t)temp->Page-start_channel]);
    LCD_set_xy(align_text_right(string, Font_7x10)-1,39);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, selectedMenuItem->Text);
    LCD_set_xy(1,26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "%d",calib_table[(uint8_t)selectedMenuItem->Page-start_channel]);
    LCD_set_xy(align_text_right(string, Font_7x10)-1,26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,26,127,39);
    LCD_invert_area(1,27,126,38);

    temp = selectedMenuItem->Next;
    sprintf(string, temp->Text);
    LCD_set_xy(1,14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "%d",calib_table[(uint8_t)temp->Page-start_channel]);
    LCD_set_xy(align_text_right(string, Font_7x10)-1,14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    switch (navigation_style) {
    case MENU_NAVIGATION:
        sprintf(string, "<назад   изменить>");
        LCD_set_xy(0,0);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        LCD_invert_area(0,0,42,11);
        LCD_invert_area(62,0,127,11);

        if(pressed_time[BUTTON_RIGHT].pressed > navigation_task_period){
            while(pressed_time[BUTTON_RIGHT].last_state == BUTTON_PRESSED){
            }
            navigation_style = DIGIT_EDIT;
            edit_val.digit_max = 3;
            edit_val.digit = 0;
            edit_val.val_min = 0;
            edit_val.val_max = 0x4095;
            edit_val.p_val = &calib_table[(uint8_t)selectedMenuItem->Page-start_channel];
        }
        break;
    case DIGIT_EDIT:
        sprintf(string, "*ввод");
        LCD_set_xy(align_text_center(string, Font_7x10),0);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        LCD_invert_area(46,0,82,11);

        LCD_invert_area(127-(edit_val.digit+1)*Font_7x10.FontWidth,27,126-edit_val.digit*Font_7x10.FontWidth,38);
        break;
    }

}


static void mdb_print(void){
    static uint8_t reinit_uart = 0;
    char string[100];
    menuItem* temp = selectedMenuItem->Parent;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string,Font_7x10),52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,53,127,63);

    temp = selectedMenuItem->Previous;
    sprintf(string, temp->Text);
    LCD_set_xy(1,39);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    switch (temp->Page) {
    case MDB_OVERRUN_ERR:
        sprintf(string, "%d",uart_2.overrun_err_cnt);
        break;
    case MDB_PARITY_ERR:
        sprintf(string, "%d",uart_2.parity_err_cnt);
        break;
    case MDB_FRAME_ERR:
        sprintf(string, "%d",uart_2.frame_err_cnt);
        break;
    case MDB_NOISE_ERR:
        sprintf(string, "%d",uart_2.noise_err_cnt);
        break;
    case MDB_ADDR:
        sprintf(string, "%d",config.params.mdb_address);
        break;
    case MDB_BITRATE:
        sprintf(string, "%d",bitrate_array[bitrate_array_pointer]*100);
        break;
    }
    LCD_set_xy(align_text_right(string, Font_7x10)-1,39);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    sprintf(string, selectedMenuItem->Text);
    LCD_set_xy(1,26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    switch (selectedMenuItem->Page) {
    case MDB_OVERRUN_ERR:
        sprintf(string, "%d",uart_2.overrun_err_cnt);
        break;
    case MDB_PARITY_ERR:
        sprintf(string, "%d",uart_2.parity_err_cnt);
        break;
    case MDB_FRAME_ERR:
        sprintf(string, "%d",uart_2.frame_err_cnt);
        break;
    case MDB_NOISE_ERR:
        sprintf(string, "%d",uart_2.noise_err_cnt);
        break;
    case MDB_ADDR:
        sprintf(string, "%d",config.params.mdb_address);
        break;
    case MDB_BITRATE:
        sprintf(string, "%d",bitrate_array[bitrate_array_pointer]*100);
        break;
    }
    LCD_set_xy(align_text_right(string, Font_7x10)-1,26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,26,127,39);
    LCD_invert_area(1,27,126,38);

    temp = selectedMenuItem->Next;
    sprintf(string, temp->Text);
    LCD_set_xy(1,14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    switch (temp->Page) {
    case MDB_OVERRUN_ERR:
        sprintf(string, "%d",uart_2.overrun_err_cnt);
        break;
    case MDB_PARITY_ERR:
        sprintf(string, "%d",uart_2.parity_err_cnt);
        break;
    case MDB_FRAME_ERR:
        sprintf(string, "%d",uart_2.frame_err_cnt);
        break;
    case MDB_NOISE_ERR:
        sprintf(string, "%d",uart_2.noise_err_cnt);
        break;
    case MDB_ADDR:
        sprintf(string, "%d",config.params.mdb_address);
        break;
    case MDB_BITRATE:
        sprintf(string, "%d",bitrate_array[bitrate_array_pointer]*100);
        break;
    }
    LCD_set_xy(align_text_right(string, Font_7x10)-1,14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    switch (navigation_style) {
    case MENU_NAVIGATION:
        if(reinit_uart){
            reinit_uart = 0;
            config.params.mdb_bitrate = (uint16_t)bitrate_array[bitrate_array_pointer];
            uart_init(config.params.mdb_bitrate, 8, 1, PARITY_NONE, 10000, UART_CONN_LOST_TIMEOUT);
        }

        sprintf(string, "<назад");
        LCD_set_xy(0,0);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        LCD_invert_area(0,0,42,11);

        switch (selectedMenuItem->Page) {
        case MDB_ADDR:
        case MDB_BITRATE:
            sprintf(string, "изменить>");
            LCD_set_xy(align_text_right(string,Font_7x10),0);
            LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
            LCD_invert_area(62,0,127,11);

            if(pressed_time[BUTTON_RIGHT].pressed > navigation_task_period){
                while(pressed_time[BUTTON_RIGHT].last_state == BUTTON_PRESSED){
                }
                navigation_style = DIGIT_EDIT;
                switch (selectedMenuItem->Page) {
                case MDB_ADDR:
                    edit_val.digit_max = 2;
                    edit_val.digit = 0;
                    edit_val.val_min = 0;
                    edit_val.val_max = 255;
                    edit_val.p_val = &config.params.mdb_address;
                    break;
                case MDB_BITRATE:
                    edit_val.digit_max = 0;
                    edit_val.digit = 0;
                    edit_val.val_min = 0;
                    edit_val.val_max = 13;
                    edit_val.p_val = &bitrate_array_pointer;
                    break;
                }
            }
        }
        break;
    case DIGIT_EDIT:
        reinit_uart = 1;

        sprintf(string, "*ввод");
        LCD_set_xy(align_text_center(string, Font_7x10),0);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        LCD_invert_area(46,0,82,11);

        switch (selectedMenuItem->Page) {
        case MDB_BITRATE:
            sprintf(string,"%d",bitrate_array[bitrate_array_pointer]*100);
            LCD_invert_area(126 - (uint8_t)strlen(string)*Font_7x10.FontWidth,27,126,38);
            break;
        default:
            LCD_invert_area(127-(edit_val.digit+1)*Font_7x10.FontWidth,27,126-edit_val.digit*Font_7x10.FontWidth,38);
        }
        break;
    }
}
static void display_print(void){
    static uint8_t reinit_backlight = 0;
    char string[100];
    menuItem* temp = selectedMenuItem->Parent;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string,Font_7x10),52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,53,127,63);

    sprintf(string, selectedMenuItem->Text);
    LCD_set_xy(1,26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    switch (selectedMenuItem->Page) {
    case LIGHT_LVL:
        sprintf(string, "%d%%",LCD.backlight_lvl*10);
        break;
    case AUTO_OFF:
        if(LCD.auto_off == 0){
            sprintf(string, "выкл");
        }else{
            sprintf(string, "%dсек",LCD.auto_off*10);
        }
        break;
    }
    LCD_set_xy(align_text_right(string, Font_7x10)-1,26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,26,127,39);
    LCD_invert_area(1,27,126,38);

    temp = selectedMenuItem->Next;
    sprintf(string, temp->Text);
    LCD_set_xy(1,14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    switch (temp->Page) {
    case LIGHT_LVL:
        sprintf(string, "%d%%",LCD.backlight_lvl*10);
        break;
    case AUTO_OFF:
        if(LCD.auto_off == 0){
            sprintf(string, "выкл");
        }else{
            sprintf(string, "%dсек",LCD.auto_off*10);
        }
        break;
    }
    LCD_set_xy(align_text_right(string, Font_7x10)-1,14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);

    switch (navigation_style) {
    case MENU_NAVIGATION:
        if(reinit_backlight == 1){
            reinit_backlight = 0;
            config.params.lcd_backlight_lvl = LCD.backlight_lvl;
            config.params.lcd_backlight_time = LCD.auto_off;
        }

        sprintf(string, "<назад   изменить>");
        LCD_set_xy(0,0);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        LCD_invert_area(0,0,42,11);
        LCD_invert_area(62,0,127,11);

        if(pressed_time[BUTTON_RIGHT].pressed > navigation_task_period){
            while(pressed_time[BUTTON_RIGHT].last_state == BUTTON_PRESSED){
            }
            navigation_style = DIGIT_EDIT;
            switch (selectedMenuItem->Page) {
            case LIGHT_LVL:
                edit_val.digit_max = 1;
                edit_val.digit = 0;
                edit_val.val_min = 0;
                edit_val.val_max = 10;
                edit_val.p_val = &LCD.backlight_lvl;
                break;
            case AUTO_OFF:
                edit_val.digit_max = 1;
                edit_val.digit = 0;
                edit_val.val_min = 0;
                edit_val.val_max = 60;
                edit_val.p_val = &LCD.auto_off;
                break;
            default:
                break;
            }
        }
        break;
    case DIGIT_EDIT:

        reinit_backlight = 1;

        sprintf(string, "*ввод");
        LCD_set_xy(align_text_center(string, Font_7x10),0);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        LCD_invert_area(46,0,82,11);

        switch (selectedMenuItem->Page) {
        case LIGHT_LVL:
            LCD_backlight_timer_init();
            LCD_backlight_on();
            LCD_invert_area(113-(edit_val.digit+1)*Font_7x10.FontWidth,27,112-edit_val.digit*Font_7x10.FontWidth,38);
            break;
        case AUTO_OFF:
            LCD_invert_area(99-(edit_val.digit+1)*Font_7x10.FontWidth,27,98-edit_val.digit*Font_7x10.FontWidth,38);
            break;
        }
        break;
    }
}


static void save_page_print (void){
    char string[100];

    LCD_fill_area(5,20,123,48,LCD_COLOR_BLACK);
    LCD_fill_area(6,21,122,47,LCD_COLOR_WHITE);
    sprintf(string, "СОХРАНЕНИЕ НОВЫХ");
    LCD_set_xy(align_text_center(string, Font_7x10),32);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "КОЭФФИЦИЕНТОВ");
    LCD_set_xy(align_text_center(string, Font_7x10),22);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
}


/**
 * @brief am2302_task
 * @param argument
 */

#define am2302_task_period 3000
void am2302_task (void const * argument){
    (void)argument;
    uint32_t last_wake_time = osKernelSysTick();
    am2302_init();
    am2302_data_t am2302 = {0};
    while(1){
        am2302 = am2302_get(2);
        taskENTER_CRITICAL();
        if(am2302.error == 1){
            dcts_meas[PREDBANNIK_HUM].valid = FALSE;
            dcts_meas[PREDBANNIK_TMPR].valid = FALSE;
        }else{
            dcts_meas[PREDBANNIK_HUM].value = (float)am2302.hum/10;
            dcts_meas[PREDBANNIK_HUM].valid = TRUE;
            dcts_meas[PREDBANNIK_TMPR].value = (float)am2302.tmpr/10;
            dcts_meas[PREDBANNIK_TMPR].valid = TRUE;
        }
        taskEXIT_CRITICAL();

        am2302 = am2302_get(1);
        taskENTER_CRITICAL();
        if(am2302.error == 1){
            dcts_meas[MOYKA_HUM].valid = FALSE;
            dcts_meas[MOYKA_TMPR].valid = FALSE;
        }else{
            dcts_meas[MOYKA_HUM].value = (float)am2302.hum/10;
            dcts_meas[MOYKA_HUM].valid = TRUE;
            dcts_meas[MOYKA_TMPR].value = (float)am2302.tmpr/10;
            dcts_meas[MOYKA_TMPR].valid = TRUE;
        }
        taskEXIT_CRITICAL();

        am2302 = am2302_get(0);
        taskENTER_CRITICAL();
        if(am2302.error == 1){
            dcts_meas[PARILKA_TMPR].valid = FALSE;
        }else{
            dcts_meas[PARILKA_TMPR].value = (float)am2302.tmpr/10;
            dcts_meas[PARILKA_TMPR].valid = TRUE;
        }
        taskEXIT_CRITICAL();

        osDelayUntil(&last_wake_time, am2302_task_period);
    }
}

#define uart_task_period 5
void uart_task(void const * argument){
    (void)argument;
    uart_init(config.params.mdb_bitrate, 8, 1, PARITY_NONE, 10000, UART_CONN_LOST_TIMEOUT);
    uint16_t tick = 0;
    char string[100];
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        if((uart_2.state & UART_STATE_RECIEVE)&&\
                ((uint16_t)(us_tim_get_value() - uart_2.timeout_last) > uart_2.timeout)){
            memcpy(uart_2.buff_received, uart_2.buff_in, uart_2.in_ptr);
            uart_2.received_len = uart_2.in_ptr;
            uart_2.in_ptr = 0;
            uart_2.state &= ~UART_STATE_RECIEVE;
            uart_2.state &= ~UART_STATE_ERROR;
            uart_2.state |= UART_STATE_IN_HANDING;
            uart_2.conn_last = 0;

            if(modbus_packet_for_me(uart_2.buff_received, uart_2.received_len)){
                uint16_t new_len = modbus_rtu_packet(uart_2.buff_received, uart_2.received_len);
                uart_send(uart_2.buff_received, new_len);
                uart_2.state &= ~UART_STATE_IN_HANDING;
            }
            if(uart_2.state & UART_STATE_IN_HANDING){
                dcts_packet_handle(uart_2.buff_received, uart_2.received_len);
            }else{
                uart_2.state &= ~UART_STATE_IN_HANDING;
            }
        }
        if(uart_2.conn_last > uart_2.conn_lost_timeout){
            uart_deinit();
            uart_init(config.params.mdb_bitrate, 8, 1, PARITY_NONE, 10000, UART_CONN_LOST_TIMEOUT);
        }
        if(tick == 1000/uart_task_period){
            tick = 0;
            HAL_GPIO_TogglePin(LED_PORT,LED_PIN);
            for(uint8_t i = 0; i < MEAS_NUM; i++){
                sprintf(string, "%s:\t%.1f(%s)\n",dcts_meas[i].name,(double)dcts_meas[i].value,dcts_meas[i].unit);
                if(i == MEAS_NUM - 1){
                    strncat(string,"\n",1);
                }
                //uart_send(string,(uint16_t)strlen(string));
            }
        }else{
            tick++;
            uart_2.conn_last += uart_task_period;
        }

        osDelayUntil(&last_wake_time, uart_task_period);
    }
}

uint16_t uint16_pow(uint16_t x, uint16_t pow){
    uint16_t result = 1;
    while(pow){
        result *= x;
        pow--;
    }
    return result;
}

/**
 * @brief Init us timer
 * @ingroup MAIN
 */
static void tim2_init(void){
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    __HAL_RCC_TIM2_CLK_ENABLE();
    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 71;
    htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim2.Init.Period = 0xFFFF;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    if (HAL_TIM_Base_Init(&htim2) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)  {
        _Error_Handler(__FILE__, __LINE__);
    }
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK){
        _Error_Handler(__FILE__, __LINE__);
    }
}
/**
 * @brief Get value from global us timer
 * @return global us timer value
 * @ingroup MAIN
 */
uint32_t us_tim_get_value(void){
    uint32_t value = us_cnt_H + TIM2->CNT;
    return value;
}
/**
 * @brief Us delayy
 * @param us - delau value
 * @ingroup MAIN
 */
void us_tim_delay(uint32_t us){
    uint32_t current;
    uint8_t with_yield;
    current = TIM2->CNT;
    with_yield = 0;
    if(us > TIME_YIELD_THRESHOLD){
        with_yield =1;
    }
    while ((TIM2->CNT - current)<us){
        if(with_yield){
            osThreadYield();
        }
    }
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    /* USER CODE BEGIN Callback 0 */

    /* USER CODE END Callback 0 */
    if (htim->Instance == TIM1) {
        HAL_IncTick();
    }

}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
    /* USER CODE BEGIN Error_Handler_Debug */
    /* User can add his own implementation to report the HAL error return state */
    while(1)
    {
    }
    /* USER CODE END Error_Handler_Debug */
}

static void save_params(void){
    static menuItem* current_menu;
    current_menu = selectedMenuItem;
    menuChange(&save_changes);

    int area_cnt = find_free_area();
    if(area_cnt < 0){
        uint32_t erase_error = 0;
        FLASH_EraseInitTypeDef flash_erase = {0};
        flash_erase.TypeErase = FLASH_TYPEERASE_PAGES;
        flash_erase.NbPages = 1;
        flash_erase.PageAddress = FLASH_SAVE_PAGE_ADDRESS;
        HAL_FLASH_Unlock();
        HAL_FLASHEx_Erase(&flash_erase, &erase_error);
        HAL_FLASH_Lock();
        area_cnt = 0;
    }
    for(uint8_t i = 0; i < SAVED_PARAMS_SIZE; i ++){
        save_to_flash(area_cnt, i, &config.word[i]);
    }
    // rewrite new params
    dcts.dcts_address = (uint8_t)config.params.mdb_address;
    uart_deinit();
    uart_init(config.params.mdb_bitrate, 8, 1, PARITY_NONE, 10000, UART_CONN_LOST_TIMEOUT);
    //delay for show message
    osDelay(2000);
    menuChange(current_menu);
}

static void restore_params(void){
    int area_cnt = find_free_area();
    if(area_cnt != 0){
        if(area_cnt == -1){
            // page is fill, actual values in last area
            area_cnt = SAVE_AREA_NMB - 1;
        }else{
            // set last filled area number
            area_cnt--;
        }
        uint16_t *addr;
        addr = (uint32_t)(FLASH_SAVE_PAGE_ADDRESS + area_cnt*SAVE_AREA_SIZE);
        for(uint8_t i = 0; i < SAVED_PARAMS_SIZE; i++){
            config.word[i] = *addr;
            addr++;
        }
    }else{
        //init default values if saved params not found
        config.params.mdb_address = dcts.dcts_address;
        config.params.mdb_bitrate = BITRATE_56000;
        memcpy(config.params.lvl_calib_table, def_lvl_calib_table, 6);
        memcpy(config.params.tmpr_calib_table, def_tmpr_calib_table, 11);
    }
    for(bitrate_array_pointer = 0; bitrate_array_pointer < 14; bitrate_array_pointer++){
        if(bitrate_array[bitrate_array_pointer] == config.params.mdb_bitrate){
            break;
        }
    }
}

static void save_to_bkp(u8 bkp_num, u8 var){
    uint32_t data = var;
    if(bkp_num%2 == 1){
        data = data << 8;
    }
    HAL_PWR_EnableBkUpAccess();
    switch (bkp_num / 2){
    case 0:
        BKP->DR1 |= data;
        break;
    case 1:
        BKP->DR2 |= data;
        break;
    case 2:
        BKP->DR3 |= data;
        break;
    case 3:
        BKP->DR4 |= data;
        break;
    case 4:
        BKP->DR5 |= data;
        break;
    case 5:
        BKP->DR6 |= data;
        break;
    case 6:
        BKP->DR7 |= data;
        break;
    case 7:
        BKP->DR8 |= data;
        break;
    case 8:
        BKP->DR9 |= data;
        break;
    case 9:
        BKP->DR10 |= data;
        break;
    }
    HAL_PWR_DisableBkUpAccess();
}

static void save_float_to_bkp(u8 bkp_num, float var){
    char buf[5] = {0};
    sprintf(buf, "%4.0f", (double)var);
    u8 data = (u8)atoi(buf);
    save_to_bkp(bkp_num, data);
}
static u8 read_bkp(u8 bkp_num){
    uint32_t data = 0;
    switch (bkp_num/2){
    case 0:
        data = BKP->DR1;
        break;
    case 1:
        data = BKP->DR2;
        break;
    case 2:
        data = BKP->DR3;
        break;
    case 3:
        data = BKP->DR4;
        break;
    case 4:
        data = BKP->DR5;
        break;
    case 5:
        data = BKP->DR6;
        break;
    case 6:
        data = BKP->DR7;
        break;
    case 7:
        data = BKP->DR8;
        break;
    case 8:
        data = BKP->DR9;
        break;
    case 9:
        data = BKP->DR10;
        break;
    }
    if(bkp_num%2 == 1){
        data = data >> 8;
    }
    return (u8)(data & 0xFF);
}
static float read_float_bkp(u8 bkp_num, u8 sign){
    u8 data = read_bkp(bkp_num);
    char buf[5] = {0};
    if(sign == READ_FLOAT_SIGNED){
        sprintf(buf, "%d", (s8)data);
    }else{
        sprintf(buf, "%d", data);
    }
    return atoff(buf);
}

static void led_lin_init(void){
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = LED_PIN;
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
    HAL_GPIO_Init (LED_PORT, &GPIO_InitStruct);
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
    /* USER CODE BEGIN 6 */
    /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

