
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
#include "ds18.h"
#include "time.h"
#include "math.h"

/**
  * @defgroup MAIN
  */

#define FEEDER 0
#define DEFAULT_TASK_PERIOD 100
#define RELEASE 1
#define DO_NUM 6
#define IN_CHANNEL_NUM 8
#define RTC_KEY 0xABCD

#if (DISP == ST7735_DISP)
    #include "st7735.h"
#endif

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef hpwmtim;
IWDG_HandleTypeDef hiwdg;
osThreadId rtcTaskHandle;
osThreadId buttonsTaskHandle;
osThreadId displayTaskHandle;
osThreadId menuTaskHandle;
osThreadId controlTaskHandle;
osThreadId adcTaskHandle;
osThreadId am2302TaskHandle;
osThreadId ds18TaskHandle;
osThreadId navigationtTaskHandle;
osThreadId uartTaskHandle;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void dcts_init (void);
static void channels_init(void);
static void MX_IWDG_Init(void);
static void RTC_Init(void);
static int RTC_write_cnt(time_t cnt_value);
static void print_header(void);
static void main_page_print(u8 tick);
static void menu_page_print(u8 tick);
static void value_print(u8 tick);
static void error_page_print(menu_page_t page);
static void save_page_print (u8 tick);
static void info_print (void);
static int get_param_value(char* string, menu_page_t page);
static void set_edit_value(menu_page_t page);
static void print_back(void);
static void print_enter_right(void);
static void print_enter_ok(void);
static void print_change(void);
static void save_params(void);
static void restore_params(void);
static void save_to_bkp(u8 bkp_num, uint16_t var);
//static void save_float_to_bkp(u8 bkp_num, float var);
static uint16_t read_bkp(u8 bkp_num);
//static float read_float_bkp(u8 bkp_num, u8 sign);
static int channel_PWM_timer_init(u8 channel);
static int channel_PWM_duty_set(u8 channel, float duty);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

uint32_t us_cnt_H = 0;
navigation_t navigation_style = MENU_NAVIGATION;
edit_val_t edit_val = {0};
//bkp_data_t * bkp_data_p;
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
static const char off_on_descr[2][10] = {
    "Выкл.",
    "Вкл.",
};
static const char manual_auto_descr[2][10] = {
    "Ручной",
    "Авто",
};
/*static const char ch_mode_descr[6][10] = {
    "Неактивен",
    "Аналоговый",
    "Дис. вход",
    "Дис. выход",
    "AM2302",
    "ШИМ-выход",
};*/

const in_channel_t input_ch[8] = {
    {.mode = CH_MODE_ADC,   .port = CH_0_PORT, .pin = CH_0_PIN, .adc_num = ADC1, .adc_channel = ADC_CHANNEL_0, .pwm_tim = TIM2, .pwm_channel = TIM_CHANNEL_1},
    {.mode = CH_MODE_ADC,   .port = CH_1_PORT, .pin = CH_1_PIN, .adc_num = ADC1, .adc_channel = ADC_CHANNEL_1, .pwm_tim = TIM2, .pwm_channel = TIM_CHANNEL_2},
    {.mode = CH_MODE_AM3202,.port = CH_2_PORT, .pin = CH_2_PIN, .adc_num = ADC1, .adc_channel = ADC_CHANNEL_2, .pwm_tim = TIM2, .pwm_channel = TIM_CHANNEL_3},
    {.mode = CH_MODE_AM3202,.port = CH_3_PORT, .pin = CH_3_PIN, .adc_num = ADC1, .adc_channel = ADC_CHANNEL_3, .pwm_tim = TIM2, .pwm_channel = TIM_CHANNEL_4},
    {.mode = CH_MODE_AM3202,.port = CH_4_PORT, .pin = CH_4_PIN, .adc_num = ADC1, .adc_channel = ADC_CHANNEL_6, .pwm_tim = TIM3, .pwm_channel = TIM_CHANNEL_1},
    {.mode = CH_MODE_PWM,   .port = CH_5_PORT, .pin = CH_5_PIN, .adc_num = ADC1, .adc_channel = ADC_CHANNEL_7, .pwm_tim = TIM3, .pwm_channel = TIM_CHANNEL_2},
    {.mode = CH_MODE_PWM,   .port = CH_6_PORT, .pin = CH_6_PIN, .adc_num = ADC1, .adc_channel = ADC_CHANNEL_8, .pwm_tim = TIM3, .pwm_channel = TIM_CHANNEL_3},
    {.mode = CH_MODE_NONE,  .port = CH_7_PORT, .pin = CH_7_PIN, .adc_num = ADC1, .adc_channel = ADC_CHANNEL_9, .pwm_tim = TIM3, .pwm_channel = TIM_CHANNEL_4},
};

const ch_t do_ch[6] = {
    {.pin = DO_0_PIN, .port = DO_0_PORT},
    {.pin = DO_1_PIN, .port = DO_1_PORT},
    {.pin = DO_2_PIN, .port = DO_2_PORT},
    {.pin = DO_3_PIN, .port = DO_3_PORT},
    {.pin = DO_4_PIN, .port = DO_4_PORT},
    {.pin = DO_5_PIN, .port = DO_5_PORT},
};


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
    refresh_watchdog();

    osThreadDef(rtc_task, rtc_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    rtcTaskHandle = osThreadCreate(osThread(rtc_task), NULL);

    osThreadDef(display_task, display_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*4);
    displayTaskHandle = osThreadCreate(osThread(display_task), NULL);

    osThreadDef(adc_task, adc_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE*3);
    adcTaskHandle = osThreadCreate(osThread(adc_task), NULL);

    osThreadDef(buttons_task, buttons_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    buttonsTaskHandle = osThreadCreate(osThread(buttons_task), NULL);

    osThreadDef(am2302_task, am2302_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    am2302TaskHandle = osThreadCreate(osThread(am2302_task), NULL);

    osThreadDef(navigation_task, navigation_task, osPriorityNormal, 0, configMINIMAL_STACK_SIZE);
    navigationtTaskHandle = osThreadCreate(osThread(navigation_task), NULL);

    osThreadDef(uart_task, uart_task, osPriorityHigh, 0, configMINIMAL_STACK_SIZE*4);
    uartTaskHandle = osThreadCreate(osThread(uart_task), NULL);

    osThreadDef(control_task, control_task, osPriorityHigh, 0, configMINIMAL_STACK_SIZE*2);
    controlTaskHandle = osThreadCreate(osThread(control_task), NULL);

    /*osThreadDef(ds18_task, ds18_task, osPriorityHigh, 0, configMINIMAL_STACK_SIZE*2);
    ds18TaskHandle = osThreadCreate(osThread(ds18_task), NULL);*/

    /* Start scheduler */
    osKernelStart();

    while (1)  {

    }

}

void dcts_init (void) {

    dcts.dcts_id = DCTS_ID_COMBINED;
    strcpy (dcts.dcts_ver, "0.2.3");
    strcpy (dcts.dcts_name, "Pogreb");
    strcpy (dcts.dcts_name_cyr, "Погреб");
    dcts.dcts_address = 0x0B;
    dcts.dcts_rtc.day = 1;
    dcts.dcts_rtc.month = 1;
    dcts.dcts_rtc.year = 2021;
    dcts.dcts_rtc.weekday = 6;
    dcts.dcts_rtc.hour = 12;
    dcts.dcts_rtc.minute = 0;
    dcts.dcts_rtc.second = 0;
    dcts.dcts_pwr = 0.0f;
    dcts.dcts_meas_num = MEAS_NUM;
    dcts.dcts_rele_num = RELE_NUM;
    dcts.dcts_act_num  = ACT_NUM;
    dcts.dcts_alrm_num = ALRM_NUM;

    //meas_channels

    dcts_meas_channel_init(TMPR_IN_1, "Tmpr 1", "Температура 1", "°C", "°C");
    dcts_meas_channel_init(TMPR_IN_2, "Tmpr 2", "Температура 2", "°C", "°C");
    dcts_meas_channel_init(TMPR_IN_AVG, "Tmpr avg", "Температура усред.", "°C", "°C");
    dcts_meas_channel_init(HUM_IN_1, "Hum 1", "Влажность 1", "%", "%");
    dcts_meas_channel_init(HUM_IN_2, "Hum 2", "Влажнсть 2", "%", "%");
    dcts_meas_channel_init(HUM_IN_AVG, "Hum avg", "Влажность усред.", "%", "%");
    dcts_meas_channel_init(TMPR_OUT, "Tmpr out", "Температура снаружи", "°C", "°C");
    dcts_meas_channel_init(HUM_OUT, "Hum out", "Влажность снаружи", "%", "%");
    dcts_meas_channel_init(WTR_MIN_RES, "Water min res", "Верх. уровень сопр.", "Ohm", "Ом");
    dcts_meas_channel_init(WTR_MIN_ADC, "Water min ADC", "Верх. уровень АЦП", "adc", "adc");
    dcts_meas_channel_init(WTR_MIN_VLT, "Water min V", "Верх. уровень В", "V", "В");
    dcts_meas_channel_init(WTR_MAX_RES, "Water max res", "Ниж. уровень сопр.", "Ohm", "Ом");
    dcts_meas_channel_init(WTR_MAX_ADC, "Water max ADC", "Ниж. уровень АЦП", "adc", "adc");
    dcts_meas_channel_init(WTR_MAX_VLT, "Water max V", "Ниж. уровень В", "V", "В");
    dcts_meas_channel_init(VREF_VLT, "Vref V", "Опорное напр. В", "V", "В");
    dcts_meas_channel_init(VBAT_VLT, "RTC battery V", "Батарейка В", "V", "В");

    //act_channels

    dcts_act_channel_init(VALVE_IN, "Valve IN", "Клапан приточный", "%", "%");
    dcts_act_channel_init(VALVE_OUT, "Valve OUT", "Клапан вытяжной", "%", "%");
    dcts_act_channel_init(TMPR_IN_HEATING, "Tmpr IN heating", "Температура нагрев", "°C", "°C");
    dcts_act_channel_init(TMPR_IN_COOLING, "Tmpr IN cooling", "Температура охлаждение", "°C", "°C");
    dcts_act_channel_init(HUM_IN, "Hum IN", "Влажность", "%", "%");
    dcts_act_channel_init(AUTO_PUMP, "Auto Pump", "Автодренаж", "", "");
    dcts_act_channel_init(WTR_MIN_LVL, "Min level detect", "Минимальный уровень", "Ohm", "Ом");
    dcts_act_channel_init(WTR_MAX_LVL, "Max level detect", "Максимальный уровень", "Ohm", "Ом");

    //rele_channels

    dcts_rele_channel_init(FAN_IN, "Fan IN", "Вентилятор приточный");
    dcts_rele_channel_init(HEATER, "Heater", "Нагреватель");
    dcts_rele_channel_init(FREEZER, "Freezer", "Охладитель");
    dcts_rele_channel_init(FAN_CONVECTION, "Convection", "Конвекция");
    dcts_rele_channel_init(WTR_PUMP, "Water pump", "Дренаж");
    dcts_rele_channel_init(RESERV, "Reserv", "Резерв");
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
static void RTC_Init(void){
    time_t unix_time = 0;
    struct tm system_time = {0};
    __HAL_RCC_BKP_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_RCC_RTC_ENABLE();
    hrtc.Instance = RTC;
    hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
    if (HAL_RTC_Init(&hrtc) != HAL_OK) {
        _Error_Handler(__FILE__, __LINE__);
    }

    uint16_t read = read_bkp(0);
    if(read == RTC_KEY){
        dcts.dcts_rtc.state = RTC_STATE_READY;
    }else{
        save_to_bkp(0, RTC_KEY);
        dcts.dcts_rtc.state = RTC_STATE_SET;
    }
    if(dcts.dcts_rtc.state == RTC_STATE_SET){

        system_time.tm_hour = dcts.dcts_rtc.hour;
        system_time.tm_min = dcts.dcts_rtc.minute;
        system_time.tm_sec = dcts.dcts_rtc.second;

        system_time.tm_mday = dcts.dcts_rtc.day;
        system_time.tm_mon = dcts.dcts_rtc.month;
        system_time.tm_year = dcts.dcts_rtc.year - 1900;

        unix_time = mktime(&system_time);

        RTC_write_cnt(unix_time);
    }
    dcts.dcts_rtc.state = RTC_STATE_READY;
}
/**
 * @brief RTC_write_cnt
 * @param cnt_value - time in unix format
 * @return  0 - OK,\n
 *          -1 - timeout error,\n
 *          -2 - timeout error
 */
static int RTC_write_cnt(time_t cnt_value){
    int result = 0;
    u32 start = HAL_GetTick();
    u32 timeout = 0;
    PWR->CR |= PWR_CR_DBP;                                          //разрешить доступ к Backup области
    while ((!(RTC->CRL & RTC_CRL_RTOFF))&&(timeout <= start + 500)){//проверить закончены ли изменения регистров RTC
        osDelay(1);
        timeout++;
    }
    if(timeout > start + 500){
        result = -1;
    }
    RTC->CRL |= RTC_CRL_CNF;                                        //Разрешить Запись в регистры RTC
    RTC->CNTH = (u32)cnt_value>>16;                                 //записать новое значение счетного регистра
    RTC->CNTL = (u32)cnt_value;
    RTC->CRL &= ~RTC_CRL_CNF;                                       //Запретить запись в регистры RTC
    start = HAL_GetTick();
    while ((!(RTC->CRL & RTC_CRL_RTOFF))&&(timeout <= start + 500)){//Дождаться окончания записи
        osDelay(1);
        timeout++;
    }
    if(timeout > start + 500){
        result = -2;
    }
    PWR->CR &= ~PWR_CR_DBP;                                         //запретить доступ к Backup области
    return result;
}
/**
 * @brief RTC task
 * @param argument - None
 * @todo add group
 */
#define RTC_TASK_PERIOD 500
void rtc_task(void const * argument){
    time_t unix_time = 0;
    struct tm system_time = {0};
    (void)argument;
    RTC_Init();
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        switch (dcts.dcts_rtc.state) {
        case RTC_STATE_READY:   //update dcts_rtc from rtc
            unix_time = (time_t)(RTC->CNTL);
            unix_time |= (time_t)(RTC->CNTH<<16);
            system_time = *localtime(&unix_time);

            taskENTER_CRITICAL();
            dcts.dcts_rtc.hour      = (u8)system_time.tm_hour;
            dcts.dcts_rtc.minute    = (u8)system_time.tm_min;
            dcts.dcts_rtc.second    = (u8)system_time.tm_sec;

            dcts.dcts_rtc.day       = (u8)system_time.tm_mday;
            dcts.dcts_rtc.month     = (u8)system_time.tm_mon;
            dcts.dcts_rtc.year      = (u8)system_time.tm_year + 1900;
            dcts.dcts_rtc.weekday   = (u8)system_time.tm_wday;
            taskEXIT_CRITICAL();
            break;
        case RTC_STATE_SET:     //set new values from dcts_rtc

            system_time.tm_hour = dcts.dcts_rtc.hour;
            system_time.tm_min  = dcts.dcts_rtc.minute;
            system_time.tm_sec  = dcts.dcts_rtc.second;

            system_time.tm_mday = dcts.dcts_rtc.day;
            system_time.tm_mon  = dcts.dcts_rtc.month;
            system_time.tm_year = dcts.dcts_rtc.year - 1900;

            unix_time = mktime(&system_time);

            RTC_write_cnt(unix_time);

            dcts.dcts_rtc.state = RTC_STATE_READY;
            break;
        default:
            break;
        }
        refresh_watchdog();
        osDelayUntil(&last_wake_time, RTC_TASK_PERIOD);
    }
}
/**
 * @brief display_task
 * @param argument
 */
#if(DISP == LCD_DISP)
#define display_task_period 100
#elif(DISP == ST7735_DISP)
#define display_task_period 250
#endif // DISP
void display_task(void const * argument){
    (void)argument;
    menu_init();
#if(DISP == LCD_DISP)
    LCD_init();
#elif(DISP == ST7735_DISP)
    LCD.auto_off = config.params.lcd_backlight_time;
    LCD.backlight_lvl = config.params.lcd_backlight_lvl;
    LCD_backlight_timer_init();
    LCD_backlight_on();
    st7735_init();
    st7735_fill_rect(0,0,160,128,ST7735_WHITE);
#endif
    u8 tick = 0;
    u8 tick_2 = 0;
    menu_page_t last_page = selectedMenuItem->Page;
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        refresh_watchdog();
#if(DISP == LCD_DISP)
        LCD_clr();
#elif(DISP == ST7735_DISP)
        if(last_page != selectedMenuItem->Page){
            st7735_fill_rect(0,0,160,128,ST7735_WHITE);
            last_page = selectedMenuItem->Page;
        }
#endif
        if(last_page != selectedMenuItem->Page){
            tick = 0;
            last_page = selectedMenuItem->Page;
        }
        switch (selectedMenuItem->Page) {
        case MAIN_PAGE:
            main_page_print(tick);
            break;
        case INFO:
            info_print();
            break;
        case SAVE_CHANGES:
            save_page_print(tick);
            break;
        default:
            if(selectedMenuItem->Child_num > 0){
                menu_page_print(tick);
            }else if(selectedMenuItem->Child_num == 0){
                value_print(tick);
            }
        }

#if(DISP == LCD_DISP)
        LCD_update();
#elif(DISP == ST7735_DISP)
        //st7735_update();
#endif
        if((LCD.auto_off != 0)&&(LCD.backlight == LCD_BACKLIGHT_ON)){
            LCD.auto_off_timeout += display_task_period;
            if(LCD.auto_off_timeout > (uint32_t)LCD.auto_off * 10000){
                LCD.auto_off_timeout = 0;
                LCD_backlight_shutdown();
            }
        }
        if(tick_2 == 500/display_task_period){
            tick_2 = 0;
            tick++;
        }
        tick_2++;
        osDelayUntil(&last_wake_time, display_task_period);
    }
}

#define BUTTON_PRESS_TIME 1000
#define BUTTON_PRESS_TIMEOUT 10000
#define BUTTON_CLICK_TIME 10
#define BUTTON_DISP_RESET 3000
#define navigation_task_period 20
void navigation_task (void const * argument){
    (void)argument;
    u16 timeout = 0;
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        switch (navigation_style){
        case MENU_NAVIGATION:
            if(button_click(BUTTON_UP, BUTTON_CLICK_TIME)){
                menuChange(selectedMenuItem->Previous);
            }
            if(button_click(BUTTON_DOWN, BUTTON_CLICK_TIME)){
                menuChange(selectedMenuItem->Next);
            }
            if(button_click(BUTTON_LEFT, BUTTON_CLICK_TIME)){
                menuChange(selectedMenuItem->Parent);
            }
            if(button_click(BUTTON_RIGHT, BUTTON_CLICK_TIME)){
                menuChange(selectedMenuItem->Child);
            }
            if(button_click(BUTTON_OK, BUTTON_CLICK_TIME)){
                menuChange(selectedMenuItem->Child);
            }
            break;
        case DIGIT_EDIT:
            switch (selectedMenuItem->Page){
            case TIME_HOUR:
            case TIME_MIN:
            case TIME_SEC:
            case DATE_DAY:
            case DATE_MONTH:
            case DATE_YEAR:
                dcts.dcts_rtc.state = RTC_STATE_EDIT;
                break;
            }
            if(button_click(BUTTON_UP,BUTTON_CLICK_TIME)){
                // increment value
                switch(edit_val.type){
                case VAL_INT8:
                    if(*edit_val.p_val.p_int8 < edit_val.val_max.int8){
                        *edit_val.p_val.p_int8 += (int8_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_int8 > edit_val.val_max.int8)||(*edit_val.p_val.p_int8 < edit_val.val_min.int8)){ //if out of range
                        *edit_val.p_val.p_int8 = edit_val.val_max.int8;
                    }
                    break;
                case VAL_UINT8:
                    if(*edit_val.p_val.p_uint8 < edit_val.val_max.uint8){
                        *edit_val.p_val.p_uint8 += (uint8_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_uint8 > edit_val.val_max.uint8)||(*edit_val.p_val.p_uint8 < edit_val.val_min.uint8)){ //if out of range
                        *edit_val.p_val.p_uint8 = edit_val.val_max.uint8;
                    }
                    break;
                case VAL_INT16:
                    if(*edit_val.p_val.p_int16 < edit_val.val_max.int16){
                        *edit_val.p_val.p_int16 += (int16_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_int16 > edit_val.val_max.int16)||(*edit_val.p_val.p_int16 < edit_val.val_min.int16)){ //if out of range
                        *edit_val.p_val.p_int16 = edit_val.val_max.int16;
                    }
                    break;
                case VAL_UINT16:
                    if(*edit_val.p_val.p_uint16 < edit_val.val_max.uint16){
                        *edit_val.p_val.p_uint16 += (uint16_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_uint16 > edit_val.val_max.uint16)||(*edit_val.p_val.p_uint16 < edit_val.val_min.uint16)){ //if out of range
                        *edit_val.p_val.p_uint16 = edit_val.val_max.uint16;
                    }
                    break;
                case VAL_INT32:
                    if(*edit_val.p_val.p_int32 < edit_val.val_max.int32){
                        *edit_val.p_val.p_int32 += (int32_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_int32 > edit_val.val_max.int32)||(*edit_val.p_val.p_int32 < edit_val.val_min.int32)){ //if out of range
                        *edit_val.p_val.p_int32 = edit_val.val_max.int32;
                    }
                    break;
                case VAL_UINT32:
                    if(*edit_val.p_val.p_uint32 < edit_val.val_max.uint32){
                        *edit_val.p_val.p_uint32 += (uint32_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_uint32 > edit_val.val_max.uint32)||(*edit_val.p_val.p_uint32 < edit_val.val_min.uint32)){ //if out of range
                        *edit_val.p_val.p_uint32 = edit_val.val_max.uint32;
                    }
                    break;
                case VAL_FLOAT:
                    if(*edit_val.p_val.p_float < edit_val.val_max.vfloat){
                        *edit_val.p_val.p_float += float_pow(10.0, edit_val.digit);
                    }
                    if((*edit_val.p_val.p_float > edit_val.val_max.vfloat)||(*edit_val.p_val.p_float < edit_val.val_min.vfloat)){ //if out of range
                        *edit_val.p_val.p_float = edit_val.val_max.vfloat;
                    }
                    break;
                default:
                    break;
                }
            }
            if(button_click(BUTTON_DOWN,BUTTON_CLICK_TIME)){
                // decrement value
                switch(edit_val.type){
                case VAL_INT8:
                    if(*edit_val.p_val.p_int8 > edit_val.val_min.int8){
                        *edit_val.p_val.p_int8 -= (int8_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_int8 > edit_val.val_max.int8)||(*edit_val.p_val.p_int8 < edit_val.val_min.int8)){ //if out of range
                        *edit_val.p_val.p_int8 = edit_val.val_min.int8;
                    }
                    break;
                case VAL_UINT8:
                    if(*edit_val.p_val.p_uint8 > edit_val.val_min.uint8){
                        *edit_val.p_val.p_uint8 -= (uint8_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_uint8 > edit_val.val_max.uint8)||(*edit_val.p_val.p_uint8 < edit_val.val_min.uint8)){ //if out of range
                        *edit_val.p_val.p_uint8 = edit_val.val_min.uint8;
                    }
                    break;
                case VAL_INT16:
                    if(*edit_val.p_val.p_int16 > edit_val.val_min.int16){
                        *edit_val.p_val.p_int16 -= (int16_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_int16 > edit_val.val_max.int16)||(*edit_val.p_val.p_int16 < edit_val.val_min.int16)){ //if out of range
                        *edit_val.p_val.p_int16 = edit_val.val_min.int16;
                    }
                    break;
                case VAL_UINT16:
                    if(*edit_val.p_val.p_uint16 > edit_val.val_min.uint16){
                        *edit_val.p_val.p_uint16 -= (uint16_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_uint16 > edit_val.val_max.uint16)||(*edit_val.p_val.p_uint16 < edit_val.val_min.uint16)){ //if out of range
                        *edit_val.p_val.p_uint16 = edit_val.val_min.uint16;
                    }
                    break;
                case VAL_INT32:
                    if(*edit_val.p_val.p_int32 > edit_val.val_min.int32){
                        *edit_val.p_val.p_int32 -= (int32_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_int32 > edit_val.val_max.int32)||(*edit_val.p_val.p_int32 < edit_val.val_min.int32)){ //if out of range
                        *edit_val.p_val.p_int32 = edit_val.val_min.int32;
                    }
                    break;
                case VAL_UINT32:
                    if(*edit_val.p_val.p_uint32 > edit_val.val_min.uint32){
                        *edit_val.p_val.p_uint32 -= (uint32_t)uint32_pow(10, (uint8_t)edit_val.digit);
                    }
                    if((*edit_val.p_val.p_uint32 > edit_val.val_max.uint32)||(*edit_val.p_val.p_uint32 < edit_val.val_min.uint32)){ //if out of range
                        *edit_val.p_val.p_uint32 = edit_val.val_min.uint32;
                    }
                    break;
                case VAL_FLOAT:
                    if(*edit_val.p_val.p_float > edit_val.val_min.vfloat){
                        *edit_val.p_val.p_float -= float_pow(10.0, edit_val.digit);
                    }
                    if((*edit_val.p_val.p_float > edit_val.val_max.vfloat)||(*edit_val.p_val.p_float < edit_val.val_min.vfloat)){ //if out of range
                        *edit_val.p_val.p_float = edit_val.val_min.vfloat;
                    }
                    break;
                default:
                    break;
                }
            }
            if(button_click(BUTTON_LEFT,BUTTON_CLICK_TIME)){
                //shift position left
                if(edit_val.digit < edit_val.digit_max){
                    edit_val.digit++;
                }
            }
            if(button_click(BUTTON_RIGHT,BUTTON_CLICK_TIME)){
                //shift position right
                if(edit_val.digit > edit_val.digit_min){
                    edit_val.digit--;
                }
            }
            if(button_click(BUTTON_OK,BUTTON_CLICK_TIME)){
                //out from digit_edit mode
                switch (selectedMenuItem->Page){
                case TIME_HOUR:
                case TIME_MIN:
                case TIME_SEC:
                case DATE_DAY:
                case DATE_MONTH:
                case DATE_YEAR:
                    dcts.dcts_rtc.state = RTC_STATE_SET;
                    break;
                }
                navigation_style = MENU_NAVIGATION;
            }
            break;
        }
        if(button_click(BUTTON_BREAK,BUTTON_CLICK_TIME)){
            if(LCD.auto_off == 0){
                LCD_backlight_toggle();
            }
        }
        if(button_click(BUTTON_SET,BUTTON_CLICK_TIME)){
            save_params();
        }
        if(button_clamp(BUTTON_BREAK, BUTTON_DISP_RESET)){
            vTaskSuspend(displayTaskHandle);
            LCD_deinit();
            LCD_init();
            vTaskResume(displayTaskHandle);
        }
        if(button_clamp(BUTTON_LEFT, BUTTON_DISP_RESET)){
            NVIC_SystemReset();
        }
        osDelayUntil(&last_wake_time, navigation_task_period);
    }
}

static void error_page_print(menu_page_t page){
    char string[50];
#if(DISP == LCD_DISP)
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
#elif(DISP == ST7735_DISP)
    st7735_xy(25,45);
    sprintf(string, "СТРАНИЦА НЕ");
    st7735_print(string,&Font_7x10,ST7735_BLACK);
    st7735_xy(25,35);
    sprintf(string, "НАЙДЕНА: %d", page);
    st7735_print(string,&Font_7x10,ST7735_BLACK);

    sprintf(string, "<назад");
    st7735_xy(0,0);
    st7735_fill_rect(0,0,42,11,ST7735_BLACK);
    st7735_print(string,&Font_7x10,ST7735_WHITE);
#endif // DISP
}

static void main_page_print(u8 tick){
    char string[50];

    //convection
#if(DISP == LCD_DISP)
    LCD_set_xy(76,26);
    if(dcts_rele[FAN_CONVECTION].state.control == 0){
        LCD_print_char(6,&Icon_16x16,LCD_COLOR_BLACK);
    }else{
        switch(tick%4){
        case 0:
            LCD_print_char(4,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(21,31);
            LCD_print_char(15,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(96,31);
            LCD_print_char(13,&Icon_16x16,LCD_COLOR_BLACK);

            LCD_set_xy(32,50);
            LCD_print_char(12,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(59,50);
            LCD_print_char(12,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(86,50);
            LCD_print_char(12,&Icon_16x16,LCD_COLOR_BLACK);

            LCD_set_xy(32,12);
            LCD_print_char(14,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(59,12);
            LCD_print_char(14,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(86,12);
            LCD_print_char(14,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 1:
            LCD_print_char(5,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(21,37);
            LCD_print_char(15,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(96,25);
            LCD_print_char(13,&Icon_16x16,LCD_COLOR_BLACK);

            LCD_set_xy(39,50);
            LCD_print_char(12,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(66,50);
            LCD_print_char(12,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(88,50);
            LCD_print_char(12,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_fill_area(88,50,92,64,LCD_COLOR_WHITE);

            LCD_set_xy(28,12);
            LCD_print_char(14,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_fill_area(41,12,51,28,LCD_COLOR_WHITE);
            LCD_set_xy(52,12);
            LCD_print_char(14,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(79,12);
            LCD_print_char(14,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 2:
            LCD_print_char(2,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(21,42);
            LCD_print_char(15,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(21,20);
            LCD_print_char(15,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(96,42);
            LCD_print_char(13,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(96,20);
            LCD_print_char(13,&Icon_16x16,LCD_COLOR_BLACK);

            LCD_set_xy(45,50);
            LCD_print_char(12,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(72,50);
            LCD_print_char(12,&Icon_16x16,LCD_COLOR_BLACK);

            LCD_set_xy(45,12);
            LCD_print_char(14,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(72,12);
            LCD_print_char(14,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 3:
            LCD_print_char(3,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(21,25);
            LCD_print_char(15,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(96,37);
            LCD_print_char(13,&Icon_16x16,LCD_COLOR_BLACK);

            LCD_set_xy(25,50);
            LCD_print_char(12,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_fill_area(25,48,27,64,LCD_COLOR_WHITE);
            LCD_set_xy(52,50);
            LCD_print_char(12,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(79,50);
            LCD_print_char(12,&Icon_16x16,LCD_COLOR_BLACK);

            LCD_set_xy(39,12);
            LCD_print_char(14,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(66,12);
            LCD_print_char(14,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_set_xy(93,12);
            LCD_print_char(14,&Icon_16x16,LCD_COLOR_BLACK);
            LCD_fill_area(104,12,109,28,LCD_COLOR_WHITE);
            break;
        }
    }
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(92,94,17,17,ST7735_WHITE);
    st7735_xy(92,94);
    if(dcts_rele[FAN_CONVECTION].state.control == 0){
        st7735_print_char(6,&Icon_16x16,ST7735_BLACK);
    }else{
        switch(tick%4){
        case 0:
            st7735_print_char(4,&Icon_16x16,ST7735_BLACK);
            /*st7735_xy(21,31);
            st7735_print_char(15,&Icon_16x16,ST7735_BLACK);
            st7735_xy(96,31);
            st7735_print_char(13,&Icon_16x16,ST7735_BLACK);

            st7735_xy(32,50);
            st7735_print_char(12,&Icon_16x16,ST7735_BLACK);
            st7735_xy(59,50);
            st7735_print_char(12,&Icon_16x16,ST7735_BLACK);
            st7735_xy(86,50);
            st7735_print_char(12,&Icon_16x16,ST7735_BLACK);

            st7735_xy(32,12);
            st7735_print_char(14,&Icon_16x16,ST7735_BLACK);
            st7735_xy(59,12);
            st7735_print_char(14,&Icon_16x16,ST7735_BLACK);
            st7735_xy(86,12);
            st7735_print_char(14,&Icon_16x16,ST7735_BLACK);*/
            break;
        case 1:
            st7735_print_char(5,&Icon_16x16,ST7735_BLACK);
            /*st7735_xy(21,37);
            st7735_print_char(15,&Icon_16x16,ST7735_BLACK);
            st7735_xy(96,25);
            st7735_print_char(13,&Icon_16x16,ST7735_BLACK);

            st7735_xy(39,50);
            st7735_print_char(12,&Icon_16x16,ST7735_BLACK);
            st7735_xy(66,50);
            st7735_print_char(12,&Icon_16x16,ST7735_BLACK);
            st7735_xy(88,50);
            st7735_print_char(12,&Icon_16x16,ST7735_BLACK);
            st7735_fill_rect(88,50,4,14,ST7735_WHITE);

            st7735_xy(28,12);
            st7735_print_char(14,&Icon_16x16,ST7735_BLACK);
            st7735_fill_rect(41,12,10,16,ST7735_WHITE);
            st7735_xy(52,12);
            st7735_print_char(14,&Icon_16x16,ST7735_BLACK);
            st7735_xy(79,12);
            st7735_print_char(14,&Icon_16x16,ST7735_BLACK);*/
            break;
        case 2:
            st7735_print_char(2,&Icon_16x16,ST7735_BLACK);
            /*st7735_xy(21,42);
            st7735_print_char(15,&Icon_16x16,ST7735_BLACK);
            st7735_xy(21,20);
            st7735_print_char(15,&Icon_16x16,ST7735_BLACK);
            st7735_xy(96,42);
            st7735_print_char(13,&Icon_16x16,ST7735_BLACK);
            st7735_xy(96,20);
            st7735_print_char(13,&Icon_16x16,ST7735_BLACK);

            st7735_xy(45,50);
            st7735_print_char(12,&Icon_16x16,ST7735_BLACK);
            st7735_xy(72,50);
            st7735_print_char(12,&Icon_16x16,ST7735_BLACK);

            st7735_xy(45,12);
            st7735_print_char(14,&Icon_16x16,ST7735_BLACK);
            st7735_xy(72,12);
            st7735_print_char(14,&Icon_16x16,ST7735_BLACK);*/
            break;
        case 3:
            st7735_print_char(3,&Icon_16x16,ST7735_BLACK);
            /*st7735_xy(21,25);
            st7735_print_char(15,&Icon_16x16,ST7735_BLACK);
            st7735_xy(96,37);
            st7735_print_char(13,&Icon_16x16,ST7735_BLACK);

            st7735_xy(25,50);
            st7735_print_char(12,&Icon_16x16,ST7735_BLACK);
            st7735_fill_rect(25,48,2,16,ST7735_WHITE);
            st7735_xy(52,50);
            st7735_print_char(12,&Icon_16x16,ST7735_BLACK);
            st7735_xy(79,50);
            st7735_print_char(12,&Icon_16x16,ST7735_BLACK);

            st7735_xy(39,12);
            st7735_print_char(14,&Icon_16x16,ST7735_BLACK);
            st7735_xy(66,12);
            st7735_print_char(14,&Icon_16x16,ST7735_BLACK);
            st7735_xy(93,12);
            st7735_print_char(14,&Icon_16x16,ST7735_BLACK);
            st7735_fill_rect(104,12,5,16,ST7735_WHITE);*/
            break;
        }
    }
#endif // DISP

    // temperature and hummidity out
#if(DISP == LCD_DISP)
    LCD_set_xy(0,56);
    if(dcts_meas[TMPR_OUT].valid == 1){
        sprintf(string,"%.1f%s",(double)dcts_meas[TMPR_OUT].value,dcts_meas[TMPR_OUT].unit_cyr);
    }else{
        sprintf(string,"Обрыв");
    }
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
    LCD_set_xy(0,48);
    if(dcts_meas[HUM_OUT].valid == 1){
        sprintf(string,"%.1f%s",(double)dcts_meas[HUM_OUT].value,dcts_meas[HUM_OUT].unit_cyr);
    }else{
        sprintf(string,"датчика");
    }
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK); 
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(0,111,36,17,ST7735_WHITE);
    st7735_xy(0,119);
    if(dcts_meas[TMPR_OUT].valid == 1){
        sprintf(string,"%.1f%s",(double)dcts_meas[TMPR_OUT].value,dcts_meas[TMPR_OUT].unit_cyr);
        st7735_print(string,&Font_5x7,ST7735_BLACK);
    }else{
        sprintf(string,"Обрыв");
        st7735_print(string,&Font_5x7,ST7735_RED);
    }
    st7735_xy(0,111);
    if(dcts_meas[HUM_OUT].valid == 1){
        sprintf(string,"%.1f%s",(double)dcts_meas[HUM_OUT].value,dcts_meas[HUM_OUT].unit_cyr);
        st7735_print(string,&Font_5x7,ST7735_BLACK);
    }else{
        sprintf(string,"датчика");
        st7735_print(string,&Font_5x7,ST7735_RED);
    }
#endif // DISP

    //fan in
#if(DISP == LCD_DISP)
    LCD_set_xy(3,31);
    if(dcts_rele[FAN_IN].state.control == 0){
        LCD_print_char(6,&Icon_16x16,LCD_COLOR_BLACK);
    }else{
        switch(tick%4){
        case 0:
            LCD_print_char(2,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 1:
            LCD_print_char(3,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 2:
            LCD_print_char(4,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 3:
            LCD_print_char(5,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        }
    }
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(3,94,17,17,ST7735_WHITE);
    st7735_xy(3,94);
    if(dcts_rele[FAN_IN].state.control == 0){
        st7735_print_char(6,&Icon_16x16,ST7735_BLACK);
    }else{
        switch(tick%4){
        case 0:
            st7735_print_char(2,&Icon_16x16,ST7735_BLACK);
            break;
        case 1:
            st7735_print_char(3,&Icon_16x16,ST7735_BLACK);
            break;
        case 2:
            st7735_print_char(4,&Icon_16x16,ST7735_BLACK);
            break;
        case 3:
            st7735_print_char(5,&Icon_16x16,ST7735_BLACK);
            break;
        }
    }
#endif // DISP

    //valve in
#if(DISP == LCD_DISP)
    LCD_fill_area(0,21,1,48,LCD_COLOR_BLACK);
    LCD_fill_area(20,21,21,48,LCD_COLOR_BLACK);
    LCD_set_xy(3,12);
    if(dcts_act[VALVE_IN].set_value < 10.0f){
        LCD_print_char(16,&Icon_16x16,LCD_COLOR_BLACK);
    }else if((dcts_act[VALVE_IN].set_value >= 10.0f)&&(dcts_act[VALVE_IN].set_value <= 90.0f)){
        LCD_print_char(17,&Icon_16x16,LCD_COLOR_BLACK);
    }else if(dcts_act[VALVE_IN].set_value > 90.0f){
        LCD_print_char(18,&Icon_16x16,LCD_COLOR_BLACK);
    }
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(1,80,1,31,ST7735_BLACK);
    st7735_fill_rect(20,80,1,31,ST7735_BLACK);
    st7735_fill_rect(3,76,17,17,ST7735_WHITE);
    st7735_xy(3,76);
    if(dcts_act[VALVE_IN].set_value < 10.0f){
        st7735_print_char(16,&Icon_16x16,ST7735_BLACK);
    }else if((dcts_act[VALVE_IN].set_value >= 10.0f)&&(dcts_act[VALVE_IN].set_value <= 90.0f)){
        st7735_print_char(17,&Icon_16x16,ST7735_BLACK);
    }else if(dcts_act[VALVE_IN].set_value > 90.0f){
        st7735_print_char(18,&Icon_16x16,ST7735_BLACK);
    }
#endif // DISP

    //valve out
#if(DISP == LCD_DISP)
    LCD_fill_area(107,52,108,63,LCD_COLOR_BLACK);
    LCD_fill_area(126,52,127,63,LCD_COLOR_BLACK);
    LCD_set_xy(110,43);
    if(dcts_act[VALVE_OUT].set_value < 10.0f){
        LCD_print_char(16,&Icon_16x16,LCD_COLOR_BLACK);
    }else if((dcts_act[VALVE_OUT].set_value >= 10.0f)&&(dcts_act[VALVE_OUT].set_value <= 90.0f)){
        LCD_print_char(17,&Icon_16x16,LCD_COLOR_BLACK);
    }else if(dcts_act[VALVE_OUT].set_value > 90.0f){
        LCD_print_char(18,&Icon_16x16,LCD_COLOR_BLACK);
    }
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(139,80,1,48,ST7735_BLACK);
    st7735_fill_rect(158,80,1,48,ST7735_BLACK);
    st7735_fill_rect(141,76,17,17,ST7735_WHITE);
    st7735_xy(141,76);
    if(dcts_act[VALVE_OUT].set_value < 10.0f){
        st7735_print_char(16,&Icon_16x16,ST7735_BLACK);
    }else if((dcts_act[VALVE_OUT].set_value >= 10.0f)&&(dcts_act[VALVE_OUT].set_value <= 90.0f)){
        st7735_print_char(17,&Icon_16x16,ST7735_BLACK);
    }else if(dcts_act[VALVE_OUT].set_value > 90.0f){
        st7735_print_char(18,&Icon_16x16,ST7735_BLACK);
    }
#endif // DISP

    //water pump
#if(DISP == LCD_DISP)
    LCD_set_xy(94,0);
    if(dcts_rele[WTR_PUMP].state.control == 0){
        LCD_print_char(20,&Icon_16x16,LCD_COLOR_BLACK);
    }else{
        switch(tick%2){
        case 0:
            LCD_print_char(20,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 1:
            LCD_print_char(21,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        }
    }
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(126,0,17,17,ST7735_WHITE);
    st7735_xy(126,0);
    if(dcts_rele[WTR_PUMP].state.control == 0){
        st7735_print_char(20,&Icon_16x16,ST7735_BLACK);
    }else{
        switch(tick%2){
        case 0:
            st7735_print_char(20,&Icon_16x16,ST7735_BLACK);
            break;
        case 1:
            st7735_print_char(21,&Icon_16x16,ST7735_BLACK);
            break;
        }
    }
#endif // DISP

    //water level
#if(DISP == LCD_DISP)
    LCD_set_xy(111,0);
    if(dcts_act[WTR_MIN_LVL].meas_value > dcts_act[WTR_MIN_LVL].hysteresis){
        //empty
        LCD_print_char(22,&Icon_16x16,LCD_COLOR_BLACK);
    }else if((dcts_act[WTR_MIN_LVL].meas_value < dcts_act[WTR_MIN_LVL].hysteresis)&&(dcts_act[WTR_MAX_LVL].meas_value > dcts_act[WTR_MAX_LVL].hysteresis)){
        //min level
        switch(tick%2){
        case 0:
            LCD_print_char(23,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 1:
            LCD_print_char(24,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        }
    }else if((dcts_act[WTR_MIN_LVL].meas_value < dcts_act[WTR_MIN_LVL].hysteresis)&&(dcts_act[WTR_MAX_LVL].meas_value < dcts_act[WTR_MAX_LVL].hysteresis)){
        //max level
        switch(tick%2){
        case 0:
            LCD_print_char(25,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 1:
            LCD_print_char(26,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        }
    }
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(143,0,17,17,ST7735_WHITE);
    st7735_xy(143,0);
    if(dcts_meas[WTR_MIN_RES].value > config.params.wtr_min_ref){
        //empty
        st7735_print_char(22,&Icon_16x16,ST7735_BLACK);
    }else if((dcts_meas[WTR_MIN_RES].value < config.params.wtr_min_ref)&&(dcts_meas[WTR_MAX_RES].value > config.params.wtr_max_ref)){
        //min level
        switch(tick%2){
        case 0:
            st7735_print_char(23,&Icon_16x16,ST7735_BLACK);
            break;
        case 1:
            st7735_print_char(24,&Icon_16x16,ST7735_BLACK);
            break;
        }
    }else if((dcts_meas[WTR_MIN_RES].value < config.params.wtr_min_ref)&&(dcts_meas[WTR_MAX_RES].value < config.params.wtr_max_ref)){
        //max level
        switch(tick%2){
        case 0:
            st7735_print_char(25,&Icon_16x16,ST7735_BLACK);
            break;
        case 1:
            st7735_print_char(26,&Icon_16x16,ST7735_BLACK);
            break;
        }
    }
#endif // DISP

    //heating
#if(DISP == LCD_DISP)
    LCD_set_xy(40,26);
    if(dcts_rele[HEATER].state.control == 1){
        switch(tick%4){
        case 0:
            LCD_print_char(7,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 1:
            LCD_print_char(8,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 2:
            LCD_print_char(9,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        case 3:
            LCD_print_char(10,&Icon_16x16,LCD_COLOR_BLACK);
            break;
        }
    }
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(52,94,17,17,ST7735_WHITE);
    st7735_xy(52,94);
    if(dcts_rele[HEATER].state.control == 1){
        switch(tick%4){
        case 0:
            st7735_print_char(7,&Icon_16x16,ST7735_RED);
            break;
        case 1:
            st7735_print_char(8,&Icon_16x16,ST7735_RED);
            break;
        case 2:
            st7735_print_char(9,&Icon_16x16,ST7735_RED);
            break;
        case 3:
            st7735_print_char(10,&Icon_16x16,ST7735_RED);
            break;
        }
    }
#endif // DISP

    //cooling
#if(DISP == LCD_DISP)
    LCD_set_xy(58,26);
    if(dcts_rele[FREEZER].state.control == 1){
        LCD_print_char(11,&Icon_16x16,LCD_COLOR_BLACK);
    }
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(72,94,17,17,ST7735_WHITE);
    st7735_xy(72,94);
    if(dcts_rele[FREEZER].state.control == 1){
        st7735_print_char(11,&Icon_16x16,ST7735_BLUE);
    }
#endif // DISP

    //temperature and hummidity in
#if(DISP == LCD_DISP)
    if(dcts_meas[TMPR_IN_AVG].valid == 1){
        if((dcts_act[TMPR_IN_HEATING].state.control == 1)||(dcts_act[TMPR_IN_COOLING].state.control == 1)){
            if((dcts_act[TMPR_IN_HEATING].state.control == 1)&&(dcts_act[TMPR_IN_COOLING].state.control == 1)){
                        sprintf(string,"T %.1f%s (ошибка)",(double)dcts_meas[TMPR_IN_AVG].value,dcts_meas[TMPR_IN_AVG].unit_cyr);
            }else if(dcts_act[TMPR_IN_HEATING].state.control == 1){
                sprintf(string,"T %.1f%s (%.1f%s)",(double)dcts_meas[TMPR_IN_AVG].value,dcts_meas[TMPR_IN_AVG].unit_cyr,(double)dcts_act[TMPR_IN_HEATING].set_value,dcts_meas[TMPR_IN_AVG].unit_cyr);
            }else if(dcts_act[TMPR_IN_COOLING].state.control == 1){
                sprintf(string,"T %.1f%s (%.1f%s)",(double)dcts_meas[TMPR_IN_AVG].value,dcts_meas[TMPR_IN_AVG].unit_cyr,(double)dcts_act[TMPR_IN_COOLING].set_value,dcts_meas[TMPR_IN_AVG].unit_cyr);
            }
        }else{
            sprintf(string,"T %.1f%s",(double)dcts_meas[TMPR_IN_AVG].value,dcts_meas[TMPR_IN_AVG].unit_cyr);
        }
    }else{
        sprintf(string,"Обрыв  обоих  ");
    }
    LCD_set_xy(align_text_right(string,Font_5x7)-36,7);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);

    if(dcts_meas[HUM_IN_AVG].valid == 1){
        if(dcts_act[HUM_IN].state.control == 1){
            sprintf(string,"Rh %.1f%s (%.1f%s)",(double)dcts_act[HUM_IN].meas_value,dcts_meas[HUM_IN_AVG].unit_cyr,(double)dcts_act[HUM_IN].set_value,dcts_meas[HUM_IN_AVG].unit_cyr);
        }else{
            sprintf(string,"Rh %.1f%s",(double)dcts_meas[HUM_IN_AVG].value,dcts_meas[HUM_IN_AVG].unit_cyr);
        }
    }else{
        sprintf(string,"датчиков    ");
    }
    LCD_set_xy(align_text_right(string,Font_5x7)-36,0);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(0,0,94,17,ST7735_WHITE);
    if(dcts_meas[TMPR_IN_AVG].valid == 1){
        if((dcts_act[TMPR_IN_HEATING].state.control == 1)||(dcts_act[TMPR_IN_COOLING].state.control == 1)){
            if((dcts_act[TMPR_IN_HEATING].state.control == 1)&&(dcts_act[TMPR_IN_COOLING].state.control == 1)){
                        sprintf(string,"T %.1f%s (ошибка)",(double)dcts_meas[TMPR_IN_AVG].value,dcts_meas[TMPR_IN_AVG].unit_cyr);
            }else if(dcts_act[TMPR_IN_HEATING].state.control == 1){
                sprintf(string,"T %.1f%s (%.1f%s)",(double)dcts_meas[TMPR_IN_AVG].value,dcts_meas[TMPR_IN_AVG].unit_cyr,(double)dcts_act[TMPR_IN_HEATING].set_value,dcts_meas[TMPR_IN_AVG].unit_cyr);
            }else if(dcts_act[TMPR_IN_COOLING].state.control == 1){
                sprintf(string,"T %.1f%s (%.1f%s)",(double)dcts_meas[TMPR_IN_AVG].value,dcts_meas[TMPR_IN_AVG].unit_cyr,(double)dcts_act[TMPR_IN_COOLING].set_value,dcts_meas[TMPR_IN_AVG].unit_cyr);
            }
        }else{
            sprintf(string,"T %.1f%s",(double)dcts_meas[TMPR_IN_AVG].value,dcts_meas[TMPR_IN_AVG].unit_cyr);
        }
    }else{
        sprintf(string,"Обрыв  обоих  ");
    }
    st7735_xy(align_text_right(string,Font_5x7)-36,7);
    st7735_print(string,&Font_5x7,ST7735_BLACK);

    if(dcts_meas[HUM_IN_AVG].valid == 1){
        if(dcts_act[HUM_IN].state.control == 1){
            sprintf(string,"Rh %.1f%s (%.1f%s)",(double)dcts_act[HUM_IN].meas_value,dcts_meas[HUM_IN_AVG].unit_cyr,(double)dcts_act[HUM_IN].set_value,dcts_meas[HUM_IN_AVG].unit_cyr);
        }else{
            sprintf(string,"Rh %.1f%s",(double)dcts_meas[HUM_IN_AVG].value,dcts_meas[HUM_IN_AVG].unit_cyr);
        }
    }else{
        sprintf(string,"датчиков    ");
    }
    st7735_xy(align_text_right(string,Font_5x7)-36,0);
    st7735_print(string,&Font_5x7,ST7735_BLACK);
#endif // DISP

    //time
#if(DISP == LCD_DISP)
    sprintf(string,"%02d:%02d:%02d",dcts.dcts_rtc.hour,dcts.dcts_rtc.minute,dcts.dcts_rtc.second);
    LCD_set_xy(align_text_center(string,Font_5x7),45);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(16,45,129,27,ST7735_WHITE);
    sprintf(string,"%02d:%02d:%02d",dcts.dcts_rtc.hour,dcts.dcts_rtc.minute,dcts.dcts_rtc.second);
    st7735_xy(align_text_center(string,Font_16x26)+16,45);
    st7735_print(string,&Font_16x26,ST7735_GREEN);
#endif // DISP
}

static void print_header(void){
    char string[50];
    //print header
#if(DISP == LCD_DISP)
    menuItem* temp = selectedMenuItem->Parent;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),52);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(0,53,127,63);
#elif(DISP == ST7735_DISP)
    menuItem* temp = selectedMenuItem->Parent;
    sprintf(string, temp->Text);
    st7735_fill_rect(0,116,160,12,ST7735_ORANGE);
    st7735_xy(align_text_center(string, Font_7x10)+16,116);
    st7735_print(string,&Font_7x10,ST7735_WHITE);
#endif // DISP
}

static void menu_page_print(u8 tick){
    char string[50];
    print_header();

    menuItem* temp = selectedMenuItem->Parent;
#if(DISP == LCD_DISP)
    if(temp->Child_num >= 3){
        //print previous
        temp = selectedMenuItem->Previous;
        sprintf(string, temp->Text);
        LCD_set_xy(align_text_center(string, Font_7x10),39);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    }

    //print selected
    sprintf(string, selectedMenuItem->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_invert_area(5,26,120,38);
    LCD_invert_area(6,27,119,37);

    //print next
    temp = selectedMenuItem->Next;
    sprintf(string, temp->Text);
    LCD_set_xy(align_text_center(string, Font_7x10),14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
#elif(DISP == ST7735_DISP)
    uint8_t line_pointer = 0;

    sprintf(string, selectedMenuItem->Text);
    st7735_fill_rect(5,102,150,13,ST7735_BLUE);
    st7735_xy(align_text_center(string, Font_7x10)+16,102);
    st7735_print(string,&Font_7x10,ST7735_WHITE);
    line_pointer++;

    menuItem* next = selectedMenuItem->Next;
    while((line_pointer < temp->Child_num)&&(line_pointer < DISP_MAX_LINES)){
        sprintf(string, next->Text);
        st7735_xy(align_text_center(string, Font_7x10)+16,102-13*line_pointer);
        st7735_print(string,&Font_7x10,ST7735_BLACK);
        next = next->Next;
        line_pointer++;
    }
#endif // DISP

    print_back();
    print_enter_right();
}

static void value_print(u8 tick){
    char string[50];
    print_header();
    int prev = 0;
    int cur = 0;
    int next = 0;

    menuItem* temp = selectedMenuItem->Parent;
#if(DISP == LCD_DISP)
    if(temp->Child_num >= 3){
        //print previous name
        temp = selectedMenuItem->Previous;
        sprintf(string, temp->Text);
        LCD_set_xy(2,39);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        LCD_fill_area(80,39,127,49,LCD_COLOR_WHITE);
        prev = get_param_value(string, temp->Page);
        LCD_set_xy(align_text_right(string,Font_7x10),39);
        LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
        if(prev == -2){
            // invalid value
            LCD_fill_area(84,45,127,45,LCD_COLOR_BLACK);
        }
    }

    //print selected name
    sprintf(string, selectedMenuItem->Text);
    LCD_set_xy(2,26);
    LCD_print_ticker(string,&Font_7x10,LCD_COLOR_BLACK,11,tick);
    cur = get_param_value(string, selectedMenuItem->Page);
    LCD_set_xy(align_text_right(string,Font_7x10),26);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    if(cur == -2){
        // invalid value
        LCD_fill_area(84,32,127,32,LCD_COLOR_BLACK);
    }

    LCD_invert_area(0,26,82,38);
    LCD_invert_area(1,27,81,37);

    //print next name
    temp = selectedMenuItem->Next;
    sprintf(string, temp->Text);
    LCD_set_xy(2,14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_fill_area(80,14,127,24,LCD_COLOR_WHITE);
    next = get_param_value(string, temp->Page);
    LCD_set_xy(align_text_right(string,Font_7x10),14);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    if(next == -2){
        // invalid value
        LCD_fill_area(84,20,127,20,LCD_COLOR_BLACK);
    }
#elif(DISP == ST7735_DISP)
    uint8_t line_pointer = 0;

    sprintf(string, selectedMenuItem->Text);
    st7735_fill_rect(0,102,96,13,ST7735_BLUE);
    st7735_xy(2,102);
    st7735_print(string,&Font_7x10,ST7735_WHITE);
    cur = get_param_value(string, selectedMenuItem->Page);
    st7735_fill_rect(96,102,64,11,ST7735_WHITE);
    st7735_xy(align_text_right(string,Font_7x10)+32,102);
    if(cur == -2){
        // invalid value
        st7735_print(string,&Font_7x10,ST7735_RED);
    }else{
        st7735_print(string,&Font_7x10,ST7735_BLACK);
    }
    line_pointer++;

    menuItem* next_page = selectedMenuItem->Next;
    while((line_pointer < temp->Child_num)&&(line_pointer < DISP_MAX_LINES)){
        sprintf(string, next_page->Text);
        st7735_xy(2,102-13*line_pointer);
        st7735_print(string,&Font_7x10,ST7735_BLACK);
        cur = get_param_value(string, next_page->Page);
        st7735_fill_rect(96,102-13*line_pointer,64,11,ST7735_WHITE);
        st7735_xy(align_text_right(string,Font_7x10)+32,102-13*line_pointer);
        if(cur == -2){
            // invalid value
            st7735_print(string,&Font_7x10,ST7735_RED);
        }else{
            st7735_print(string,&Font_7x10,ST7735_BLACK);
        }
        next_page = next_page->Next;
        line_pointer++;
    }

    st7735_fill_rect(0,0,160,9,ST7735_WHITE);
#endif // DISP
    print_back();

    if(navigation_style == MENU_NAVIGATION){
        set_edit_value(selectedMenuItem->Page);
        if((selectedMenuItem->Child == &EDITED_VAL)&&(cur != -3)){
            print_change();
        }
    }else if(navigation_style == DIGIT_EDIT){
        print_enter_ok();
#if(DISP == LCD_DISP)
        if(edit_val.digit < 0){
            LCD_invert_area(127-(u8)(edit_val.digit+edit_val.select_shift)*edit_val.select_width,26,127-(u8)(edit_val.digit+edit_val.select_shift-1)*edit_val.select_width,38);
        }else{
            LCD_invert_area(127-(u8)(edit_val.digit+edit_val.select_shift+1)*edit_val.select_width,26,127-(u8)(edit_val.digit+edit_val.select_shift)*edit_val.select_width,38);
        }
#elif(DISP == ST7735_DISP)
        if(edit_val.digit < 0){
            st7735_fill_rect(159-(u8)(edit_val.digit+edit_val.select_shift)*edit_val.select_width,102,edit_val.select_width,1,ST7735_BLACK);
        }else{
            st7735_fill_rect(159-(u8)(edit_val.digit+edit_val.select_shift+1)*edit_val.select_width,102,edit_val.select_width,1,ST7735_BLACK);
        }
#endif // DISP
    }
}

/**
 * @brief get_param_value
 * @param string - buffer for set value
 * @param page -
 * @return  0 - haven't additional data,\n
 *          -1 - valid value,\n
 *          -2 - invalid value,\n
 *          -3 - don't change,
 */
static int get_param_value(char* string, menu_page_t page){
    int result = 0;
    switch (page) {
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
    case MEAS_CH_13:
    case MEAS_CH_14:
    case MEAS_CH_15:
        sprintf(string, "%.1f", (double)dcts_meas[(uint8_t)(page - MEAS_CH_0)].value);//, dcts_meas[(uint8_t)(page - MEAS_CH_0)].unit_cyr);
        if(dcts_meas[(uint8_t)(page - MEAS_CH_0)].valid == 1){
            result = -1;
        }else{
            result = -2;
        }
        break;

    case ACT_EN_0:
    case ACT_EN_1:
    case ACT_EN_2:
    case ACT_EN_3:
    case ACT_EN_4:
    case ACT_EN_5:
        sprintf(string, "%s", off_on_descr[dcts_act[(uint8_t)(page - ACT_EN_0)/5].state.control]);
        break;

    case ACT_SET_0:
    case ACT_SET_1:
    case ACT_SET_2:
    case ACT_SET_3:
    case ACT_SET_4:
    case ACT_SET_5:
        sprintf(string, "%.1f%s", (double)dcts_act[(uint8_t)(page - ACT_EN_0)/5].set_value, dcts_act[(uint8_t)(page - ACT_EN_0)/5].unit_cyr);
        break;

    case ACT_HYST_0:
    case ACT_HYST_1:
    case ACT_HYST_2:
    case ACT_HYST_3:
    case ACT_HYST_4:
    case ACT_HYST_5:
        sprintf(string, "%.1f%s", (double)dcts_act[(uint8_t)(page - ACT_HYST_0)/5].hysteresis, dcts_act[(uint8_t)(page - ACT_HYST_0)/5].unit_cyr);
        break;

    case ACT_CUR_0:
    case ACT_CUR_1:
    case ACT_CUR_2:
    case ACT_CUR_3:
    case ACT_CUR_4:
    case ACT_CUR_5:
        sprintf(string, "%.1f%s", (double)dcts_act[(uint8_t)(page - ACT_CUR_0)/5].meas_value, dcts_act[(uint8_t)(page - ACT_CUR_0)/5].unit_cyr);
        break;

    case WTR_MIN_REF:
        sprintf(string, "%.1f", (double)dcts_act[WTR_MIN_LVL].hysteresis);// config.params.wtr_min_ref);
        break;
    case WTR_MAX_REF:
        sprintf(string, "%.1f", (double)dcts_act[WTR_MAX_LVL].hysteresis);// config.params.wtr_max_ref);
        break;

    case RELE_AUTO_MAN_0:
    case RELE_AUTO_MAN_1:
    case RELE_AUTO_MAN_2:
    case RELE_AUTO_MAN_3:
    case RELE_AUTO_MAN_4:
    case RELE_AUTO_MAN_5:
        sprintf(string, "%s", manual_auto_descr[dcts_rele[(uint8_t)(page - RELE_AUTO_MAN_0)/3].state.control_by_act]);
        break;

    case RELE_CONTROL_0:
    case RELE_CONTROL_1:
    case RELE_CONTROL_2:
    case RELE_CONTROL_3:
    case RELE_CONTROL_4:
    case RELE_CONTROL_5:
        sprintf(string, "%s", off_on_descr[dcts_rele[(uint8_t)(page - RELE_CONTROL_0)/3].state.control]);
        if(dcts_rele[(uint8_t)(page - RELE_CONTROL_0)/3].state.control_by_act == 1){
            result = -3;
        }
        break;

    case MDB_ADDR:
        sprintf(string, "%d", dcts.dcts_address);
        break;
    case MDB_BITRATE:
        sprintf(string, "%d", bitrate_array[bitrate_array_pointer]*100);
        break;
    case MDB_RECIEVED:
        sprintf(string, "%d", uart_1.recieved_cnt);
        break;
    case MDB_SENT:
        sprintf(string, "%d", uart_1.send_cnt);
        break;
    case MDB_OVERRUN_ERR:
        sprintf(string, "%d", uart_1.overrun_err_cnt);
        break;
    case MDB_PARITY_ERR:
        sprintf(string, "%d", uart_1.parity_err_cnt);
        break;
    case MDB_FRAME_ERR:
        sprintf(string, "%d", uart_1.frame_err_cnt);
        break;
    case MDB_NOISE_ERR:
        sprintf(string, "%d", uart_1.noise_err_cnt);
        break;

    case LIGHT_LVL:
        sprintf(string, "%d%%", LCD.backlight_lvl*10);
        LCD_backlight_timer_init();
        LCD_backlight_on();
        break;
    case AUTO_OFF:
        sprintf(string, "%dс", LCD.auto_off*10);
        break;

    case TIME_HOUR:
        sprintf(string, "%02d", dcts.dcts_rtc.hour);
        break;
    case TIME_MIN:
        sprintf(string, "%02d", dcts.dcts_rtc.minute);
        break;
    case TIME_SEC:
        sprintf(string, "%02d", dcts.dcts_rtc.second);
        break;
    case DATE_DAY:
        sprintf(string, "%02d", dcts.dcts_rtc.day);
        break;
    case DATE_MONTH:
        sprintf(string, "%02d", dcts.dcts_rtc.month);
        break;
    case DATE_YEAR:
        sprintf(string, "%04d", dcts.dcts_rtc.year);
        break;
    }
    return result;
}

static void set_edit_value(menu_page_t page){
    switch(page){
    case ACT_EN_0:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_act[VALVE_IN].state.control;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*5;
        break;
    case ACT_EN_1:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_act[VALVE_OUT].state.control;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*5;
        break;
    case ACT_EN_2:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_act[TMPR_IN_HEATING].state.control;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*5;
        break;
    case ACT_EN_3:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_act[TMPR_IN_COOLING].state.control;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*5;
        break;
    case ACT_EN_4:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_act[HUM_IN].state.control;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*5;
        break;
    case ACT_EN_5:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_act[AUTO_PUMP].state.control;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*5;
        break;
    case ACT_SET_0:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[VALVE_IN].set_value;
        edit_val.select_shift = 3;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case ACT_SET_1:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[VALVE_OUT].set_value;
        edit_val.select_shift = 3;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case ACT_SET_2:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[TMPR_IN_HEATING].set_value;
        edit_val.select_shift = 4;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case ACT_SET_3:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[TMPR_IN_COOLING].set_value;
        edit_val.select_shift = 4;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case ACT_SET_4:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[HUM_IN].set_value;
        edit_val.select_shift = 3;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case ACT_SET_5:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[AUTO_PUMP].set_value;
        edit_val.select_shift = 3;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case ACT_HYST_0:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[VALVE_IN].hysteresis;
        edit_val.select_shift = 3;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case ACT_HYST_1:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[VALVE_OUT].hysteresis;
        edit_val.select_shift = 3;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case ACT_HYST_2:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[TMPR_IN_HEATING].hysteresis;
        edit_val.select_shift = 4;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case ACT_HYST_3:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[TMPR_IN_COOLING].hysteresis;
        edit_val.select_shift = 4;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case ACT_HYST_4:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[HUM_IN].hysteresis;
        edit_val.select_shift = 3;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case ACT_HYST_5:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 2;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 100.0;
        edit_val.p_val.p_float = &dcts_act[AUTO_PUMP].hysteresis;
        edit_val.select_shift = 3;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case WTR_MIN_REF:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 3;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 9999.0;
        edit_val.p_val.p_float = &dcts_act[WTR_MIN_LVL].hysteresis;
        edit_val.select_shift = 2;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case WTR_MAX_REF:
        edit_val.type = VAL_FLOAT;
        edit_val.digit_max = 3;
        edit_val.digit_min = -1;
        edit_val.digit = 0;
        edit_val.val_min.vfloat = 0.0;
        edit_val.val_max.vfloat = 9999.0;
        edit_val.p_val.p_float = &dcts_act[WTR_MAX_LVL].hysteresis;
        edit_val.select_shift = 2;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case RELE_AUTO_MAN_0:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_rele[FAN_IN].state.control_by_act;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*6;
        break;
    case RELE_AUTO_MAN_1:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_rele[HEATER].state.control_by_act;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*6;
        break;
    case RELE_AUTO_MAN_2:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_rele[FREEZER].state.control_by_act;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*6;
        break;
    case RELE_AUTO_MAN_3:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_rele[FAN_CONVECTION].state.control_by_act;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*6;
        break;
    case RELE_AUTO_MAN_4:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_rele[WTR_PUMP].state.control_by_act;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*6;
        break;
    case RELE_AUTO_MAN_5:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 1;
        edit_val.p_val.p_uint8 = &dcts_rele[RESERV].state.control_by_act;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*6;
        break;
    case RELE_CONTROL_0:
        if(dcts_rele[FAN_IN].state.control_by_act == 0){
            edit_val.type = VAL_UINT8;
            edit_val.digit_max = 0;
            edit_val.digit_min = 0;
            edit_val.digit = 0;
            edit_val.val_min.uint8 = 0;
            edit_val.val_max.uint8 = 1;
            edit_val.p_val.p_uint8 = &dcts_rele[FAN_IN].state.control;
            edit_val.select_shift = 0;
            edit_val.select_width = Font_7x10.FontWidth*5;
        }
        break;
    case RELE_CONTROL_1:
        if(dcts_rele[HEATER].state.control_by_act == 0){
            edit_val.type = VAL_UINT8;
            edit_val.digit_max = 0;
            edit_val.digit_min = 0;
            edit_val.digit = 0;
            edit_val.val_min.uint8 = 0;
            edit_val.val_max.uint8 = 1;
            edit_val.p_val.p_uint8 = &dcts_rele[HEATER].state.control;
            edit_val.select_shift = 0;
            edit_val.select_width = Font_7x10.FontWidth*5;
        }
        break;
    case RELE_CONTROL_2:
        if(dcts_rele[FREEZER].state.control_by_act == 0){
            edit_val.type = VAL_UINT8;
            edit_val.digit_max = 0;
            edit_val.digit_min = 0;
            edit_val.digit = 0;
            edit_val.val_min.uint8 = 0;
            edit_val.val_max.uint8 = 1;
            edit_val.p_val.p_uint8 = &dcts_rele[FREEZER].state.control;
            edit_val.select_shift = 0;
            edit_val.select_width = Font_7x10.FontWidth*5;
        }
        break;
    case RELE_CONTROL_3:
        if(dcts_rele[FAN_CONVECTION].state.control_by_act == 0){
            edit_val.type = VAL_UINT8;
            edit_val.digit_max = 0;
            edit_val.digit_min = 0;
            edit_val.digit = 0;
            edit_val.val_min.uint8 = 0;
            edit_val.val_max.uint8 = 1;
            edit_val.p_val.p_uint8 = &dcts_rele[FAN_CONVECTION].state.control;
            edit_val.select_shift = 0;
            edit_val.select_width = Font_7x10.FontWidth*5;
        }
        break;
    case RELE_CONTROL_4:
        if(dcts_rele[WTR_PUMP].state.control_by_act == 0){
            edit_val.type = VAL_UINT8;
            edit_val.digit_max = 0;
            edit_val.digit_min = 0;
            edit_val.digit = 0;
            edit_val.val_min.uint8 = 0;
            edit_val.val_max.uint8 = 1;
            edit_val.p_val.p_uint8 = &dcts_rele[WTR_PUMP].state.control;
            edit_val.select_shift = 0;
            edit_val.select_width = Font_7x10.FontWidth*5;
        }
        break;
    case RELE_CONTROL_5:
        if(dcts_rele[RESERV].state.control_by_act == 0){
            edit_val.type = VAL_UINT8;
            edit_val.digit_max = 0;
            edit_val.digit_min = 0;
            edit_val.digit = 0;
            edit_val.val_min.uint8 = 0;
            edit_val.val_max.uint8 = 1;
            edit_val.p_val.p_uint8 = &dcts_rele[RESERV].state.control;
            edit_val.select_shift = 0;
            edit_val.select_width = Font_7x10.FontWidth*5;
        }
        break;
    case MDB_ADDR:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 2;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 255;
        edit_val.p_val.p_uint8 = &dcts.dcts_address;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case MDB_BITRATE:
        edit_val.type = VAL_UINT16;
        edit_val.digit_max = 0;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint16 = 0;
        edit_val.val_max.uint16 = 13;
        edit_val.p_val.p_uint16 = &bitrate_array_pointer;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth*6;
        break;
    case LIGHT_LVL:
        edit_val.type = VAL_UINT16;
        edit_val.digit_max = 1;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint16 = 1;
        edit_val.val_max.uint16 = 10;
        edit_val.p_val.p_uint16 = &LCD.backlight_lvl;
        edit_val.select_shift = 2;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case AUTO_OFF:
        edit_val.type = VAL_UINT16;
        edit_val.digit_max = 1;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint16 = 1;
        edit_val.val_max.uint16 = 60;
        edit_val.p_val.p_uint16 = &LCD.auto_off;
        edit_val.select_shift = 2;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case TIME_HOUR:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 1;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 23;
        edit_val.p_val.p_uint8 = &dcts.dcts_rtc.hour;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case TIME_MIN:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 1;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 59;
        edit_val.p_val.p_uint8 = &dcts.dcts_rtc.minute;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case TIME_SEC:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 1;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 0;
        edit_val.val_max.uint8 = 59;
        edit_val.p_val.p_uint8 = &dcts.dcts_rtc.second;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case DATE_DAY:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 1;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 1;
        edit_val.val_max.uint8 = 31;
        edit_val.p_val.p_uint8 = &dcts.dcts_rtc.day;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case DATE_MONTH:
        edit_val.type = VAL_UINT8;
        edit_val.digit_max = 1;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint8 = 1;
        edit_val.val_max.uint8 = 12;
        edit_val.p_val.p_uint8 = &dcts.dcts_rtc.month;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    case DATE_YEAR:
        edit_val.type = VAL_UINT16;
        edit_val.digit_max = 3;
        edit_val.digit_min = 0;
        edit_val.digit = 0;
        edit_val.val_min.uint16 = 2000;
        edit_val.val_max.uint16 = 3000;
        edit_val.p_val.p_uint16 = &dcts.dcts_rtc.year;
        edit_val.select_shift = 0;
        edit_val.select_width = Font_7x10.FontWidth;
        break;
    }
}


static void info_print (void){
    char string[50];
    print_header();
#if(DISP == LCD_DISP)
    sprintf(string, "Имя:%s",dcts.dcts_name_cyr);
    LCD_set_xy(2,44);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
    sprintf(string, "Адрес:%d",dcts.dcts_address);
    LCD_set_xy(2,36);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
    sprintf(string, "Версия:%s",dcts.dcts_ver);
    LCD_set_xy(2,28);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
    sprintf(string, "Питание:%.1fВ",(double)dcts.dcts_pwr);
    LCD_set_xy(2,20);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
    sprintf(string, "Батарейка:%.1fВ",(double)dcts_meas[VBAT_VLT].value);
    LCD_set_xy(2,12);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
    sprintf(string, "%02d:%02d:%02d", dcts.dcts_rtc.hour, dcts.dcts_rtc.minute, dcts.dcts_rtc.second);
    LCD_set_xy(70,44);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
    sprintf(string, "%02d.%02d.%04d", dcts.dcts_rtc.day, dcts.dcts_rtc.month, dcts.dcts_rtc.year);
    LCD_set_xy(70,36);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
#elif(DISP == ST7735_DISP)
    sprintf(string, "Имя:%s",dcts.dcts_name_cyr);
    st7735_xy(2,44);
    st7735_print(string,&Font_5x7,ST7735_BLACK);
    sprintf(string, "Адрес:%d",dcts.dcts_address);
    st7735_xy(2,36);
    st7735_print(string,&Font_5x7,ST7735_BLACK);
    sprintf(string, "Версия:%s",dcts.dcts_ver);
    st7735_xy(2,28);
    st7735_print(string,&Font_5x7,ST7735_BLACK);
    sprintf(string, "Питание:%.1fВ",(double)dcts.dcts_pwr);
    st7735_xy(2,20);
    st7735_print(string,&Font_5x7,ST7735_BLACK);
    sprintf(string, "Батарейка:%.1fВ",(double)dcts_meas[VBAT_VLT].value);
    st7735_xy(2,12);
    st7735_print(string,&Font_5x7,ST7735_BLACK);
    sprintf(string, "%02d:%02d:%02d", dcts.dcts_rtc.hour, dcts.dcts_rtc.minute, dcts.dcts_rtc.second);
    st7735_xy(70,44);
    st7735_print(string,&Font_5x7,ST7735_BLACK);
    sprintf(string, "%02d.%02d.%04d", dcts.dcts_rtc.day, dcts.dcts_rtc.month, dcts.dcts_rtc.year);
    st7735_xy(70,36);
    st7735_print(string,&Font_5x7,ST7735_BLACK);
#endif // DISP

    print_back();
}

static void save_page_print (u8 tick){
    char string[50];

#if(DISP == LCD_DISP)
    LCD_fill_area(5,30,123,58,LCD_COLOR_BLACK);
    LCD_fill_area(6,31,122,57,LCD_COLOR_WHITE);
    sprintf(string, "СОХРАНЕНИЕ НОВЫХ");
    LCD_set_xy(align_text_center(string, Font_7x10),42);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    sprintf(string, "КОЭФФИЦИЕНТОВ");
    LCD_set_xy(align_text_center(string, Font_7x10),32);
    LCD_print(string,&Font_7x10,LCD_COLOR_BLACK);
    LCD_set_xy(55,6);
    LCD_print_char(1,&Icon_16x16,LCD_COLOR_BLACK);
    switch(tick%4){
    case 0:
        LCD_fill_area(55,10,71,22,LCD_COLOR_WHITE);
        break;
    case 1:
        LCD_fill_area(55,14,71,22,LCD_COLOR_WHITE);
        break;
    case 2:
        LCD_fill_area(55,18,71,22,LCD_COLOR_WHITE);
        break;
    }
#elif(DISP == ST7735_DISP)
    st7735_fill_rect(5,30,118,28,ST7735_BLACK);
    st7735_fill_rect(6,31,116,26,ST7735_WHITE);
    sprintf(string, "СОХРАНЕНИЕ НОВЫХ");
    st7735_xy(align_text_center(string, Font_7x10),42);
    st7735_print(string,&Font_7x10,ST7735_BLACK);
    sprintf(string, "КОЭФФИЦИЕНТОВ");
    st7735_xy(align_text_center(string, Font_7x10),32);
    st7735_print(string,&Font_7x10,ST7735_BLACK);
    st7735_xy(55,6);
    st7735_print(1,&Icon_16x16,ST7735_BLACK);
    switch(tick%4){
    case 0:
        st7735_fill_rect(55,10,16,12,ST7735_BLACK);
        break;
    case 1:
        st7735_fill_rect(55,14,16,8,ST7735_BLACK);
        break;
    case 2:
        st7735_fill_rect(55,18,16,4,ST7735_BLACK);
        break;
    }
#endif // DISP
}

#define control_task_period 5
void control_task(void const * argument){
    (void)argument;
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    static pump_st_t pump_state = PUMP_EMPTY;
    static t_heat_t t_heat = T_HEAT_HEATING;
    static t_cool_t t_cool = T_COOL_COOLING;
    static uint16_t valve_tick = 0;
    double last_valve_in_state = (double)dcts_act[VALVE_IN].set_value;
    double last_valve_out_state = (double)dcts_act[VALVE_OUT].set_value;
    #define VALVE_PWM_RUN_TIME  1*1000
    #define VALVE_CTRL_PERIOD   60*1000
    channels_init();
    channel_PWM_timer_init(5);
    channel_PWM_timer_init(6);
    uint32_t last_wake_time = osKernelSysTick();
    while(1){

        // input valve
        if((dcts_act[VALVE_IN].state.control)&&(valve_tick < VALVE_PWM_RUN_TIME)){
            channel_PWM_duty_set(5, (dcts_act[VALVE_IN].set_value*0.05f+5.0f));
            HAL_TIM_PWM_Start(&htim3, input_ch[5].pwm_channel);
        }else {
            HAL_TIM_PWM_Stop(&htim3, input_ch[5].pwm_channel);
            HAL_GPIO_WritePin(input_ch[5].port, input_ch[5].pin, GPIO_PIN_RESET);
        }

        // output valve
        if((dcts_act[VALVE_OUT].state.control)&&(valve_tick < VALVE_PWM_RUN_TIME)){
            channel_PWM_duty_set(6, (dcts_act[VALVE_OUT].set_value*0.05f+5.0f));
            HAL_TIM_PWM_Start(&htim3, input_ch[6].pwm_channel);
        }else {
            HAL_TIM_PWM_Stop(&htim3, input_ch[6].pwm_channel);
            HAL_GPIO_WritePin(input_ch[6].port, input_ch[6].pin, GPIO_PIN_RESET);
        }
        valve_tick += control_task_period;
        if(valve_tick >= VALVE_CTRL_PERIOD){
            valve_tick = 0;
        }

        // set new valves value
        if((fabs(last_valve_in_state - (double)dcts_act[VALVE_IN].set_value) > (double)0.1f)||
                (fabs(last_valve_out_state - (double)dcts_act[VALVE_OUT].set_value) > (double)0.1f)){
            valve_tick = 0;
            last_valve_in_state = (double)dcts_act[VALVE_IN].set_value;
            last_valve_out_state = (double)dcts_act[VALVE_OUT].set_value;
        }

        // temperature in (heating)
        if(dcts_act[TMPR_IN_HEATING].state.control){
            if(dcts_meas[TMPR_IN_AVG].valid){
                dcts_act[TMPR_IN_HEATING].meas_value = dcts_meas[TMPR_IN_AVG].value;

                switch(t_heat){
                case T_HEAT_HEATING:
                    dcts_act[TMPR_IN_HEATING].state.pin_state = 1;
                    if(dcts_act[TMPR_IN_HEATING].meas_value >= (dcts_act[TMPR_IN_HEATING].set_value + 0.5f*dcts_act[TMPR_IN_HEATING].hysteresis)){
                        t_heat = T_HEAT_COOLING;
                    }
                    break;
                case T_HEAT_COOLING:
                    dcts_act[TMPR_IN_HEATING].state.pin_state = 0;
                    if(dcts_act[TMPR_IN_HEATING].meas_value <= (dcts_act[TMPR_IN_HEATING].set_value - 0.5f*dcts_act[TMPR_IN_HEATING].hysteresis)){
                        t_heat = T_HEAT_HEATING;
                    }
                    break;
                }
            }else{
                // current value unknown
                dcts_act[TMPR_IN_HEATING].state.pin_state = 0;
            }
            // set rele_control if control_by_act enable
            if(dcts_rele[HEATER].state.control_by_act == 1){
                if(dcts_rele[HEATER].state.control != dcts_act[TMPR_IN_HEATING].state.pin_state){
                    dcts_rele[HEATER].state.control = dcts_act[TMPR_IN_HEATING].state.pin_state;
                }
            }
        }else{
            // disable rele_control if control_by_act enable
            if(dcts_rele[HEATER].state.control_by_act == 1){
                dcts_rele[HEATER].state.control = 0;
            }
        }

        // temperature in (cooling)
        if(dcts_act[TMPR_IN_COOLING].state.control){
            if((dcts_meas[TMPR_IN_AVG].valid)&&(dcts_meas[TMPR_OUT].valid)&&
                    (dcts_meas[TMPR_OUT].value < dcts_meas[TMPR_IN_AVG].value)){
                dcts_act[TMPR_IN_COOLING].meas_value = dcts_meas[TMPR_IN_AVG].value;
                switch(t_cool){
                case T_COOL_COOLING:
                    dcts_act[TMPR_IN_COOLING].state.pin_state = 1;
                    dcts_act[VALVE_IN].set_value = 100.0f;
                    dcts_act[VALVE_OUT].set_value = 100.0f;
                    if(dcts_act[TMPR_IN_COOLING].meas_value <= (dcts_act[TMPR_IN_COOLING].set_value - 0.5f*dcts_act[TMPR_IN_COOLING].hysteresis)){
                        t_cool = T_COOL_HEATING;
                    }
                    break;
                case T_COOL_HEATING:
                    dcts_act[TMPR_IN_COOLING].state.pin_state = 0;
                    dcts_act[VALVE_IN].set_value = 0.0f;
                    dcts_act[VALVE_OUT].set_value = 0.0f;
                    if(dcts_act[TMPR_IN_COOLING].meas_value >= (dcts_act[TMPR_IN_COOLING].set_value + 0.5f*dcts_act[TMPR_IN_COOLING].hysteresis)){
                        t_cool = T_COOL_COOLING;
                    }
                    break;
                }
            }else{
                // current value unknown
                dcts_act[TMPR_IN_COOLING].state.pin_state = 0;
                dcts_act[VALVE_IN].set_value = 0.0f;
                dcts_act[VALVE_OUT].set_value = 0.0f;
            }
            // set rele_control if control_by_act enable
            if(dcts_rele[FAN_IN].state.control_by_act == 1){
                if(dcts_rele[FAN_IN].state.control != dcts_act[TMPR_IN_COOLING].state.pin_state)
                dcts_rele[FAN_IN].state.control = dcts_act[TMPR_IN_COOLING].state.pin_state;
            }
        }else{
            // disable rele_control if control_by_act enable
            if(dcts_rele[FAN_IN].state.control_by_act == 1){
                dcts_rele[FAN_IN].state.control = 0;
            }
        }

        // hummidity in
        if(dcts_act[HUM_IN].state.control){
            if(dcts_meas[HUM_IN_AVG].valid){
                dcts_act[HUM_IN].meas_value = dcts_meas[HUM_IN_AVG].value;
            }else{
                // current value unknown
                dcts_act[HUM_IN].state.pin_state = 0;
            }
        }

        // water min
        if(dcts_meas[WTR_MIN_RES].valid){
            dcts_act[WTR_MIN_LVL].meas_value = dcts_meas[WTR_MIN_RES].value;
            if(dcts_act[WTR_MIN_LVL].meas_value < dcts_act[WTR_MIN_LVL].hysteresis){
                dcts_act[WTR_MIN_LVL].state.pin_state = 1;
            }else{
                dcts_act[WTR_MIN_LVL].state.pin_state = 0;
            }
        }

        // water max
        if(dcts_meas[WTR_MAX_RES].valid){
            dcts_act[WTR_MAX_LVL].meas_value = dcts_meas[WTR_MAX_RES].value;
            if(dcts_act[WTR_MAX_LVL].meas_value < dcts_act[WTR_MAX_LVL].hysteresis){
                dcts_act[WTR_MAX_LVL].state.pin_state = 1;
            }else{
                dcts_act[WTR_MAX_LVL].state.pin_state = 0;
            }
        }

        // water pump
        if(dcts_act[AUTO_PUMP].state.control){
            switch(pump_state){
            case PUMP_EMPTY:
                if(dcts_act[WTR_MIN_LVL].state.pin_state == 1){
                    pump_state = PUMP_FILLING;
                }
                break;
            case PUMP_FILLING:
                if(dcts_act[WTR_MIN_LVL].state.pin_state == 0){
                    pump_state = PUMP_EMPTY;
                }else if(dcts_act[WTR_MAX_LVL].state.pin_state == 1){
                    pump_state = PUMP_ACTIVE;
                }
                break;
            case PUMP_ACTIVE:
                dcts_act[WTR_PUMP].state.pin_state = 1;
                if(dcts_act[WTR_MAX_LVL].state.pin_state == 0){
                    dcts_act[WTR_PUMP].state.pin_state = 0;
                    pump_state = PUMP_EMPTY;
                }
                break;
            }
            if(dcts_rele[WTR_PUMP].state.control_by_act == 1){
                if(dcts_rele[WTR_PUMP].state.control != dcts_act[WTR_PUMP].state.pin_state)
                dcts_rele[WTR_PUMP].state.control = dcts_act[WTR_PUMP].state.pin_state;
            }
        }

        // control DO channels
        for(u8 i = 0; i < DO_NUM; i ++){
            if(dcts_rele[i].state.control == 1){
                //rele on
                if(dcts_rele[i].state.status == 0){
                    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
                    GPIO_InitStruct.Pin = do_ch[i].pin;
                    HAL_GPIO_Init (do_ch[i].port, &GPIO_InitStruct);
                    HAL_GPIO_WritePin(do_ch[i].port, do_ch[i].pin, GPIO_PIN_RESET);
                    dcts_rele[i].state.status = 1;
                }
            }else{
                //rele off
                if(dcts_rele[i].state.status == 1){
                    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                    GPIO_InitStruct.Pin = do_ch[i].pin;
                    HAL_GPIO_Init (do_ch[i].port, &GPIO_InitStruct);
                    dcts_rele[i].state.status = 0;
                }
            }
        }
        osDelayUntil(&last_wake_time, control_task_period);
    }
}

/**
 * @brief Init timer for LCD backlight level control
 * @param channel - channel number
 * @return  0 - OK,\n
 *          -1 - TIM init error,\n
 *          -2 - TIM syncronyzation config error,\n
 *          -3 - PWM channel config error
 * @ingroup LCD
 */
static int channel_PWM_timer_init(u8 channel){
    int result = 0;
    TIM_MasterConfigTypeDef sMasterConfig = {0};
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();

    if(input_ch[channel].pwm_tim == TIM3){
        htim3.Instance = TIM3;
        htim3.Init.Prescaler = 719;
        htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim3.Init.Period = 2000;
        htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
        htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
        {
          result = -1;
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
        {
          result = -2;
        }
    }/*else if(input_ch[channel].pwm_tim == TIM2){
        htim2.Instance = TIM2;
        htim2.Init.Prescaler = 71;
        htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
        htim2.Init.Period = 100;
        htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
        if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
        {
          result = -1;
        }
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
        if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
        {
          result = -2;
        }
    }*/
    if(channel_PWM_duty_set(channel, 0) < 0){
        result = -3;
    }
    return result;
}

static int channel_PWM_duty_set(u8 channel, float duty){
    int result = 0;
    TIM_OC_InitTypeDef sConfigOC = {0};

    if(input_ch[channel].pwm_tim == TIM3){
        htim3.Instance = TIM3;
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = (uint16_t)(duty * 20.0f);
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, input_ch[channel].pwm_channel) != HAL_OK)
        {
          result = -1;
        }
    }/*else if(input_ch[channel].pwm_tim == TIM2){
        htim2.Instance = TIM2;
        sConfigOC.OCMode = TIM_OCMODE_PWM1;
        sConfigOC.Pulse = (uint16_t)(duty * 10.0f);
        sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
        sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
        if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, input_ch[channel].pwm_channel) != HAL_OK)
        {
          result = -1;
        }
    }*/
    return result;
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
void tim2_init(void){
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

static void print_back(void){
    char string[100];
#if(DISP == LCD_DISP)
    sprintf(string, "<назад");
    LCD_set_xy(0,0);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
    LCD_invert_area(0,0,30,8);
#elif(DISP == ST7735_DISP)
    sprintf(string, "<назад");
    st7735_fill_rect(0,0,32,9,ST7735_BLACK);
    st7735_xy(0,0);
    st7735_print(string,&Font_5x7,ST7735_WHITE);
#endif // DISP
}

static void print_enter_right(void){
    char string[100];
#if(DISP == LCD_DISP)
    sprintf(string, "выбор>");
    LCD_set_xy(align_text_right(string,Font_5x7),0);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
    LCD_invert_area(97,0,127,8);
#elif(DISP == ST7735_DISP)
    sprintf(string, "выбор>");
    st7735_fill_rect(128,0,32,9,ST7735_BLACK);
    st7735_xy(align_text_right(string,Font_5x7)+32,0);
    st7735_print(string,&Font_5x7,ST7735_WHITE);
#endif // DISP
}

static void print_enter_ok(void){
    char string[100];
#if(DISP == LCD_DISP)
    sprintf(string, "ввод*");
    LCD_set_xy(align_text_center(string,Font_5x7),0);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
    LCD_invert_area(51,0,76,8);
#elif(DISP == ST7735_DISP)
    sprintf(string, "ввод*");
    st7735_fill_rect(66,0,27,9,ST7735_BLACK);
    st7735_xy(align_text_center(string,Font_5x7)+16,0);
    st7735_print(string,&Font_5x7,ST7735_WHITE);
#endif // DISP
}

static void print_change(void){
    char string[100];
#if(DISP == LCD_DISP)
    sprintf(string, "изменить>");
    LCD_set_xy(align_text_right(string,Font_5x7),0);
    LCD_print(string,&Font_5x7,LCD_COLOR_BLACK);
    LCD_invert_area(82,0,127,8);
#elif(DISP == ST7735_DISP)
    sprintf(string, "изменить>");
    st7735_fill_rect(113,0,47,9,ST7735_BLACK);
    st7735_xy(align_text_right(string,Font_5x7)+32,0);
    st7735_print(string,&Font_5x7,ST7735_WHITE);
#endif // DISP
}

static void save_params(void){
    saved_to_flash_t config;
    static menuItem* current_menu;
    current_menu = selectedMenuItem;
    menuChange(&save_changes);

    // store ModBus params
    config.params.mdb_address = dcts.dcts_address;
    config.params.mdb_bitrate = (uint16_t)bitrate_array[bitrate_array_pointer];
    // store display params
    config.params.lcd_backlight_lvl = LCD.backlight_lvl;
    config.params.lcd_backlight_time = LCD.auto_off;
    // store dcts_act
    for(uint8_t i = 0; i < ACT_NUM; i++){
        config.params.act_set[i] = dcts_act[i].set_value;
        config.params.act_hyst[i] = dcts_act[i].hysteresis;
        config.params.act_enable[i] = dcts_act[i].state.control;
    }
    // store dcts_rele
    for(uint8_t i = 0; i < RELE_NUM; i++){
        config.params.rele[i] = dcts_rele[i].state.control_by_act;
    }

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
    // reinit uart
    uart_deinit();
    uart_1.bitrate = (uint16_t)bitrate_array[bitrate_array_pointer];
    uart_init(uart_1.bitrate, 8, 1, PARITY_NONE, 10000, UART_CONN_LOST_TIMEOUT);
    //delay for show message
    osDelay(2000);
    menuChange(current_menu);
}

static void restore_params(void){
    saved_to_flash_t config;
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

        // restore ModBus params
        dcts.dcts_address = (uint8_t)config.params.mdb_address;
        uart_1.bitrate = config.params.mdb_bitrate;
        // restore display params
        LCD.backlight_lvl = config.params.lcd_backlight_lvl;
        LCD.auto_off = config.params.lcd_backlight_time;
        // restore dcts_act
        for(uint8_t i = 0; i < ACT_NUM; i++){
            dcts_act[i].set_value = config.params.act_set[i];
            dcts_act[i].hysteresis = config.params.act_hyst[i];
            dcts_act[i].state.control = (uint8_t)config.params.act_enable[i];
        }

        // restore dcts_rele
        for(uint8_t i = 0; i < RELE_NUM; i++){
            dcts_rele[i].state.control_by_act = (uint8_t)(config.params.rele[i]);
        }
    }else{
        //init default values if saved params not found
        dcts.dcts_address = 0x0B;
        uart_1.bitrate = BITRATE_115200;
        LCD.backlight_lvl = 1;
        LCD.auto_off = 3;
        for(uint8_t i = 0; i < ACT_NUM; i++){
            dcts_act[i].set_value = 0.0f;
            dcts_act[i].hysteresis = 0.0f;
            dcts_act[i].state.control = 0;
        }
        for(uint8_t i = 0; i < RELE_NUM; i++){
            dcts_rele[i].state.control_by_act = 0;
        }
    }
    for(bitrate_array_pointer = 0; bitrate_array_pointer < 14; bitrate_array_pointer++){
        if(bitrate_array[bitrate_array_pointer] == uart_1.bitrate){
            break;
        }
    }
}

static void save_to_bkp(u8 bkp_num, uint16_t var){
    uint32_t data = var;
    HAL_PWR_EnableBkUpAccess();
    switch (bkp_num){
    case 0:
        BKP->DR1 = data;
        break;
    case 1:
        BKP->DR2 = data;
        break;
    case 2:
        BKP->DR3 = data;
        break;
    case 3:
        BKP->DR4 = data;
        break;
    case 4:
        BKP->DR5 = data;
        break;
    case 5:
        BKP->DR6 = data;
        break;
    case 6:
        BKP->DR7 = data;
        break;
    case 7:
        BKP->DR8 = data;
        break;
    case 8:
        BKP->DR9 = data;
        break;
    case 9:
        BKP->DR10 = data;
        break;
    }
    HAL_PWR_DisableBkUpAccess();
}

/*static void save_float_to_bkp(u8 bkp_num, float var){
    char buf[5] = {0};
    sprintf(buf, "%4.0f", (double)var);
    u8 data = (u8)atoi(buf);
    save_to_bkp(bkp_num, data);
}*/

static uint16_t read_bkp(u8 bkp_num){
    uint32_t data = 0;
    switch (bkp_num){
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
    return (uint16_t)(data & 0xFFFF);
}
/*static float read_float_bkp(u8 bkp_num, u8 sign){
    u8 data = read_bkp(bkp_num);
    char buf[5] = {0};
    if(sign == READ_FLOAT_SIGNED){
        sprintf(buf, "%d", (s8)data);
    }else{
        sprintf(buf, "%d", data);
    }
    return atoff(buf);
}*/


static void channels_init(void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    for(u8 i = 0;i < DO_NUM; i++){
        GPIO_InitStruct.Pin = do_ch[i].pin;
        //HAL_GPIO_WritePin(do_ch[i].port, do_ch[i].pin, GPIO_PIN_SET);
        HAL_GPIO_Init (do_ch[i].port, &GPIO_InitStruct);
    }
    for(u8 i = 0; i < IN_CHANNEL_NUM; i++){
        switch (input_ch[i].mode){
        case CH_MODE_ADC:
            GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Pin = input_ch[i].pin;
            HAL_GPIO_Init(input_ch[i].port, &GPIO_InitStruct);
            break;
        case CH_MODE_DI:
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            GPIO_InitStruct.Pin = input_ch[i].pin;
            HAL_GPIO_Init(input_ch[i].port, &GPIO_InitStruct);
            break;
        case CH_MODE_DO:
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            GPIO_InitStruct.Pin = input_ch[i].pin;
            HAL_GPIO_Init(input_ch[i].port, &GPIO_InitStruct);
            break;
        case CH_MODE_AM3202:
            GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
            GPIO_InitStruct.Pull = GPIO_PULLUP;
            GPIO_InitStruct.Pin = input_ch[i].pin;
            HAL_GPIO_Init(input_ch[i].port, &GPIO_InitStruct);
            break;
        case CH_MODE_PWM:
            GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
            GPIO_InitStruct.Pull = GPIO_PULLDOWN;
            GPIO_InitStruct.Pin = input_ch[i].pin;
            HAL_GPIO_Init(input_ch[i].port, &GPIO_InitStruct);
            break;
        case CH_MODE_DS18B20:
            GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
            GPIO_InitStruct.Pull = GPIO_NOPULL;
            GPIO_InitStruct.Pin = input_ch[i].pin;
            HAL_GPIO_Init(input_ch[i].port, &GPIO_InitStruct);
            break;
        default:
            ;
        }
    }
}

static void MX_IWDG_Init(void){

    hiwdg.Instance = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
    hiwdg.Init.Reload = 3124;   //10sec
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
    {
      Error_Handler();
    }
}

uint32_t uint32_pow(uint16_t x, uint8_t pow){
    uint32_t result = 1;
    while(pow){
        result *= x;
        pow--;
    }
    return result;
}

float float_pow(float x, int pow){
    float result = 1.0;
    if(pow > 0){
        while(pow > 0){
            result *= x;
            pow--;
        }
    }else if(pow < 0){
        while(pow < 0){
            result /= x;
            pow++;
        }
    }
    return  result;
}

void refresh_watchdog(void){
#if(RELEASE == 1)
    MX_IWDG_Init();
#endif//RELEASE
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

