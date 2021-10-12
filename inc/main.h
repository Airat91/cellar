/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H__
#define __MAIN_H__

#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stdlib.h"
#include "cmsis_os.h"
#include "uart.h"
#include "flash.h"


#define AIR_PORT GPIOA
#define AIR_PIN  LL_GPIO_PIN_7
#define FLOW_PORT GPIOA
#define FLOW_PIN  LL_GPIO_PIN_5
#define LIGTH_PORT GPIOA
#define LIGTH_PIN  LL_GPIO_PIN_4
#define LIGTH2_PORT GPIOA
#define LIGTH2_PIN  LL_GPIO_PIN_3
#define	_DS18B20_GPIO  GPIOB
#define	_DS18B20_PIN   GPIO_PIN_15
#define I2C1_SCL_PORT GPIOB
#define I2C1_SCL  GPIO_PIN_6
#define I2C1_SDA_PORT GPIOB
#define I2C1_SDA  GPIO_PIN_7
#define ADC0_PIN  GPIO_PIN_0
#define ADC1_PIN  GPIO_PIN_1
#define ADC_PORT GPIOA
#define STEP_OUT1_1 LL_GPIO_PIN_13
#define STEP_OUT1_2 LL_GPIO_PIN_14
#define STEP_OUT2_1 LL_GPIO_PIN_11
#define STEP_OUT2_2 LL_GPIO_PIN_12
#define STEP_PORT GPIOB

#define MEAS_NUM 16
#define ACT_NUM 8
#define RELE_NUM 6
#define ARRAY_NUM 2
#define SAVED_PARAMS_SIZE 50
#if(STM32F103xB == 1)
#define BKP_REG_NUM 10
#elif(STM32F103x8 == 1)
#define BKP_REG_NUM 5
#endif

#define LCD_DISP 1
#define ST7735_DISP 2
#define DISP LCD_DISP // LCD_DISP or ST7735_DISP
#define DISP_MAX_LINES  8

#if(SAVED_PARAMS_SIZE > SAVE_AREA_SIZE/2)
    #error(SAVED_PARAMS_SIZE > SAVE_AREA_SIZE)
#endif

#define TIME_YIELD_THRESHOLD 100


/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

#ifdef __cplusplus
 extern "C" {
#endif


#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

 typedef enum {
     TMPR_IN_1 = 0,
     TMPR_IN_2,
     TMPR_IN_AVG,
     HUM_IN_1,
     HUM_IN_2,
     HUM_IN_AVG,
     TMPR_OUT,
     HUM_OUT,
     WTR_MIN_RES,
     WTR_MIN_ADC,
     WTR_MIN_VLT,
     WTR_MAX_RES,
     WTR_MAX_ADC,
     WTR_MAX_VLT,
     VREF_VLT,
     VBAT_VLT,
 }dcts_meas_t;

 typedef enum {
     VALVE_IN = 0,
     VALVE_OUT,
     TMPR_IN_HEATING,
     TMPR_IN_COOLING,
     HUM_IN,
     AUTO_PUMP,
     WTR_MIN_LVL,
     WTR_MAX_LVL,
 }dcts_act_t;

 typedef enum {
     FAN_IN = 0,
     HEATER,
     FREEZER,
     FAN_CONVECTION,
     WTR_PUMP,
     RESERV,
 }dcts_rele_t;

typedef enum{
    MENU_NAVIGATION,
    DIGIT_EDIT,
}navigation_t;

typedef enum{
    CH_VALVE_IN = 0,
    CH_VALVE_OUT,
    CH_AM2302_OUT,
    CH_AM2302_IN_1,
    CH_AM2302_IN_2,
    CH_VALVE_IN_PWM,
    CH_VALVE_OUT_PWM,
    CH_RESERVED,
}ch_list;

typedef enum{
    DO_FAN_IN = 0,
    DO_HEATER,
    DO_FREEZER,
    DO_FAN_CONVECTION,
    DO_WTR_PUMP,
    DO_RESERVED,
}do_list;

 typedef enum{
     VAL_UNKNOWN = 0,
     VAL_UINT8,
     VAL_INT8,
     VAL_UINT16,
     VAL_INT16,
     VAL_UINT32,
     VAL_INT32,
     VAL_FLOAT,
 }edit_val_type;

 typedef union{
     uint8_t * p_uint8;
     int8_t * p_int8;
     uint16_t * p_uint16;
     int16_t * p_int16;
     uint32_t * p_uint32;
     int32_t * p_int32;
     float * p_float;
 }edit_val_p_type_t;

 typedef union{
     uint8_t uint8;
     int8_t int8;
     uint16_t uint16;
     int16_t int16;
     uint32_t uint32;
     int32_t int32;
     float vfloat;
 }edit_val_type_t;

 typedef struct{
     edit_val_p_type_t p_val;
     edit_val_type_t val_min;
     edit_val_type_t val_max;
     int8_t digit;
     int8_t digit_min;
     int8_t digit_max;
     edit_val_type type;
     uint8_t select_width;
     uint8_t select_shift;
 }edit_val_t;

typedef union{
    struct{
        uint16_t mdb_address;
        uint16_t mdb_bitrate;
        uint16_t lcd_backlight_lvl;
        uint16_t lcd_backlight_time;
        uint16_t act_enable[ACT_NUM];
        float    act_set[ACT_NUM];
        float    act_hyst[ACT_NUM];
        uint16_t rele[RELE_NUM];
    }params;
    uint16_t word[SAVED_PARAMS_SIZE];
}saved_to_flash_t;

typedef struct{
    GPIO_TypeDef * port;
    uint16_t pin;
}ch_t;

typedef enum{
    CH_MODE_NONE = 0,
    CH_MODE_ADC,
    CH_MODE_DI,
    CH_MODE_DO,
    CH_MODE_AM3202,
    CH_MODE_PWM,
    CH_MODE_DS18B20,
}ch_mode_t;

typedef enum{
    PUMP_EMPTY = 0,
    PUMP_FILLING,
    PUMP_ACTIVE,
}pump_st_t;

typedef enum{
    T_HEAT_HEATING = 0,
    T_HEAT_COOLING,
}t_heat_t;

 typedef enum{
     T_COOL_COOLING = 0,
     T_COOL_HEATING,
 }t_cool_t;

typedef struct{
    ch_mode_t mode;
    GPIO_TypeDef * port;
    uint16_t pin;
    ADC_TypeDef * adc_num;
    uint32_t adc_channel;
    TIM_TypeDef * pwm_tim;
    uint32_t pwm_channel;
}in_channel_t;

typedef union{
    uint8_t u8_data[BKP_REG_NUM*2];
    uint16_t u16_data[BKP_REG_NUM];
}bkp_data_t;

typedef enum{
    BKP_DATA_U8 = 0,
    BKP_DATA_U16,
}bkp_data_type;

void _Error_Handler(char *, int);
extern uint32_t us_cnt_H;
extern navigation_t navigation_style;
extern edit_val_t edit_val;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef hpwmtim;
extern osThreadId rtcTaskHandle;
extern osThreadId buttonsTaskHandle;
extern osThreadId displayTaskHandle;
extern osThreadId menuTaskHandle;
extern osThreadId controlTaskHandle;
extern osThreadId adcTaskHandle;
extern osThreadId am2302TaskHandle;
extern osThreadId ds18TaskHandle;
extern osThreadId navigationtTaskHandle;
extern osThreadId uartTaskHandle;
//extern saved_to_flash_t config;
extern const ch_t do_ch[];
extern const ch_t ch[];
extern const in_channel_t input_ch[];
extern bkp_data_t *bkp_data_p;
extern IWDG_HandleTypeDef hiwdg;

void display_task(void const * argument);
void am2302_task(void const * argument);
void rtc_task(void const * argument);
void navigation_task(void const * argument);
void uart_task(void const * argument);
void control_task(void const * argument);
void tim2_init(void);
//void ds18_task(void const * argument);

uint32_t us_tim_get_value(void);
void us_tim_delay(uint32_t us);
uint32_t uint32_pow(uint16_t x, uint8_t pow);
uint16_t uint16_pow(uint16_t x, uint16_t pow);
float float_pow(float x, int pow);
void refresh_watchdog(void);


#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
