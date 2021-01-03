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

#define SAVED_PARAMS_SIZE 21

#if(SAVED_PARAMS_SIZE > SAVE_AREA_SIZE)
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

typedef enum{
    MENU_NAVIGATION,
    DIGIT_EDIT,
}navigation_t;

typedef struct{
    uint16_t * p_val;
    uint16_t val_min;
    uint16_t val_max;
    uint8_t digit;
    uint8_t digit_max;
}edit_val_t;

typedef union{
    struct{
        uint16_t lvl_calib_table[6];
        uint16_t tmpr_calib_table[11];
        uint16_t mdb_address;
        uint16_t mdb_bitrate;
        uint16_t lcd_backlight_lvl;
        uint16_t lcd_backlight_time;
    }params;
    uint16_t word[SAVED_PARAMS_SIZE];
}saved_to_flash_t;

void _Error_Handler(char *, int);
extern uint32_t us_cnt_H;
extern navigation_t navigation_style;
extern edit_val_t edit_val;
extern RTC_HandleTypeDef hrtc;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern osThreadId defaultTaskHandle;
extern osThreadId buttonsTaskHandle;
extern osThreadId displayTaskHandle;
extern osThreadId menuTaskHandle;
extern osThreadId controlTaskHandle;
extern osThreadId adcTaskHandle;
extern osThreadId am2302TaskHandle;
extern osThreadId navigationtTaskHandle;
extern osThreadId uartTaskHandle;
extern saved_to_flash_t config;

void display_task(void const * argument);
void am2302_task(void const * argument);
void default_task(void const * argument);
void navigation_task(void const * argument);
void uart_task(void const * argument);

uint32_t us_tim_get_value(void);
void us_tim_delay(uint32_t us);
uint16_t uint16_pow(uint16_t x, uint16_t pow);


#endif /* __MAIN_H__ */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
