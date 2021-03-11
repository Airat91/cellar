#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#define CH_0_PORT       GPIOA
#define CH_0_PIN        GPIO_PIN_0
#define CH_1_PORT       GPIOA
#define CH_1_PIN        GPIO_PIN_1
#define CH_2_PORT       GPIOA
#define CH_2_PIN        GPIO_PIN_2
#define CH_3_PORT       GPIOA
#define CH_3_PIN        GPIO_PIN_3
#define PWR_PORT        GPIOA
#define PWR_PIN         GPIO_PIN_4
#define DO_1_PORT       GPIOA
#define DO_1_PIN        GPIO_PIN_5
#define CH_4_PORT       GPIOA
#define CH_4_PIN        GPIO_PIN_6
#define CH_5_PORT       GPIOA
#define CH_5_PIN        GPIO_PIN_7
#define OK_PORT         GPIOA
#define OK_PIN          GPIO_PIN_8
#define RS_485_TX_PORT  GPIOA
#define RS_485_TX_PIN   GPIO_PIN_9
#define RS_485_RX_PORT  GPIOA
#define RS_485_RX_PIN   GPIO_PIN_10
#define RS_485_DE_PORT  GPIOA
#define RS_485_DE_PIN   GPIO_PIN_11
#define DO_5_PORT       GPIOA
#define DO_5_PIN        GPIO_PIN_12
#define DEBUG_TMS_PORT  GPIOA
#define DEBUG_TMS_PIN   GPIO_PIN_13
#define DEBUG_TCK_PORT  GPIOA
#define DEBUG_TCK_PIN   GPIO_PIN_14
#define LCD_CS_PORT     GPIOA
#define LCD_CS_PIN      GPIO_PIN_15

#define CH_6_PORT       GPIOB
#define CH_6_PIN        GPIO_PIN_0
#define CH_7_PORT       GPIOB
#define CH_7_PIN        GPIO_PIN_1

#define LCD_SCK_PORT    GPIOB
#define LCD_SCK_PIN     GPIO_PIN_3
#define LCD_RST_PORT    GPIOB
#define LCD_RST_PIN     GPIO_PIN_4
#define LCD_MOSI_PORT   GPIOB
#define LCD_MOSI_PIN    GPIO_PIN_5
#define LCD_LIGHT_PORT  GPIOB
#define LCD_LIGHT_PIN   GPIO_PIN_6
#define DO_4_PORT       GPIOB
#define DO_4_PIN        GPIO_PIN_7
#define DO_3_PORT       GPIOB
#define DO_3_PIN        GPIO_PIN_8
#define DO_2_PORT       GPIOB
#define DO_2_PIN        GPIO_PIN_9
#define BREAK_PORT      GPIOB
#define BREAK_PIN       GPIO_PIN_10
#define SET_PORT        GPIOB
#define SET_PIN         GPIO_PIN_11
#define UP_PORT         GPIOB
#define UP_PIN          GPIO_PIN_12
#define DOWN_PORT       GPIOB
#define DOWN_PIN        GPIO_PIN_13
#define LEFT_PORT       GPIOB
#define LEFT_PIN        GPIO_PIN_14
#define RIGHT_PORT      GPIOB
#define RIGHT_PIN       GPIO_PIN_15

#define DO_0_PORT        GPIOC
#define DO_0_PIN         GPIO_PIN_13

#endif // PIN_MAP_H
