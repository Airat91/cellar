#ifndef PIN_MAP_H
#define PIN_MAP_H

#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

#define PWR_PORT        GPIOA
#define PWR_PIN         GPIO_PIN_0
#define RS_485_DE_PORT  GPIOA
#define RS_485_DE_PIN   GPIO_PIN_1
#define RS_485_TX_PORT  GPIOA
#define RS_485_TX_PIN   GPIO_PIN_2
#define RS_485_RX_PORT  GPIOA
#define RS_485_RX_PIN   GPIO_PIN_3
#define WTR_LEV_PORT    GPIOA
#define WTR_LEV_PIN     GPIO_PIN_4
#define WTR_TMP_PORT    GPIOA
#define WTR_TMP_PIN     GPIO_PIN_5

#define OK_PORT         GPIOA
#define OK_PIN          GPIO_PIN_8
#define SET_PORT        GPIOA
#define SET_PIN         GPIO_PIN_9
#define BREAK_PORT      GPIOA
#define BREAK_PIN       GPIO_PIN_10
#define AM2302_2_PORT   GPIOA
#define AM2302_2_PIN    GPIO_PIN_11
#define AM2302_1_PORT   GPIOA
#define AM2302_1_PIN    GPIO_PIN_12
#define DEBUG_TMS_PORT  GPIOA
#define DEBUG_TMS_PIN   GPIO_PIN_13
#define DEBUG_TCK_PORT  GPIOA
#define DEBUG_TCK_PIN   GPIO_PIN_14
#define LCD_CS_PORT     GPIOA
#define LCD_CS_PIN      GPIO_PIN_15

#define LCD_SCK_PORT    GPIOB
#define LCD_SCK_PIN     GPIO_PIN_3
#define LCD_RST_PORT    GPIOB
#define LCD_RST_PIN     GPIO_PIN_4
#define LCD_MOSI_PORT   GPIOB
#define LCD_MOSI_PIN    GPIO_PIN_5
#define LCD_LIGHT_PORT  GPIOB
#define LCD_LIGHT_PIN   GPIO_PIN_6

#define AM2302_3_PORT   GPIOB
#define AM2302_3_PIN    GPIO_PIN_11
#define UP_PORT         GPIOB
#define UP_PIN          GPIO_PIN_12
#define DOWN_PORT       GPIOB
#define DOWN_PIN        GPIO_PIN_13
#define LEFT_PORT       GPIOB
#define LEFT_PIN        GPIO_PIN_14
#define RIGHT_PORT      GPIOB
#define RIGHT_PIN       GPIO_PIN_15

#define LED_PORT        GPIOC
#define LED_PIN         GPIO_PIN_13

#endif // PIN_MAP_H
