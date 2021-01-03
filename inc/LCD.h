#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "fonts.h"

#ifndef LCD_H
#define LCD_H 1

/**
  * @addtogroup LCD
  * @{
  */
#define LCD_SPI SPI1
#define MAX_X 128
#define MAX_Y 64
#define LCD_BUF_ARRAY_LEN 1024
/**
  * @}
  */

/*========== TYPEDEFS ==========*/

typedef enum {
    LCD_RW_WRITE = 0,
    LCD_RW_READ = 1,
}LCD_RW_t;

typedef enum {
    LCD_RS_COMM = 0,
    LCD_RS_DATA = 1,
}LCD_RS_t;

typedef enum {
    LCD_COLOR_WHITE = 0,
    LCD_COLOR_BLACK = 1,
}LCD_color_t;

typedef enum {
    LCD_BACKLIGHT_OFF = 0,
    LCD_BACKLIGHT_ON,
    LCD_BACKLIGHT_SHUTDOWN,
}LCD_backlight_t;

typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t backlight;          //backlight state {0 - off, 1 - on}
    uint16_t backlight_lvl;      //backlight level by PWM in ON state {0 - always off, 1 - 10% ... 10 - 100%}
    uint8_t buf[LCD_BUF_ARRAY_LEN];
    uint16_t auto_off;           //*10 seconds to display backloght auto off
    uint32_t auto_off_timeout;
}LCD_t;

/**
 * @brief lcd_spi
 * @ingroup LCD
 */
extern SPI_HandleTypeDef lcd_spi;

/**
 * @brief Params of LCD
 * @ingroup LCD
 */
extern LCD_t LCD;

/**
 * @brief Buffer for update LCD
 * @ingroup LCD
 */
extern uint8_t LCD_buf[];
extern TIM_HandleTypeDef htim4;

/*========== FUNCTION PROTOTYPES ==========*/

int LCD_init (void);
void LCD_deinit (void);
int LCD_spi_init (void);
void LCD_spi_deinit (void);
void LCD_gpio_init (void);
void LCD_gpio_deinit (void);
void LCD_send(LCD_RW_t rw, LCD_RS_t rs, uint8_t data);
void LCD_update(void);
int LCD_set_pixel(uint8_t x, uint8_t y, LCD_color_t color);
int LCD_fill_area(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2, LCD_color_t color);
int LCD_invert_area(uint8_t x1, uint8_t y1, uint8_t x2, uint8_t y2);
void LCD_clr(void);
int LCD_set_xy(uint8_t x, uint8_t y);
void LCD_backlight_on (void);
void LCD_backlight_off (void);
void LCD_backlight_shutdown(void);
void LCD_backlight_toggle (void);
int LCD_print_char(char ch, FontDef_t* font, LCD_color_t color);
int LCD_print(char* string, FontDef_t* font, LCD_color_t color);
uint8_t align_text_center(char* string, FontDef_t font);
uint8_t align_text_right(char* string, FontDef_t font);
int LCD_backlight_timer_init(void);
void LCD_backlight_timer_handler(void);
void LCD_backlight_pin_init(void);
void LCD_backlight_pin_deinit(void);
void LCD_backlight_pin_off_state(void);

#endif // LCD_H
