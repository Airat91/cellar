#include "stdint.h"
#include "main.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_spi.h"
#include "fonts.h"

#ifndef ST7735_H
#define ST7735_H 1

#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MV  0x20
#define ST7735_MADCTL_ML  0x10
#define ST7735_MADCTL_RGB 0x08
#define ST7735_MADCTL_MH  0x04

#define ST7735_WIDTH 128
#define ST7735_HEIGHT 160
#define ST7735_RST_PIN_EN 0

#define ST7735_MAX_X ST7735_HEIGHT
#define ST7735_MAX_Y ST7735_WIDTH

//#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MY) // default orientation
//#define ST7735_ROTATION (ST7735_MADCTL_MY | ST7735_MADCTL_MV) // rotate right
//#define ST7735_ROTATION (ST7735_MADCTL_MX | ST7735_MADCTL_MV) // rotate left
#define ST7735_ROTATION (ST7735_MADCTL_MV | ST7735_MADCTL_MH)  // upside down

/*========== TYPEDEFS ==========*/

typedef enum{
    ST7735_CMD = 0,
    ST7735_DATA = 1,
}st7735_cmd_dat_t;

typedef enum{
    ST7735_NOP     = 0x00,
    ST7735_SWRESET = 0x01,
    ST7735_RDDID   = 0x04,
    ST7735_RDDST   = 0x09,

    ST7735_SLPIN   = 0x10,
    ST7735_SLPOUT  = 0x11,
    ST7735_PTLON   = 0x12,
    ST7735_NORON   = 0x13,

    ST7735_INVOFF  = 0x20,
    ST7735_INVON   = 0x21,
    ST7735_DISPOFF = 0x28,
    ST7735_DISPON  = 0x29,
    ST7735_CASET   = 0x2A,
    ST7735_RASET   = 0x2B,
    ST7735_RAMWR   = 0x2C,
    ST7735_RAMRD   = 0x2E,

    ST7735_PTLAR   = 0x30,
    ST7735_MADCTL  = 0x36,
    ST7735_IDMOFF  = 0x38,
    ST7735_IDMON   = 0x39,
    ST7735_COLMOD  = 0x3A,

    ST7735_FRMCTR1 = 0xB1,
    ST7735_FRMCTR2 = 0xB2,
    ST7735_FRMCTR3 = 0xB3,
    ST7735_INVCTR  = 0xB4,
    ST7735_DISSET5 = 0xB6,

    ST7735_PWCTR1  = 0xC0,
    ST7735_PWCTR2  = 0xC1,
    ST7735_PWCTR3  = 0xC2,
    ST7735_PWCTR4  = 0xC3,
    ST7735_PWCTR5  = 0xC4,
    ST7735_VMCTR1  = 0xC5,

    ST7735_RDID1   = 0xDA,
    ST7735_RDID2   = 0xDB,
    ST7735_RDID3   = 0xDC,
    ST7735_RDID4   = 0xDD,

    ST7735_PWCTR6  = 0xFC,

    ST7735_GMCTRP1 = 0xE0,
    ST7735_GMCTRN1 = 0xE1,
}st7735_cmd_t;

typedef enum{
    ST7735_BLACK   = 0x0000,
    ST7735_BLUE    = 0x001F,
    ST7735_RED     = 0xF800,
    ST7735_GREEN   = 0x07E0,
    ST7735_CYAN    = 0x07FF,
    ST7735_MAGENTA = 0xF81F,
    ST7735_YELLOW  = 0xFFE0,
    ST7735_ORANGE  = 0xF8E3,
    ST7735_WHITE   = 0xFFFF,
}st7735_color_t;

typedef struct {
    uint8_t x;
    uint8_t y;
    uint8_t backlight;          //backlight state {0 - off, 1 - on}
    uint16_t backlight_lvl;      //backlight level by PWM in ON state {0 - always off, 1 - 10% ... 10 - 100%}
    uint16_t auto_off;           //*10 seconds to display backloght auto off
    uint32_t auto_off_timeout;
}st7735_t;

/*========= GLOBAL VARIABLES ==========*/

extern st7735_t st7735;
extern uint16_t st7735_disp[];

/*========== FUNCTION PROTOTYPES ==========*/

int st7735_init (void);
void st7735_deinit (void);
int st7735_spi_init (void);
void st7735_spi_deinit (void);
void st7735_gpio_init (void);
void st7735_gpio_deinit (void);
int st7735_cmd(uint8_t cmd);
int st7735_array(uint8_t* array, uint8_t len);
int st7735_data(uint8_t data);
int st7735_draw_pixel(uint8_t x, uint8_t y, uint16_t color);
int st7735_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color);
int st7735_xy(uint8_t x, uint8_t y);
int st7735_print_char(char ch, FontDef_t* font, uint16_t color);
int st7735_print(char* string, FontDef_t* font, uint16_t color);
int st7735_update(void);

#endif // ST7735_H
