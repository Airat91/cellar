#include "st7735.h"
#include "pin_map.h"
#include "dcts.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_spi.h"
#include "stm32f1xx_hal_rcc.h"
#include "main.h"
#include "string.h"
#include "fonts.h"

/**
  * @defgroup st7735
  * @brief work with TFT display based on st7735 controller
  */

/*========= GLOBAL VARIABLES ==========*/

SPI_HandleTypeDef st7735_spi = {0};
static int st7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h);
st7735_t st7735 = {
    .x = 0,
    .y = 0,
    .backlight_lvl = 2,
    .backlight = 0,
    .auto_off = 0,
    .auto_off_timeout = 0,
};

static spi_buf_t spi_send_buf = {
    .buff = {0},
    .len = 0,
    .ptr = 0,
    .sending = 0,
};

uint16_t st7735_disp[ST7735_MAX_X*ST7735_MAX_Y] = {0};

/*========== FUNCTIONS ==========*/

int st7735_init (void){
    int result = 0;
    st7735_gpio_init();
    if(st7735_spi_init()<0){
        result = -1;
    }

    st7735_cmd(ST7735_SWRESET);
    osDelay(150);
    st7735_cmd(ST7735_SLPOUT );
    osDelay(255);
    st7735_cmd(ST7735_FRMCTR1);
    st7735_data(0x01);
    st7735_data(0x2C);
    st7735_data(0x2D);

    st7735_cmd(ST7735_FRMCTR2);
    st7735_data(0x01);
    st7735_data(0x2C);
    st7735_data(0x2D);

    st7735_cmd(ST7735_FRMCTR3);
    st7735_data(0x01);
    st7735_data(0x2C);
    st7735_data(0x2D);
    st7735_data(0x01);
    st7735_data(0x2C);
    st7735_data(0x2D);

    st7735_cmd(ST7735_INVCTR );
    st7735_data(0x07);

    st7735_cmd(ST7735_PWCTR1 );
    st7735_data(0xA2);
    st7735_data(0x02);
    st7735_data(0x84);

    st7735_cmd(ST7735_PWCTR2 );
    st7735_data(0xC5);

    st7735_cmd(ST7735_PWCTR3 );
    st7735_data(0x0A);
    st7735_data(0x00);

    st7735_cmd(ST7735_PWCTR4 );
    st7735_data(0x8A);
    st7735_data(0x2A);

    st7735_cmd(ST7735_PWCTR5 );
    st7735_data(0x8A);
    st7735_data(0xEE);

    st7735_cmd(ST7735_VMCTR1 );
    st7735_data(0x0E);

    st7735_cmd(ST7735_INVOFF );

    st7735_cmd(ST7735_MADCTL );
    st7735_data(ST7735_ROTATION);

    st7735_cmd(ST7735_COLMOD );
    st7735_data(0x05);

    st7735_cmd(ST7735_IDMON );

    //===========================

    st7735_cmd(ST7735_CASET  );
    st7735_data(0x00);
    st7735_data(0x00);
    st7735_data(0x00);
    st7735_data(0x7F);
    st7735_cmd(ST7735_RASET  );
    st7735_data(0x00);
    st7735_data(0x00);
    st7735_data(0x00);
    st7735_data(0x7F);

    //===========================

    st7735_cmd(ST7735_GMCTRP1);
    st7735_data(0x02);
    st7735_data(0x1C);
    st7735_data(0x07);
    st7735_data(0x12);
    st7735_data(0x37);
    st7735_data(0x32);
    st7735_data(0x29);
    st7735_data(0x2D);
    st7735_data(0x29);
    st7735_data(0x25);
    st7735_data(0x2B);
    st7735_data(0x39);
    st7735_data(0x00);
    st7735_data(0x01);
    st7735_data(0x03);
    st7735_data(0x10);

    st7735_cmd(ST7735_GMCTRN1);
    st7735_data(0x03);
    st7735_data(0x1D);
    st7735_data(0x07);
    st7735_data(0x06);
    st7735_data(0x2E);
    st7735_data(0x2C);
    st7735_data(0x29);
    st7735_data(0x2D);
    st7735_data(0x2E);
    st7735_data(0x2E);
    st7735_data(0x37);
    st7735_data(0x3F);
    st7735_data(0x00);
    st7735_data(0x00);
    st7735_data(0x02);
    st7735_data(0x10);

    st7735_cmd(ST7735_NORON  );
    osDelay(10);
    st7735_cmd(ST7735_DISPON );
    osDelay(100);

    //===========================

    /*char string[50];
    strcpy(string,"Hello world!");

    ST7735_fill_rect(0,0,160,128,ST7735_BLACK);
    ST7735_fill_rect(10,10,140,108,ST7735_BLUE);
    ST7735_fill_rect(20,20,120,88,ST7735_CYAN);
    ST7735_fill_rect(30,30,100,68,ST7735_YELLOW);
    ST7735_fill_rect(35,55,(uint8_t)strlen(string)*Font_7x10.FontWidth+2,Font_7x10.FontHeight+2,ST7735_WHITE);

    st7735_xy(36,55);
    st7735_print(string, &Font_7x10, ST7735_RED);

    osDelay(1000);
    ST7735_fill_rect(0,0,160,128,ST7735_BLACK);*/

    return result;
}

void st7735_deinit (void){
    st7735_spi_deinit();
    st7735_gpio_deinit();
}

/**
 * @brief Init st7735 spi
 * @return  0 - OK,\n
 *          -1 = SPI init error
 * @ingroup st7735
 */
int st7735_spi_init (void){
    __HAL_RCC_SPI1_CLK_ENABLE();
    int result = 0;
    st7735_spi.Instance = SPI1;
    st7735_spi.Init.Mode = SPI_MODE_MASTER;
    st7735_spi.Init.Direction = SPI_DIRECTION_1LINE;
    st7735_spi.Init.DataSize = SPI_DATASIZE_8BIT;
    st7735_spi.Init.CLKPolarity = SPI_POLARITY_HIGH;
    st7735_spi.Init.CLKPhase = SPI_PHASE_1EDGE;
    st7735_spi.Init.NSS = SPI_NSS_SOFT;
    st7735_spi.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
    st7735_spi.Init.FirstBit = SPI_FIRSTBIT_MSB;
    st7735_spi.Init.TIMode = SPI_TIMODE_DISABLE;
    st7735_spi.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    st7735_spi.Init.CRCPolynomial = 10;
    if (HAL_SPI_Init(&st7735_spi) != HAL_OK)
    {
        result = -1;
    }
    __HAL_SPI_ENABLE(&st7735_spi);
    SPI_1LINE_TX(&st7735_spi);
    HAL_NVIC_EnableIRQ(SPI1_IRQn);
    return result;
}

/**
 * @brief Deinit st7735 spi
 * @ingroup st7735
 */
void st7735_spi_deinit (void){
    HAL_SPI_DeInit(&st7735_spi);
}

/**
 * @brief Init st7735 pins
 * @ingroup st7735
 */
void st7735_gpio_init (void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = ST7735_CS_PIN;
    HAL_GPIO_Init(ST7735_CS_PORT, &GPIO_InitStruct);
    HAL_GPIO_WritePin(ST7735_A0_PORT, ST7735_A0_PIN, GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = ST7735_A0_PIN;
    HAL_GPIO_Init(ST7735_A0_PORT, &GPIO_InitStruct);
#if(ST7735_RST_PIN_EN)
    HAL_GPIO_WritePin(ST7735_RESET_PORT, ST7735_RESET_PIN, GPIO_PIN_SET);
    GPIO_InitStruct.Pin = ST7735_RESET_PIN;
    HAL_GPIO_Init(ST7735_RESET_PORT, &GPIO_InitStruct);
#endif  // ST7735_RST_PIN_EN


    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pin = ST7735_SCK_PIN;
    HAL_GPIO_Init(ST7735_SCK_PORT, &GPIO_InitStruct);
    GPIO_InitStruct.Pin = ST7735_MOSI_PIN;
    HAL_GPIO_Init(ST7735_MOSI_PORT, &GPIO_InitStruct);

    __HAL_AFIO_REMAP_SPI1_ENABLE();
}

/**
 * @brief Deinit st7735 pins
 * @ingroup st7735
 */
void st7735_gpio_deinit (void){
    HAL_GPIO_DeInit(ST7735_CS_PORT,ST7735_CS_PIN);
#if(ST7735_RST_PIN_EN)
    HAL_GPIO_DeInit(ST7735_RESET_PORT, ST7735_RESET_PIN);
#endif  // ST7735_RST_PIN_EN
    HAL_GPIO_DeInit(ST7735_A0_PORT, ST7735_A0_PIN);
    HAL_GPIO_DeInit(ST7735_SCK_PORT, ST7735_SCK_PIN);
    HAL_GPIO_DeInit(ST7735_MOSI_PORT, ST7735_MOSI_PIN);
}


int st7735_cmd(uint8_t cmd){
    int result = 0;
    int timeout = 0;
    HAL_GPIO_WritePin(ST7735_A0_PORT, ST7735_A0_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit_IT(&st7735_spi,&cmd,1) == HAL_OK){
        while((((uint16_t)st7735_spi.Instance->SR & SPI_SR_BSY)&&(timeout < 5000))){
            timeout++;
            osDelay(1);
        }
        if(timeout > 5000){
            result = -2;
        }
    }else{
        result = -1;
    }
    HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN,GPIO_PIN_SET);
    return result;
}

int st7735_array(uint8_t* array, uint8_t len){
    int result = 0;
    int timeout = 0;
    HAL_GPIO_WritePin(ST7735_A0_PORT, ST7735_A0_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN, GPIO_PIN_RESET);
    //while(len > 0){
        if(HAL_SPI_Transmit_IT(&st7735_spi,array,len) == HAL_OK){
            while((((uint16_t)st7735_spi.Instance->SR & SPI_SR_BSY)&&(timeout < 5000))){
                timeout++;
                osDelay(1);
            }
            if(timeout > 5000){
                result = -2;
            }
        }else{
            result = -1;
        }
        /*array++;
        len--;
    }*/
    HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN,GPIO_PIN_SET);
    return result;
}

int st7735_data(uint8_t data){
    int result = 0;
    int timeout = 0;
    HAL_GPIO_WritePin(ST7735_A0_PORT, ST7735_A0_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN, GPIO_PIN_RESET);
    if(HAL_SPI_Transmit_IT(&st7735_spi,&data,1) == HAL_OK){
        while((((uint16_t)st7735_spi.Instance->SR & SPI_SR_BSY)&&(timeout < 5000))){
            timeout++;
            osDelay(1);
        }
        if(timeout > 5000){
            result = -2;
        }
    }else{
        result = -1;
    }
    HAL_GPIO_WritePin(ST7735_CS_PORT, ST7735_CS_PIN,GPIO_PIN_SET);
    return result;
}

int st7735_draw_pixel(uint8_t x, uint8_t y, uint16_t color){
    int result = 0;

    if((x > ST7735_MAX_X)||(y > ST7735_MAX_Y)){
        result = -1;
    }else{
        st7735_SetAddressWindow(x,y,1,1);
        uint8_t data[] = { color >> 8, color & 0xFF };
        st7735_array(data, sizeof(data));
    }

    return result;
}

int st7735_fill_rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h, uint16_t color){
    int result = 0;
    if(x + w - 1 > ST7735_MAX_X){
        w = ST7735_MAX_X - x;
        result -=10;
    }
    if(y + h - 1 > ST7735_MAX_Y){
        h = ST7735_MAX_Y - y;
        result -=100;
    }
    st7735_SetAddressWindow(x, y, w - 1,h - 1);
    uint8_t data[] = { color >> 8, color & 0xFF };
    for(st7735.y = h; st7735.y > 0; st7735.y--) {
        for(st7735.x = w; st7735.x > 0; st7735.x--) {
            st7735_array(data, 2);
        }
    }

    return result;
}

int st7735_print_char(char ch, FontDef_t* font, uint16_t color){
    int result = 0;
    //check coordinates values
    if((st7735.x + font->FontWidth > ST7735_MAX_X) && (st7735.y + font->FontHeight > ST7735_MAX_Y)){
        result = -1;
    }else{
        // go through font
        uint16_t data = 0;
        for (uint8_t line_num = 1; line_num < (font->FontHeight+1); line_num++) {
            if(font->data_size_in_bytes == 1){
                uint8_t * line_1_byte = font->data;
                data = (uint16_t)(line_1_byte[(ch - font->shift + 1) * font->FontHeight - line_num] << 8);
            }else if(font->data_size_in_bytes == 2){
                uint16_t * line_2_byte = font->data;
                data = line_2_byte[(ch - font->shift + 1) * font->FontHeight - line_num];
            }
            for (uint8_t x_pos = 0; x_pos < font->FontWidth; x_pos++) {
                if (data & (0x8000 >> x_pos)) {
                    st7735_draw_pixel(st7735.x + x_pos, st7735.y + line_num, color);
                }
            }
        }
        st7735.x += font->FontWidth;
    }
    return result;
}

int st7735_print(char* string, FontDef_t* font, uint16_t color){
    int result = 0;
    while (*string) {
        if (st7735_print_char(*string, font, color) == 0) {
            string++;
        }else{
            result = -1;
        }
    }
    return result;
}

int st7735_xy(uint8_t x, uint8_t y){
    int result = 0;
    if((x >= ST7735_MAX_X)||(y >= ST7735_MAX_Y)){
        result = -1;
    }else{
        st7735.x = x;
        st7735.y = y;
    }

    return  result;
}

static int st7735_SetAddressWindow(uint8_t x0, uint8_t y0, uint8_t w, uint8_t h){
    int result = 0;
    if((x0 + w - 1 > ST7735_MAX_X)||(y0 + h - 1 > ST7735_MAX_Y)){
        result = -1;
    }else{
        st7735_cmd(ST7735_CASET);
        uint8_t data[] = { 0x00, x0, 0x00, x0 + w };
        st7735_array(data, 4);

        st7735_cmd(ST7735_RASET);
        data[1] = y0;
        data[3] = y0 + h;
        st7735_array(data, 4);

        st7735_cmd(ST7735_RAMWR);
    }

    return result;
}

int st7735_send(uint32_t timeout_us){
    int result = 0;
    uint32_t start = us_tim_get_value();
    uint32_t timeout = 0;
    while((spi_send_buf.sending)&&(timeout < timeout_us)){
        timeout = us_tim_get_value() - start;
    }
    if(spi_send_buf.sending == 0){
        HAL_SPI_Transmit_IT(&st7735_spi,spi_send_buf.buff,spi_send_buf.len);
    }else{
        result = -1;
    }
    return result;
}

int st7735_update(void){
    int result = 0;
    st7735_SetAddressWindow(0, 0, ST7735_MAX_X - 1,ST7735_MAX_Y - 1);
    st7735.x = 0;
    st7735.y = 0;
    for(uint16_t i = 0; i < ST7735_MAX_X*ST7735_MAX_Y; i++){
        st7735_array((uint8_t*)st7735_disp[i*2], 2);
    }

    return result;
}
