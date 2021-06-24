#include "ds18b20.h"
#include "pin_map.h"
#include "main.h"
#include "cmsis_os.h"
#include "dcts.h"

/**
  * @defgroup ds18b20
  * @brief work with 1-wire temperature sensor DS18B20
  */

/*========== FUNCTIONS ==========*/

static void ds18b20_pin_input(void);
static void ds18b20_pin_output(void);
static void write_bit(u8 bit);
static void write_data(u8 data);
static u8 read_bit(void);
static u8 read_data(void);
static u8 reset_1_wire(void);
static unsigned char crc8_maxim(unsigned char *pcBlock, unsigned char len);

/**
 * @brief Init DS18B20 pin as input
 * @ingroup ds18b20
 */
static void ds18b20_pin_input(void){
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = DS18B20_PIN;
    HAL_GPIO_Init (DS18B20_PORT, &GPIO_InitStruct);
}
/**
 * @brief Init DS18B20 pin as output OD
 * @ingroup ds18b20
 */
static void ds18b20_pin_output(void){
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = DS18B20_PIN;
    HAL_GPIO_Init (DS18B20_PORT, &GPIO_InitStruct);
}

/**
 * @brief Write bit by 1-Wire protocol
 * @param bit - 1 or 0
 * @note use us_delay and critical section
 * @ingroup ds18b20
 * @todo use timer and exti interrupts
 */
static void write_bit(u8 bit){
    ds18b20_pin_output();
    taskENTER_CRITICAL();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    us_tim_delay(60 - bit * 59);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
    taskEXIT_CRITICAL();
    ds18b20_pin_input();
    us_tim_delay(bit * 59);
    us_tim_delay(1);
}

/**
 * @brief Read bit by 1-Wire protocol
 * @return readed bit (1 or 0
 * @note use us_delay and critical section
 * @ingroup ds18b20
 * @todo use timer and exti interrupts
 */
static u8 read_bit(void){
    u8 result = 0;
    ds18b20_pin_output();
    taskENTER_CRITICAL();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    us_tim_delay(1);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
    ds18b20_pin_input();
    us_tim_delay(14);
    if(HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN)){
        result = 1;
    }
    taskEXIT_CRITICAL();
    us_tim_delay(46);
    return result;
}

/**
 * @brief Write byte by 1-Wire protocol
 * @param data - sended byte
 * @ingroup ds18b20
 */
static void write_data(u8 data){
    u8 temp = 0;
    for(u8 i = 0; i < 8; i++){
        temp = (data >> i)&0x01;
        write_bit(temp);
    }
}

/**
 * @brief Read byte by 1-Wire protocol
 * @return readed byte
 * @ingroup ds18b20
 */
static u8 read_data(void){
    u8 result = 0;
    for(u8 i = 0; i < 8; i++){
        if(read_bit()){
            result |= (0x01 << i);
        }
    }
    return result;
}

/**
 * @brief reset_1_wire
 * @return  0 - device not found,\n
 *          1 - presence get
 * @note use us_delay and critical section
 * @ingroup ds18b20
 * @todo use timer and exti interrupts
 */
static u8 reset_1_wire(void){
    u8 result = 0;
    u32 timeout = 0;
    u32 start = 0;
    u32 presence = 0;

    taskENTER_CRITICAL();
    ds18b20_pin_output();
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_RESET);
    us_tim_delay(480);
    HAL_GPIO_WritePin(DS18B20_PORT, DS18B20_PIN, GPIO_PIN_SET);
    ds18b20_pin_input();
    start = us_tim_get_value();
    // wait presence pulse
    while((HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == 1)&&(timeout < 60)){
        timeout = us_tim_get_value() - start;
    }
    start = us_tim_get_value();
    while((HAL_GPIO_ReadPin(DS18B20_PORT, DS18B20_PIN) == 0)&&(presence < 480)){
        presence = us_tim_get_value() - start;
    }
    taskEXIT_CRITICAL();
    if(presence < 240){
        result = 1;
    }
    us_tim_delay(480 - presence);
    return result;
}

/**
 * @brief Init DS8B20 by 1-wire
 * @return  0 - device not found,\n
 *          1 - DS18B20 init successfull,\n
 *          -1 - crc_error
 * @ingroup ds18b20
 */
int ds18b20_init (void) {
    int result = 0;
    u8 buff[9] = {0};

    if(reset_1_wire()){
        write_data(SKIP_ROM);
        write_data(WRITE_SCRPAD);
        write_data(TH_ALRM);   // Tmpr high alarm
        write_data(TL_ALRM);   // Tmpr low alarm
        write_data(CONF);   // resolution config
        reset_1_wire();
        write_data(SKIP_ROM);
        write_data(READ_SCRPAD);
        for(u8 i = 0; i < 9; i++){
            buff[i] = read_data();
        }
        if(crc8_maxim(buff,8)!=buff[8]){
            result = -1;
        }
        reset_1_wire();
        write_data(SKIP_ROM);
        write_data(COPY_SCRPAD);
        osDelay(100);
        result = 1;
    }
    return result;
}

/**
 * @brief Table for calculate crc by MAXIM
 * @ingroup ds18b20
 */
static const unsigned char Crc8Table[256] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83, 0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x9D, 0xC3, 0x21, 0x7F, 0xFC, 0xA2, 0x40, 0x1E, 0x5F, 0x01, 0xE3, 0xBD, 0x3E, 0x60, 0x82, 0xDC,
    0x23, 0x7D, 0x9F, 0xC1, 0x42, 0x1C, 0xFE, 0xA0, 0xE1, 0xBF, 0x5D, 0x03, 0x80, 0xDE, 0x3C, 0x62,
    0xBE, 0xE0, 0x02, 0x5C, 0xDF, 0x81, 0x63, 0x3D, 0x7C, 0x22, 0xC0, 0x9E, 0x1D, 0x43, 0xA1, 0xFF,
    0x46, 0x18, 0xFA, 0xA4, 0x27, 0x79, 0x9B, 0xC5, 0x84, 0xDA, 0x38, 0x66, 0xE5, 0xBB, 0x59, 0x07,
    0xDB, 0x85, 0x67, 0x39, 0xBA, 0xE4, 0x06, 0x58, 0x19, 0x47, 0xA5, 0xFB, 0x78, 0x26, 0xC4, 0x9A,
    0x65, 0x3B, 0xD9, 0x87, 0x04, 0x5A, 0xB8, 0xE6, 0xA7, 0xF9, 0x1B, 0x45, 0xC6, 0x98, 0x7A, 0x24,
    0xF8, 0xA6, 0x44, 0x1A, 0x99, 0xC7, 0x25, 0x7B, 0x3A, 0x64, 0x86, 0xD8, 0x5B, 0x05, 0xE7, 0xB9,
    0x8C, 0xD2, 0x30, 0x6E, 0xED, 0xB3, 0x51, 0x0F, 0x4E, 0x10, 0xF2, 0xAC, 0x2F, 0x71, 0x93, 0xCD,
    0x11, 0x4F, 0xAD, 0xF3, 0x70, 0x2E, 0xCC, 0x92, 0xD3, 0x8D, 0x6F, 0x31, 0xB2, 0xEC, 0x0E, 0x50,
    0xAF, 0xF1, 0x13, 0x4D, 0xCE, 0x90, 0x72, 0x2C, 0x6D, 0x33, 0xD1, 0x8F, 0x0C, 0x52, 0xB0, 0xEE,
    0x32, 0x6C, 0x8E, 0xD0, 0x53, 0x0D, 0xEF, 0xB1, 0xF0, 0xAE, 0x4C, 0x12, 0x91, 0xCF, 0x2D, 0x73,
    0xCA, 0x94, 0x76, 0x28, 0xAB, 0xF5, 0x17, 0x49, 0x08, 0x56, 0xB4, 0xEA, 0x69, 0x37, 0xD5, 0x8B,
    0x57, 0x09, 0xEB, 0xB5, 0x36, 0x68, 0x8A, 0xD4, 0x95, 0xCB, 0x29, 0x77, 0xF4, 0xAA, 0x48, 0x16,
    0xE9, 0xB7, 0x55, 0x0B, 0x88, 0xD6, 0x34, 0x6A, 0x2B, 0x75, 0x97, 0xC9, 0x4A, 0x14, 0xF6, 0xA8,
    0x74, 0x2A, 0xC8, 0x96, 0x15, 0x4B, 0xA9, 0xF7, 0xB6, 0xE8, 0x0A, 0x54, 0xD7, 0x89, 0x6B, 0x35,
};

/**
 * @brief Calculate CRC8 using MAXIM algorythm
 * @param pcBlock - pointer to buffer
 * @param len - lenht of buffer
 * @return calculated crc8
 * @ingroup ds18b20
 */
static unsigned char crc8_maxim(unsigned char *pcBlock, unsigned char len){
    unsigned char crc = 0x00;

    while (len--)
        crc = Crc8Table[crc ^ *pcBlock++];

    return crc;
}

/**
 * @brief Get data from DS18B20
 * @return ds18b20_data_t tmpr and valid information
 * @ingroup ds18b20
 */
ds18b20_data_t ds18b20_get(void){
    ds18b20_data_t data = {0};
    uint8_t buff[9] = {0};

    reset_1_wire();
    write_data(SKIP_ROM);
    write_data(READ_SCRPAD);
    for(u8 i = 0; i < 9; i++){
        buff[i] = read_data();
    }
    data.tmpr = (buff[0]+(u16)(buff[1]<<8)) * DS18B20_RESOLUTION;
    if(crc8_maxim(buff,8)==buff[8]){
        data.valid = 1;
    }else{
        data.valid = 0;
    }
    return data;
}

/**
 * @brief Start of temperature convertion
 * @ingroup ds18b20
 */
void ds18b20_start_conv(void){
    reset_1_wire();
    write_data(SKIP_ROM);
    write_data(START_CONV_T);
}

/**
 * @brief ds18b20_task
 * @param argument
 */
void ds18b20_task (void const * argument){
    (void)argument;
    ds18b20_init();
    ds18b20_data_t data = {0};

    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        ds18b20_start_conv();
        osDelayUntil(&last_wake_time, DS18B20_CONV_MS);
        data = ds18b20_get();
        dcts_meas[TMPR].value = data.tmpr;
        dcts_meas[TMPR].valid = data.valid;
    }
}
