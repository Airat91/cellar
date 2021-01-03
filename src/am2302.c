#include "am2302.h"
#include "pin_map.h"
#include "main.h"
#include "cmsis_os.h"

/**
  * @defgroup am2302
  * @brief work with temperature and hummidity sensor AM2302
  */

/**
  * @addtogroup am2302
  * @{
  */
#define AM2302_CH_NUM 3
#define AM2302_TIMEOUT 1000
/**
  * @}
  */

static const am2302_pin_t am2302_pin[AM2302_CH_NUM] = {
    {.port = AM2302_1_PORT, .pin = AM2302_1_PIN},
    {.port = AM2302_2_PORT, .pin = AM2302_2_PIN},
    {.port = AM2302_3_PORT, .pin = AM2302_3_PIN},
};

/*========== FUNCTIONS ==========*/

/**
 * @brief Init AM2302 pins
 * @return  0 - AM2302 pins init successfull,\n
 *          -1 - error
 * @ingroup am2302
 */
int am2302_init (void) {
    int result = 0;
    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    for(uint8_t i = 0; i < AM2302_CH_NUM; i++){
        GPIO_InitStruct.Pin = am2302_pin[i].pin;
        HAL_GPIO_Init (am2302_pin[i].port, &GPIO_InitStruct);
    }
  
    return result;
}

/**
 * @brief Deinit AM2302 pins
 * @ingroup am2302
 */
void am2302_deinit (void){
    for(uint8_t i = 0; i < AM2302_CH_NUM; i++){
        HAL_GPIO_DeInit(am2302_pin[i].port, am2302_pin[i].pin);
    }
}

/**
 * @brief Get value from AM2302
 * @param channel - number of AM2302 channels
 * @return Data in am2302_data_t type
 * @ingroup am2302
 */
am2302_data_t am2302_get (uint8_t channel) {
    
    am2302_data_t result = {0};
    uint32_t T = 0;
    uint8_t i = 0;
    uint8_t read_data[5] = {0};
    uint32_t current_time = 0;
    uint32_t time_left = 0;
    
    // Config AM2032_PIN to OUT
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = am2302_pin[channel].pin;
    HAL_GPIO_Init (am2302_pin[channel].port, &GPIO_InitStruct);

    // SDA = 0
    HAL_GPIO_WritePin (am2302_pin[channel].port, am2302_pin[channel].pin, GPIO_PIN_RESET);

    // Run timeout 1ms
    //osDelay(1);
    us_tim_delay(1000);

    // SDA = 1
    HAL_GPIO_WritePin (am2302_pin[channel].port, am2302_pin[channel].pin, GPIO_PIN_SET);

    // Config AM2032_PIN to INPUT
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init (am2302_pin[channel].port, &GPIO_InitStruct);

    taskENTER_CRITICAL();

    // Save current time for timeout
    current_time = us_tim_get_value();

    // Wait SDA->0 or AM2302_TIMEOUT
    while ((HAL_GPIO_ReadPin(am2302_pin[channel].port, am2302_pin[channel].pin) == 1) &&
        (time_left < AM2302_TIMEOUT)) {
        time_left = us_tim_get_value() - current_time;
    }
    if (time_left > AM2302_TIMEOUT) {
        result.error = 1;
    } else {
       current_time = us_tim_get_value();     // Update current_time
    }

    // Wait SDA->1 or AM2302_TIMEOUT
    while ((HAL_GPIO_ReadPin (am2302_pin[channel].port, am2302_pin[channel].pin) == 0) &&
           (time_left < AM2302_TIMEOUT)) {
           time_left = us_tim_get_value() - current_time;
    }
    if (time_left > AM2302_TIMEOUT) {
        result.error = 1;
    } else {
       current_time = us_tim_get_value();     // Update current_time
    }
  
    // Wait SDA->0 or AM2302_TIMEOUT
    while ((HAL_GPIO_ReadPin(am2302_pin[channel].port, am2302_pin[channel].pin) == 1) &&
           (time_left < AM2302_TIMEOUT)) {
           time_left = us_tim_get_value() - current_time;
    }
    if (time_left > AM2302_TIMEOUT) {
        result.error = 1;
    } else {
       current_time = us_tim_get_value();     // Update current_time
    }
  
    //Read data
    for(uint8_t byte_nmb = 0; byte_nmb < 5; byte_nmb++){
        for( i = 8; i > 0; i--) {

            // Wait SDA->1 or AM2302_TIMEOUT
            while ((HAL_GPIO_ReadPin (am2302_pin[channel].port, am2302_pin[channel].pin) == 0) &&
                   (time_left < AM2302_TIMEOUT)) {
                   time_left = us_tim_get_value() - current_time;
            }
            if (time_left > AM2302_TIMEOUT) {
                result.error = 1;
            } else {
               current_time = us_tim_get_value();     // Update current_time
               T = time_left;
            }

            // Wait SDA->0 or AM2302_TIMEOUT
            while ((HAL_GPIO_ReadPin(am2302_pin[channel].port, am2302_pin[channel].pin) == 1) &&
                   (time_left < AM2302_TIMEOUT)) {
                   time_left = us_tim_get_value() - current_time;
            }
            if (time_left > AM2302_TIMEOUT) {
                result.error = 1;
            } else {
               current_time = us_tim_get_value();     // Update current_time
               if(time_left > T){
                   read_data[byte_nmb] |= (1 << (i-1));
               }
            }
        }
    }
    taskEXIT_CRITICAL();
    uint8_t paritet = 0;
    if(read_data[4] != 0){
        for(i = 0; i < 4; i++){
            paritet += read_data[i];
        }
        if(paritet == read_data[4]){
            result.hum = read_data[0]*256 + read_data[1];
            result.tmpr = read_data[2]*256 + read_data[3];
            result.paritet = read_data[4];
        }else{
            result.error = 1;
        }
    }else{
        result.error = 1;
    }
    return result;
}

