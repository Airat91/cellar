#include "am2302.h"
#include "pin_map.h"
#include "main.h"
#include "cmsis_os.h"
#include "dcts.h"

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

const am2302_pin_t am2302_pin[AM2302_CH_NUM] = {
    {.port = CH_2_PORT, .pin = CH_2_PIN},
    {.port = CH_3_PORT, .pin = CH_3_PIN},
    {.port = CH_4_PORT, .pin = CH_4_PIN},
};

/*========== FUNCTIONS ==========*/

/**
 * @brief am2302_task
 * @param argument
 */

#define am2302_task_period 3000
void am2302_task (void const * argument){
    (void)argument;
    uint32_t last_wake_time = osKernelSysTick();
    am2302_init();
    am2302_data_t ch_2 = {0};
    uint8_t ch_2_lost_con_cnt = 0;
    uint32_t ch_2_recieved = 0;
    uint32_t ch_2_lost = 0;
    am2302_data_t ch_3 = {0};
    uint8_t ch_3_lost_con_cnt = 0;
    uint32_t ch_3_recieved = 0;
    uint32_t ch_3_lost = 0;
    am2302_data_t ch_4 = {0};
    uint8_t ch_4_lost_con_cnt = 0;
    uint32_t ch_4_recieved = 0;
    uint32_t ch_4_lost = 0;
    while(1){
        ch_2 = am2302_get(0);
        taskENTER_CRITICAL();
        if(ch_2.error == 1){
            ch_2_lost++;
            ch_2_lost_con_cnt++;
            if(ch_2_lost_con_cnt > 2){
                dcts_meas[HUM_OUT].valid = FALSE;
                dcts_meas[TMPR_OUT].valid = FALSE;
            }
            if(ch_2_lost_con_cnt > 9){
                am2302_reinit(0);
            }
        }else{
            ch_2_recieved++;
            ch_2_lost_con_cnt = 0;
            dcts_meas[HUM_OUT].value = (float)ch_2.hum/10;
            dcts_meas[HUM_OUT].valid = TRUE;
            dcts_meas[TMPR_OUT].value = (float)ch_2.tmpr/10;
            dcts_meas[TMPR_OUT].valid = TRUE;
        }
        taskEXIT_CRITICAL();

        ch_3 = am2302_get(1);
        taskENTER_CRITICAL();
        if(ch_3.error == 1){
            ch_3_lost++;
            ch_3_lost_con_cnt++;
            if(ch_3_lost_con_cnt > 2){
                dcts_meas[HUM_IN_1].valid = FALSE;
                dcts_meas[TMPR_IN_1].valid = FALSE;
            }
            if(ch_3_lost_con_cnt > 9){
                am2302_reinit(1);
            }
        }else{
            ch_3_recieved++;
            ch_3_lost_con_cnt = 0;
            dcts_meas[HUM_IN_1].value = (float)ch_3.hum/10;
            dcts_meas[HUM_IN_1].valid = TRUE;
            dcts_meas[TMPR_IN_1].value = (float)ch_3.tmpr/10;
            dcts_meas[TMPR_IN_1].valid = TRUE;
        }
        taskEXIT_CRITICAL();

        ch_4 = am2302_get(2);
        taskENTER_CRITICAL();
        if(ch_4.error == 1){
            ch_4_lost++;
            ch_4_lost_con_cnt++;
            if(ch_4_lost_con_cnt > 2){
                dcts_meas[HUM_IN_2].valid = FALSE;
                dcts_meas[TMPR_IN_2].valid = FALSE;
            }
            if(ch_4_lost_con_cnt > 9){
                am2302_reinit(2);
            }
        }else{
            ch_4_recieved++;
            ch_4_lost_con_cnt = 0;
            dcts_meas[HUM_IN_2].value = (float)ch_4.hum/10;
            dcts_meas[HUM_IN_2].valid = TRUE;
            dcts_meas[TMPR_IN_2].value = (float)ch_4.tmpr/10;
            dcts_meas[TMPR_IN_2].valid = TRUE;
        }
        taskEXIT_CRITICAL();

        if((dcts_meas[HUM_IN_1].valid)&&(dcts_meas[HUM_IN_2].valid)){
            dcts_meas[HUM_IN_AVG].value = (dcts_meas[HUM_IN_1].value + dcts_meas[HUM_IN_2].value)/2.0f;
            dcts_meas[TMPR_IN_AVG].value = (dcts_meas[TMPR_IN_1].value + dcts_meas[TMPR_IN_2].value)/2.0f;
            dcts_meas[HUM_IN_AVG].valid = TRUE;
            dcts_meas[TMPR_IN_AVG].valid = TRUE;
        }else if(dcts_meas[HUM_IN_1].valid){
            dcts_meas[HUM_IN_AVG].value = dcts_meas[HUM_IN_1].value;
            dcts_meas[TMPR_IN_AVG].value = dcts_meas[TMPR_IN_1].value;
            dcts_meas[HUM_IN_AVG].valid = TRUE;
            dcts_meas[TMPR_IN_AVG].valid = TRUE;
        }else if(dcts_meas[HUM_IN_2].valid){
            dcts_meas[HUM_IN_AVG].value = dcts_meas[HUM_IN_2].value;
            dcts_meas[TMPR_IN_AVG].value = dcts_meas[TMPR_IN_2].value;
            dcts_meas[HUM_IN_AVG].valid = TRUE;
            dcts_meas[TMPR_IN_AVG].valid = TRUE;
        }else{
            dcts_meas[HUM_IN_AVG].valid = FALSE;
            dcts_meas[TMPR_IN_AVG].valid = FALSE;
        }

        osDelayUntil(&last_wake_time, am2302_task_period);
    }
}
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

    GPIO_InitTypeDef GPIO_InitStruct = {0};
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
 * @brief Reinit AM2302 pin
 * @param channel
 * @ingroup am2302
 */
void am2302_reinit(int channel){
    HAL_GPIO_DeInit(am2302_pin[channel].port, am2302_pin[channel].pin);

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = am2302_pin[channel].pin;
    HAL_GPIO_Init (am2302_pin[channel].port, &GPIO_InitStruct);
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
            result.tmpr = (read_data[2]*256 + read_data[3])&0x7FFF;
            if(read_data[2]&0x80){
                result.tmpr *= -1;
            }
            result.paritet = read_data[4];
        }else{
            result.error = 1;
        }
    }else{
        result.error = 1;
    }
    return result;
}

/**
 * @brief Send data like AM2302
 * @param data - struct with data
 * @param channel - channel of AM2302
 * @ingroup am2302
 */
void am2302_send(am2302_data_t data, uint8_t channel){

    uint8_t send_data[5] = {
        (uint8_t)(data.hum >> 8),
        (uint8_t)data.hum,
        (uint8_t)(data.tmpr >> 8),
        (uint8_t)data.tmpr,
        0x00};
    for (uint8_t i = 0; i < 4; i++){
        send_data[4] += send_data[i];
    }
    uint32_t t_bus_release = 30;
    uint32_t t_response = 80;
    uint32_t t_low = 50;
    uint32_t t_0 = 26;
    uint32_t t_1 = 70;
    uint32_t t_en = 50;


    // Config AM2032_PIN to OUT
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = am2302_pin[channel].pin;

    us_tim_delay(t_bus_release);

    HAL_GPIO_WritePin(am2302_pin[channel].port,am2302_pin[channel].pin, GPIO_PIN_RESET);
    HAL_GPIO_Init (am2302_pin[channel].port, &GPIO_InitStruct);
    us_tim_delay(t_response);

    HAL_GPIO_WritePin(am2302_pin[channel].port,am2302_pin[channel].pin, GPIO_PIN_SET);
    us_tim_delay(t_response);

    // Send data
    HAL_GPIO_WritePin(am2302_pin[channel].port,am2302_pin[channel].pin, GPIO_PIN_RESET);
    for(uint8_t byte = 0; byte < 5; byte++){
        for(uint8_t bit = 0; bit < 8; bit++){
            us_tim_delay(t_low);
            HAL_GPIO_WritePin(am2302_pin[channel].port,am2302_pin[channel].pin, GPIO_PIN_SET);
            if(send_data[byte] & (1 << (7 - bit))){
                us_tim_delay(t_1);
            }else{
                us_tim_delay(t_0);
            }
            HAL_GPIO_WritePin(am2302_pin[channel].port,am2302_pin[channel].pin, GPIO_PIN_RESET);
        }
    }
    us_tim_delay(t_en);
    //HAL_GPIO_WritePin(am2302_pin[channel].port,am2302_pin[channel].pin, GPIO_PIN_SET);

    // Config AM2032_PIN to IN
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init (am2302_pin[channel].port, &GPIO_InitStruct);
}


am2302_data_t am2302_get_rtc(uint8_t channel){

    am2302_data_t result = {0};
    uint32_t T = 0;
    uint8_t i = 0;
    uint8_t read_data[5] = {0};
    uint32_t current_time = 0;
    uint32_t time_left = 0;

    // Config AM2032_PIN to OUT
    GPIO_InitTypeDef GPIO_InitStruct;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Pin = am2302_pin[channel].pin;
    HAL_GPIO_Init (am2302_pin[channel].port, &GPIO_InitStruct);

    /*
    // SDA = 0
    HAL_GPIO_WritePin (am2302_pin[channel].port, am2302_pin[channel].pin, GPIO_PIN_RESET);

    // Run timeout 1ms
    //osDelay(1);
    us_tim_delay(1000);

    // SDA = 1
    HAL_GPIO_WritePin (am2302_pin[channel].port, am2302_pin[channel].pin, GPIO_PIN_SET);

    // Config AM2032_PIN to INPUT
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    HAL_GPIO_Init (am2302_pin[channel].port, &GPIO_InitStruct);*/

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

