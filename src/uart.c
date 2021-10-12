#include "uart.h"
#include "main.h"
#include "pin_map.h"
#include "dcts.h"
#include "modbus.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "string.h"
#include "stm32f1xx_hal_uart.h"
#include "stm32f1xx_hal_usart.h"
#include "stm32f1xx_hal_rcc.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal.h"

/**
  * @defgroup uart
  * @brief work with uart for cross-device communication
  */


/*========= GLOBAL VARIABLES ==========*/

uint8_t uart_buff_out[UART_BUFF_MAX_LEN];
uint8_t uart_buff_received[UART_BUFF_MAX_LEN];
uint8_t uart_buff_in[UART_BUFF_MAX_LEN];
UART_HandleTypeDef huart1;
uart_stream_t uart_1 = {0};

/*========== FUNCTIONS ==========*/

#define uart_task_period 5
void uart_task(void const * argument){
    (void)argument;
    uart_init(config.params.mdb_bitrate, 8, 1, PARITY_NONE, 10000, UART_CONN_LOST_TIMEOUT);
    uint16_t tick = 0;
    char string[100];
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        if((uart_1.state & UART_STATE_RECIEVE)&&\
                ((uint16_t)(us_tim_get_value() - uart_1.timeout_last) > uart_1.timeout)){
            memcpy(uart_1.buff_received, uart_1.buff_in, uart_1.in_ptr);
            uart_1.received_len = uart_1.in_ptr;
            uart_1.in_ptr = 0;
            uart_1.state &= ~UART_STATE_RECIEVE;
            uart_1.state &= ~UART_STATE_ERROR;
            uart_1.state |= UART_STATE_IN_HANDING;
            uart_1.conn_last = 0;
            uart_1.recieved_cnt ++;

            if(modbus_packet_for_me(uart_1.buff_received, uart_1.received_len)){
                uint16_t new_len = modbus_rtu_packet(uart_1.buff_received, uart_1.received_len);
                uart_send(uart_1.buff_received, new_len);
                uart_1.state &= ~UART_STATE_IN_HANDING;
            }
            if(uart_1.state & UART_STATE_IN_HANDING){
                dcts_packet_handle(uart_1.buff_received, uart_1.received_len);
            }else{
                uart_1.state &= ~UART_STATE_IN_HANDING;
            }
        }
        if((uart_1.conn_last > uart_1.conn_lost_timeout)||(uart_1.overrun_err_cnt > 2)){
            uart_deinit();
            uart_init(config.params.mdb_bitrate, 8, 1, PARITY_NONE, 10000, UART_CONN_LOST_TIMEOUT);
        }
        if(tick == 1000/uart_task_period){
            tick = 0;
            //HAL_GPIO_TogglePin(LED_PORT,LED_PIN);
            /*for(uint8_t i = 0; i < MEAS_NUM; i++){
                sprintf(string, "%s:\t%.1f(%s)\n",dcts_meas[i].name,(double)dcts_meas[i].value,dcts_meas[i].unit);
                if(i == MEAS_NUM - 1){
                    strncat(string,"\n",1);
                }
                uart_send((uint8_t*)string,(uint16_t)strlen(string));
            }
            sprintf(string, "test\n");
            uart_send((uint8_t*)string,(uint16_t)strlen(string));*/

        }else{
            tick++;
            uart_1.conn_last += uart_task_period;
        }

        osDelayUntil(&last_wake_time, uart_task_period);
    }
}
/**
 * @brief Init UART
 * @param bit_rate
 * @param word_len
 * @param stop_bit_number
 * @param parity
 * @param rx_delay
 * @return  0 - UART init successfull,\n
 *          -1 - UART init error,\n
 *          -2 - word_len error,\n
 *          -3 - stop+bit_number error,\n
 *          -4 - parity error
 * @ingroup uart
 *
 * Init UART's GPIOs, config UART params and eanble UART_IRQ
 */
int uart_init(uart_bitrate_t bit_rate,uint8_t word_len,uint8_t stop_bit_number,parity_t parity,uint16_t rx_delay,uint16_t lost_conn_timeout){
    int result = 0;
    uart_1.out_len = 0;
    uart_1.out_ptr = 0;
    uart_1.in_len = 0;
    uart_1.in_ptr = 0;
    uart_1.received_len = 0;
    uart_1.max_len = UART_BUFF_MAX_LEN;
    uart_1.recieved_cnt = 0;
    uart_1.send_cnt = 0;
    uart_1.overrun_err_cnt = 0;
    uart_1.parity_err_cnt = 0;
    uart_1.frame_err_cnt = 0;
    uart_1.noise_err_cnt = 0;
    uart_1.tx_err_cnt = 0;
    uart_1.timeout_last = 0;
    uart_1.conn_lost_timeout = 10000;
    uart_1.conn_last = 0;
    uart_1.state = UART_STATE_ERASE;
    uart_1.buff_out = uart_buff_out;
    uart_1.buff_in = uart_buff_in;
    uart_1.buff_received = uart_buff_received;
    __HAL_RCC_USART1_CLK_ENABLE();
    uart_gpio_init();

    uart_1.timeout = rx_delay;

    huart1.Instance = USART1;
    huart1.Init.BaudRate = bit_rate*100;
    switch (word_len) {
    case 8:
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case 9:
        huart1.Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        huart1.Init.WordLength = UART_WORDLENGTH_8B;
        result = -2;
    }
    switch (stop_bit_number) {
    case 1:
        huart1.Init.StopBits = UART_STOPBITS_1;
        break;
    case 2:
        huart1.Init.StopBits = UART_STOPBITS_2;
        break;
    default:
        huart1.Init.StopBits = UART_STOPBITS_1;
        result = -3;
    }
    switch (parity) {
    case PARITY_NONE:
        huart1.Init.Parity = UART_PARITY_NONE;
        break;
    case PARITY_EVEN:
        huart1.Init.Parity = UART_PARITY_EVEN;
        break;
    case PARITY_ODD:
        huart1.Init.Parity = UART_PARITY_ODD;
        break;
    default:
        huart1.Init.Parity = UART_PARITY_NONE;
        result = -4;
    }
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
      result = -1;
    }

    uint16_t i = 0x0FFF; //delay for debug
    while(i){
        i--;
    }

    huart1.Instance->CR1 |= USART_CR1_RXNEIE;   // ready to input messages
    uart_1.state |= UART_STATE_ENABLED;
    NVIC_ClearPendingIRQ(USART1_IRQn);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

    return result;
}

/**
 * @brief Disable UART clk
 * @ingroup uart
 */
void uart_deinit(){
    __HAL_RCC_USART1_CLK_DISABLE();
}

/**
 * @brief Init UART's GPIOs
 * @ingroup uart
 */
void uart_gpio_init(void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Pull = GPIO_PULLUP;

    GPIO_InitStruct.Pin = RS_485_TX_PIN;
    HAL_GPIO_Init(RS_485_TX_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Mode = GPIO_MODE_AF_INPUT;
    GPIO_InitStruct.Pin = RS_485_RX_PIN;
    HAL_GPIO_Init(RS_485_RX_PORT, &GPIO_InitStruct);


    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_PULLDOWN;
    GPIO_InitStruct.Pin = RS_485_DE_PIN;
    HAL_GPIO_WritePin(RS_485_DE_PORT, RS_485_DE_PIN, GPIO_PIN_RESET);
    HAL_GPIO_Init(RS_485_DE_PORT, &GPIO_InitStruct);
}

/**
 * @brief Deinit UART's GPIOs
 * @ingroup uart
 */
void uart_gpio_deinit(void){
    HAL_GPIO_DeInit(RS_485_DE_PORT,RS_485_DE_PIN);
    HAL_GPIO_DeInit(RS_485_RX_PORT,RS_485_RX_PIN);
    HAL_GPIO_DeInit(RS_485_TX_PORT,RS_485_TX_PIN);
}

/**
 * @brief Send data by UART
 * @param buff - databuffer for sending
 * @param len - size in bytes
 * @return  0 - OK,\n
 *          -1 - oversize error,\n
 *          -2 - timeout error
 * @ingroup uart
 */
int uart_send(const uint8_t * buff,uint16_t len){
    int result = 0;
    uint32_t time = us_tim_get_value();
    uint32_t wait = us_tim_get_value() - time;
    if(len <= uart_1.max_len){
        while((uart_1.state & UART_STATE_SENDING)&&(wait < uart_1.timeout)){
              wait = (us_tim_get_value() - time);
        }
        if((uart_1.state & UART_STATE_SENDING) == 0){
            uart_1.state |= UART_STATE_SENDING;
            uart_1.state &= ~UART_STATE_SENDED;
            HAL_GPIO_WritePin(RS_485_DE_PORT, RS_485_DE_PIN,GPIO_PIN_SET);
            taskENTER_CRITICAL();
            memcpy(uart_1.buff_out,buff,len);
            len = (len<uart_1.max_len)?len:uart_1.max_len-1;
            uart_1.out_len = len;
            huart1.Instance->CR1 &= ~USART_CR1_RXNEIE;  // not ready to input messages
            huart1.Instance->CR1 &= ~USART_CR1_TCIE;
            huart1.Instance->SR &= ~USART_SR_TC;
            uart_1.out_ptr = 0;
            huart1.Instance->CR1 |= USART_CR1_TXEIE;
            //huart1.Instance->SR &= ~USART_SR_TXE;
            //huart1.Instance->DR = uart_1.buff_out[0];
            huart1.Instance->SR |= USART_SR_TXE;
            taskEXIT_CRITICAL();
            uart_1.send_cnt ++;
        }else if(wait > uart_1.timeout){
            result = -2;
        }
    }else{
        result = -1;
    }
    return result;
}

/**
 * @brief UART interrupt handle
 * @return  0 - OK
 * @ingroup uart
 */
int uart_handle(void){
    int result = 0;
    uint32_t dd;
    uint32_t status, control;
    uint32_t state;
    dd=0;
    status = huart1.Instance->SR;
    control = huart1.Instance->CR1;
    state = uart_1.state;
    // receive mode
    if((status & USART_SR_RXNE)&&!(uart_1.state & UART_STATE_SENDING)){
        huart1.Instance->SR &= ~(USART_SR_RXNE);
        dd=huart1.Instance->DR;
        uart_1.state |= UART_STATE_RECIEVE;
        uart_1.timeout_last = us_tim_get_value();
        if(uart_1.in_ptr < uart_1.max_len &&\
                ((uart_1.in_ptr!=0) || (dd!=0))){
            uart_1.buff_in[uart_1.in_ptr++]=(uint8_t)dd;
        }

        // check for errors
        if (status & USART_SR_PE){// Parity error
            uart_1.state |= UART_STATE_ERROR;
            dd = huart1.Instance->SR;
            dd = huart1.Instance->DR;
            uart_1.parity_err_cnt++;
        } else if (status & USART_SR_FE){//frame error
            uart_1.state |= UART_STATE_ERROR;
            dd = huart1.Instance->SR;
            dd = huart1.Instance->DR;
            uart_1.frame_err_cnt++;
        } else if (status & USART_SR_NE){//noise detected
            uart_1.state |= UART_STATE_ERROR;
            dd = huart1.Instance->SR;
            dd = huart1.Instance->DR;
            uart_1.noise_err_cnt++;
        }
    }else
    // transmit mode
    if((status & USART_SR_TXE)&&(uart_1.state & UART_STATE_SENDING)&&(uart_1.out_len > 0)){
        if(!(uart_1.state & UART_STATE_IS_LAST_BYTE)){
            if(uart_1.out_ptr>=uart_1.max_len){
                uart_1.out_ptr = uart_1.max_len-1;
            }
            if(uart_1.out_ptr == uart_1.out_len-1){
                huart1.Instance->CR1 &= ~USART_CR1_TXEIE;
                huart1.Instance->CR1 |= USART_CR1_TCIE;
                huart1.Instance->SR &=~USART_SR_TC;
                uart_1.state |= UART_STATE_IS_LAST_BYTE;
            }
            huart1.Instance->DR=uart_1.buff_out[uart_1.out_ptr];
            uart_1.out_ptr++;
            if(uart_1.out_ptr > uart_1.out_len){
                uart_1.out_ptr = 0;
                uart_1.tx_err_cnt++;
            }
        }else if((status & USART_SR_TC)&&(uart_1.state & UART_STATE_IS_LAST_BYTE)){
            // end of transmit
            huart1.Instance->CR1 &= ~USART_CR1_TXEIE;
            huart1.Instance->CR1 &= ~USART_CR1_TCIE;
            huart1.Instance->SR &=~USART_SR_TC;
            uart_1.in_ptr = 0;
            uart_1.out_ptr = 0;
            HAL_GPIO_WritePin(RS_485_DE_PORT, RS_485_DE_PIN, GPIO_PIN_RESET);   // ready to input messages
            huart1.Instance->SR &= ~(USART_SR_RXNE);
            uart_1.state |= UART_STATE_SENDED;
            uart_1.state &=~UART_STATE_IS_LAST_BYTE;
            uart_1.state &=~UART_STATE_SENDING;
            huart1.Instance->CR1 |= USART_CR1_RXNEIE;
        }else{
            //transmit error
            uart_1.tx_err_cnt++;
        }
    }else
    // overrun error without RXNE flag
    if(status & USART_SR_ORE){
        uart_1.state |= UART_STATE_ERROR;
        uart_1.overrun_err_cnt++;
        dd=huart1.Instance->SR;
        dd=huart1.Instance->DR;
    }else{
        uart_1.tx_err_cnt++;
        huart1.Instance->CR1 &= ~USART_CR1_TXEIE;
        huart1.Instance->CR1 &= ~USART_CR1_TCIE;
        huart1.Instance->SR &=~USART_SR_TC;
        uart_1.in_ptr = 0;
        uart_1.out_ptr = 0;
        HAL_GPIO_WritePin(RS_485_DE_PORT, RS_485_DE_PIN, GPIO_PIN_RESET);   // ready to input messages
        huart1.Instance->SR &= ~(USART_SR_RXNE);
        uart_1.state = UART_STATE_ENABLED;
        huart1.Instance->CR1 |= USART_CR1_RXNEIE;
    }
    return result;
}
