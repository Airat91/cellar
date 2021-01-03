#include "uart.h"
#include "main.h"
#include "pin_map.h"
#include "dcts.h"
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
UART_HandleTypeDef huart2;
uart_stream_t uart_2 = {0};

/*========== FUNCTIONS ==========*/

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
    uart_2.out_len = 0;
    uart_2.out_ptr = 0;
    uart_2.in_len = 0;
    uart_2.in_ptr = 0;
    uart_2.received_len = 0;
    uart_2.max_len = UART_BUFF_MAX_LEN;
    uart_2.overrun_err_cnt = 0;
    uart_2.parity_err_cnt = 0;
    uart_2.frame_err_cnt = 0;
    uart_2.noise_err_cnt = 0;
    uart_2.timeout_last = 0;
    uart_2.conn_lost_timeout = 10000;
    uart_2.conn_last = 0;
    uart_2.state = UART_STATE_ERASE;
    uart_2.buff_out = uart_buff_out;
    uart_2.buff_in = uart_buff_in;
    uart_2.buff_received = uart_buff_received;
    __HAL_RCC_USART2_CLK_ENABLE();
    uart_gpio_init();

    uart_2.timeout = rx_delay;

    huart2.Instance = USART2;
    huart2.Init.BaudRate = bit_rate*100;
    switch (word_len) {
    case 8:
        huart2.Init.WordLength = UART_WORDLENGTH_8B;
        break;
    case 9:
        huart2.Init.WordLength = UART_WORDLENGTH_9B;
        break;
    default:
        huart2.Init.WordLength = UART_WORDLENGTH_8B;
        result = -2;
    }
    switch (stop_bit_number) {
    case 1:
        huart2.Init.StopBits = UART_STOPBITS_1;
        break;
    case 2:
        huart2.Init.StopBits = UART_STOPBITS_2;
        break;
    default:
        huart2.Init.StopBits = UART_STOPBITS_1;
        result = -3;
    }
    switch (parity) {
    case PARITY_NONE:
        huart2.Init.Parity = UART_PARITY_NONE;
        break;
    case PARITY_EVEN:
        huart2.Init.Parity = UART_PARITY_EVEN;
        break;
    case PARITY_ODD:
        huart2.Init.Parity = UART_PARITY_ODD;
        break;
    default:
        huart2.Init.Parity = UART_PARITY_NONE;
        result = -4;
    }
    huart2.Init.Mode = UART_MODE_TX_RX;
    huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart2) != HAL_OK)
    {
      result = -1;
    }

    uint16_t i = 0x0FFF; //delay for debug
    while(i){
        i--;
    }

    huart2.Instance->CR1 |= USART_CR1_RXNEIE;   // ready to input messages
    uart_2.state |= UART_STATE_ENABLED;
    NVIC_ClearPendingIRQ(USART2_IRQn);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

    return result;
}

/**
 * @brief Disable UART clk
 * @ingroup uart
 */
void uart_deinit(){
    __HAL_RCC_USART2_CLK_DISABLE();
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
    if(len <= uart_2.max_len){
        while((uart_2.state & UART_STATE_SENDING)&&(wait < uart_2.timeout)){
              wait = (us_tim_get_value() - time);
        }
        if((uart_2.state & UART_STATE_SENDING) == 0){
            uart_2.state |= UART_STATE_SENDING;
            HAL_GPIO_WritePin(RS_485_DE_PORT, RS_485_DE_PIN,GPIO_PIN_SET);
            taskENTER_CRITICAL();
            memcpy(uart_2.buff_out,buff,len);
            len = (len<uart_2.max_len)?len:uart_2.max_len-1;
            uart_2.out_len = len;
            huart2.Instance->CR1 &= ~USART_CR1_RXNEIE;  // not ready to input messages
            huart2.Instance->CR1 &= ~USART_CR1_TCIE;
            huart2.Instance->SR &= ~USART_SR_TC;
            uart_2.out_ptr = 0;
            huart2.Instance->CR1 |= USART_CR1_TXEIE;
            //huart2.Instance->DR = uart_2.buff_out[0];
            huart2.Instance->SR |= USART_SR_TXE;
            taskEXIT_CRITICAL();
        }else if(wait > uart_2.timeout){
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
    uint32_t status;
    dd=0;
    status = huart2.Instance->SR;
    // receive mode
    if((status & USART_SR_RXNE)&&!(uart_2.state &= UART_STATE_SENDING)){
        huart2.Instance->SR &= ~(USART_SR_RXNE);
        dd=huart2.Instance->DR;
        uart_2.state |= UART_STATE_RECIEVE;
        uart_2.timeout_last = us_tim_get_value();
        if(uart_2.in_ptr < uart_2.max_len &&\
                ((uart_2.in_ptr!=0) || (dd!=0))){
            uart_2.buff_in[uart_2.in_ptr++]=(uint8_t)dd;
        }

        // check for errors
        if (status & USART_SR_PE){// Parity error
            uart_2.state |= UART_STATE_ERROR;
            dd = huart2.Instance->SR;
            dd = huart2.Instance->DR;
            uart_2.parity_err_cnt++;
        } else if (status & USART_SR_FE){//frame error
            uart_2.state |= UART_STATE_ERROR;
            dd = huart2.Instance->SR;
            dd = huart2.Instance->DR;
            uart_2.frame_err_cnt++;
        } else if (status & USART_SR_NE){//noise detected
            uart_2.state |= UART_STATE_ERROR;
            dd = huart2.Instance->SR;
            dd = huart2.Instance->DR;
            uart_2.noise_err_cnt++;
        }
    }
    // transmit mode
    if((status & USART_SR_TXE) ){
        if(!(uart_2.state & UART_STATE_IS_LAST_BYTE)){
            if(uart_2.out_ptr>=uart_2.max_len){
                uart_2.out_ptr = uart_2.max_len-1;
            }
            if(uart_2.out_ptr == uart_2.out_len-1){
                huart2.Instance->CR1 &= ~USART_CR1_TXEIE;
                huart2.Instance->CR1 |= USART_CR1_TCIE;
                huart2.Instance->SR &=~USART_SR_TC;
                uart_2.state |= UART_STATE_IS_LAST_BYTE;
                huart2.Instance->DR=uart_2.buff_out[uart_2.out_ptr];
            }else {
                huart2.Instance->DR=uart_2.buff_out[uart_2.out_ptr];
            }
            uart_2.out_ptr++;
            /*if(uart_2.out_ptr > uart_2.out_len){
                uart_2.err_cnt++;
            }*/
        }else if(status & USART_SR_TC){
            // end of transmit
            huart2.Instance->CR1 &= ~USART_CR1_TXEIE;
            huart2.Instance->CR1 &= ~USART_CR1_TCIE;
            huart2.Instance->SR &=~USART_SR_TC;
            uart_2.in_ptr = 0;
            uart_2.out_ptr = 0;
            huart2.Instance->CR1 |= USART_CR1_RXNEIE;   // ready to input messages
            huart2.Instance->SR &= ~(USART_SR_RXNE);
            HAL_GPIO_WritePin(RS_485_DE_PORT, RS_485_DE_PIN, GPIO_PIN_RESET);
            uart_2.state |= UART_STATE_SENDED;
            uart_2.state &=~UART_STATE_IS_LAST_BYTE;
            uart_2.state &=~UART_STATE_SENDING;
        }
    }
    // overrun error without RXNE flag
    if(status & USART_SR_ORE){
        uart_2.state |= UART_STATE_ERROR;
        uart_2.overrun_err_cnt++;
        dd=huart2.Instance->SR;
        dd=huart2.Instance->DR;
    }
    return result;
}
