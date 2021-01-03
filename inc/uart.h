#include "stdint.h"
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

#ifndef UART_H
#define UART_H 1

/*========== DEFINES ==========*/

#define UART_BUFF_MAX_LEN   256
#define UART_CONN_LOST_TIMEOUT 2500 //2,5 sec

/*========== TYPEDEFS ==========*/

typedef enum {
    UART_STATE_ERASE =0, /*!<erase all options*/
    UART_STATE_ENABLED = (1<<0),/*!<channel had enabled*/
    UART_STATE_RECIEVE = (1<<1),/*!<data recieve process*/
    UART_STATE_SENDING = (1<<2),/*!<native sending process for channels*/
    UART_STATE_SENDED = (1<<3),/*!<data sended while sended handle */
    UART_STATE_ERROR = (1<<4),
    UART_STATE_IN_HANDING = (1<<5),/*!<sets after receive packet before PM will handling*/
    UART_STATE_BUSY = (1<<6), /*!<sending data, channel is waiting for receive packet*/
    UART_STATE_IS_LAST_BYTE = (1<<7) ,
    UART_STATE_FREE = (~UART_STATE_IN_HANDING) & (~UART_STATE_BUSY),/*!<clear CHANNEL_STATE_IN_HANDING after PM handle*/
}chanel_state_enum_t;
typedef uint8_t chanel_state_t;

typedef enum {
    PARITY_NONE,
    PARITY_EVEN,
    PARITY_ODD,
}parity_t;

typedef enum {
    BITRATE_600 = 6,
    BITRATE_1200 = 12,
    BITRATE_2400 = 24,
    BITRATE_4800 = 48,
    BITRATE_9600 = 96,
    BITRATE_14400 = 144,
    BITRATE_19200 = 192,
    BITRATE_28800 = 288,
    BITRATE_38400 = 384,
    BITRATE_56000 = 560,
    BITRATE_57600 = 576,
    BITRATE_115200 = 1152,
    BITRATE_128000 = 1280,
    BITRATE_256000 = 2560,
}uart_bitrate_t;

typedef struct {
    uint16_t out_len;
    uint16_t out_ptr;
    uint16_t in_len;
    uint16_t in_ptr;
    uint16_t received_len;
    uint16_t max_len;
    uint16_t overrun_err_cnt;
    uint16_t parity_err_cnt;
    uint16_t frame_err_cnt;
    uint16_t noise_err_cnt;
    uint16_t timeout;
    uint32_t timeout_last;
    uint16_t conn_lost_timeout;
    uint16_t conn_last;
    chanel_state_t state;
    uint8_t* buff_out;
    uint8_t* buff_in;
    uint8_t* buff_received;
} uart_stream_t;

/*========= GLOBAL VARIABLES ==========*/

extern uint8_t uart_buff_out[];
extern uint8_t uart_buff_received[];
extern uint8_t uart_buff_in[];
extern uart_stream_t uart_2;
extern UART_HandleTypeDef huart2;

/*========== FUNCTION PROTOTYPES ==========*/

int uart_init(uart_bitrate_t bit_rate,uint8_t word_len,uint8_t stop_bit_number,parity_t parity,uint16_t rx_delay,uint16_t lost_conn_timeout);
void uart_deinit(void);
void uart_gpio_init(void);
void uart_gpio_deinit(void);
int uart_send(const uint8_t * buff,uint16_t len);
int uart_handle(void);

#endif // UART_H
