// Library for DS18B20
// Ver_1.0

/*========== LIBRARY DESCRIPTION ==========
- Library use STM32F3xx_HAL_Driver
*/

#ifndef ds18b20_H_
#define ds18b20_H_
#include "stm32f1xx_hal.h"

#define SEARCH_ROM      0xF0
#define READ_ROM        0x33
#define MATCH_ROM       0x55
#define SKIP_ROM        0xCC
#define ALRM_SEARCH     0xEC
// DS18B20 commands
#define START_CONV_T    0x44
#define WRITE_SCRPAD    0x4E
#define READ_SCRPAD     0xBE
#define COPY_SCRPAD     0x48
#define RECALL          0xB8
#define READ_PWR        0xB4

#define TH_ALRM         0x00
#define TL_ALRM         0x00
#define RESOLUTION      12 // 9, 10, 11 or 12

#if(RESOLUTION == 12)
#define CONF            0x7F
#define DS18B20_RESOLUTION 0.0625f
#define DS18B20_CONV_MS 750
#elif(RESOLUTION == 11)
#define CONF            0x5F
#define DS18B20_RESOLUTION 0.125f
#define DS18B20_CONV_MS 375
#elif(RESOLUTION == 10)
#define CONF            0x3F
#define DS18B20_RESOLUTION 0.25f
#define DS18B20_CONV_MS 188
#elif(RESOLUTION == 9)
#define CONF            0x1F
#define DS18B20_RESOLUTION 0.5f
#define DS18B20_CONV_MS 94
#else
#error(Please select DS18B20 ADC resolution)
#endif // RESOLUTION


/*========== TYPEDEFS ==========*/

/**
  * @brief Struct for DS18B20 data
  * @ingroup ds18b20
  */
typedef struct {
  float tmpr;
  uint8_t valid;
} ds18b20_data_t;

//========== VARIABLES ==========

//========== FUNCTIONS PROTOTYPES ==========

int ds18b20_init (void);
void ds18b20_deinit(void);
void ds18b20_task (void const * argument);
ds18b20_data_t ds18b20_get(void);
void ds18b20_start_conv(void);

#endif /* ds18b20_H_ */
