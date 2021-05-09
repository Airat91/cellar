#include "adc.h"
#include "pin_map.h"
#include "dcts.h"
#include "dcts_config.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_rcc.h"
#include <math.h>
#include "main.h"
/**
  * @defgroup ADC
  * @brief work with ADC channels
  */

ADC_HandleTypeDef hadc1;

#define ADC_BUF_SIZE 50
#define WTR_LVL_BUF_SIZE 10
#define ADC_PERIOD 100
#define ADC_MAX 4095
#define ADC_VREF 3.3f
#define INPUT_RES 10000.0f

#define PWR_K   (float)10.1
#define VREF_INT (float)1.2

/*========== FUNCTIONS ==========*/

/**
 * @brief Init and start ADC
 * @return  0 - ADC init successfull,\n
 *          -1 - ADC config error,\n
 *          -2 - PWR channel config error,\n
 *          -3 - WTR_LEV channel config error,\n
 *          -4 - WTR_TMP channel config error,\n
 *          -5 - TMP channel config error,\n
 *          -6 - ADC start error,
 * @ingroup ADC
 */
int adc_init (void){
    int result = 0;
    __HAL_RCC_ADC1_CLK_ENABLE();
    adc_gpio_init();
    ADC_InjectionConfTypeDef sConfigInjected = {0};

    //Common config
    hadc1.Instance = ADC1;
    hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
    hadc1.Init.ContinuousConvMode = ENABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion = 1;
    if (HAL_ADC_Init(&hadc1) != HAL_OK)
    {
        result = -1;
    }

    sConfigInjected.InjectedNbrOfConversion = 4;
    sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    sConfigInjected.ExternalTrigInjecConv = ADC_INJECTED_SOFTWARE_START;
    sConfigInjected.AutoInjectedConv = ENABLE;
    sConfigInjected.InjectedDiscontinuousConvMode = DISABLE;
    sConfigInjected.InjectedOffset = 0;

    //Configure PWR Channel
    sConfigInjected.InjectedChannel = ADC_CHANNEL_4;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_1;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        result = -2;
    }
    //Configure WTR_MIN Channel
    sConfigInjected.InjectedChannel = input_ch[0].adc_channel;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_2;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        result = -3;
    }
    //Configure WTR_MAX Channel
    sConfigInjected.InjectedChannel = input_ch[1].adc_channel;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_3;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        result = -4;
    }
    //Configure VREF Channel
    sConfigInjected.InjectedChannel = ADC_CHANNEL_VREFINT;
    sConfigInjected.InjectedRank = ADC_INJECTED_RANK_4;
    if (HAL_ADCEx_InjectedConfigChannel(&hadc1, &sConfigInjected) != HAL_OK)
    {
        result = -5;
    }
    //Start ADC
    if (HAL_ADC_Start(&hadc1) != HAL_OK){
        result = -6;
    }

    return result;
}
/**
 * @brief Deinit ADC
 * @ingroup ADC
 */
void adc_deinit (void){
    HAL_ADC_Stop(&hadc1);
    HAL_ADC_DeInit(&hadc1);
    __HAL_RCC_ADC1_CLK_DISABLE();
    adc_gpio_deinit();
}
/**
 * @brief Init ADC gpio
 * @ingroup ADC
 */
void adc_gpio_init (void){
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pin = PWR_PIN;
    HAL_GPIO_Init(PWR_PORT, &GPIO_InitStruct);
}
/**
 * @brief Deinit ADC gpio
 * @ingroup ADC
 */
void adc_gpio_deinit (void){
    HAL_GPIO_DeInit(PWR_PORT,PWR_PIN);
}
/**
 * @brief Measure ADC channels and write values to DCTS
 * @param argument - none
 * @ingroup ADC
 */
void adc_task(void const * argument){
    (void)argument;
    uint16_t pwr[ADC_BUF_SIZE];
    uint16_t wtr_min[WTR_LVL_BUF_SIZE];
    uint16_t wtr_max[WTR_LVL_BUF_SIZE];
    uint16_t vref[ADC_BUF_SIZE];
    uint8_t tick = 0;
    uint8_t wtr_lvl_tick = 0;
    float temp = 0.0f;
    float v_3_3 = 0.0f;
    adc_init();
    uint32_t last_wake_time = osKernelSysTick();
    while(1){
        uint32_t pwr_sum = 0;
        uint32_t wtr_min_sum = 0;
        uint32_t wtr_max_sum = 0;
        uint32_t vref_sum = 0;


        pwr[tick] = (uint16_t)hadc1.Instance->JDR1;
        wtr_min[wtr_lvl_tick] = (uint16_t)hadc1.Instance->JDR2;
        wtr_max[wtr_lvl_tick] = (uint16_t)hadc1.Instance->JDR3;
        vref[tick] = (uint16_t)hadc1.Instance->JDR4;
        v_3_3 = VREF_INT/vref[tick]*ADC_MAX;

        for(uint8_t i = 0; i < ADC_BUF_SIZE; i++){
            pwr_sum += pwr[i];
            vref_sum += vref[i];
        }

        for(uint8_t i = 0; i < WTR_LVL_BUF_SIZE; i++){
            wtr_min_sum += wtr_min[i];
            wtr_max_sum += wtr_max[i];
        }

        temp = (float)pwr_sum/ADC_BUF_SIZE;
        taskENTER_CRITICAL();
        dcts.dcts_pwr = temp/ADC_MAX*ADC_VREF*PWR_K;

        dcts_meas[WTR_MIN_ADC].value = (float)wtr_min_sum/WTR_LVL_BUF_SIZE;
        dcts_meas[WTR_MIN_VLT].value = dcts_meas[WTR_MIN_ADC].value*v_3_3/ADC_MAX;
        dcts_meas[WTR_MIN_RES].value = dcts_meas[WTR_MIN_VLT].value*INPUT_RES/(v_3_3 -  dcts_meas[WTR_MIN_VLT].value);

        dcts_meas[WTR_MAX_ADC].value = (float)wtr_max_sum/WTR_LVL_BUF_SIZE;
        dcts_meas[WTR_MAX_VLT].value = dcts_meas[WTR_MAX_ADC].value*v_3_3/ADC_MAX;
        dcts_meas[WTR_MAX_RES].value = dcts_meas[WTR_MAX_VLT].value*INPUT_RES/(v_3_3 -  dcts_meas[WTR_MAX_VLT].value);

        dcts_meas[VREF_VLT].value = v_3_3;

        dcts_meas[WTR_MIN_ADC].valid = TRUE;
        dcts_meas[WTR_MIN_VLT].valid = TRUE;
        dcts_meas[WTR_MIN_RES].valid = TRUE;
        dcts_meas[WTR_MAX_ADC].valid = TRUE;
        dcts_meas[WTR_MAX_VLT].valid = TRUE;
        dcts_meas[WTR_MAX_RES].valid = TRUE;
        dcts_meas[VREF_VLT].valid = TRUE;
        taskEXIT_CRITICAL();

        tick++;
        wtr_lvl_tick++;
        if(tick >= ADC_BUF_SIZE){
            tick = 0;
        }
        if(wtr_lvl_tick >= WTR_LVL_BUF_SIZE){
            wtr_lvl_tick = 0;
        }
        osDelayUntil(&last_wake_time, ADC_PERIOD);
    }
}
