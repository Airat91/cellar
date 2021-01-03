/**
 * @file ds18.c
 * @author Shoma Gane <shomagan@gmail.com>
 *         Ayrat Girfanov <girfanov.ayrat@yandex.ru>
 * @defgroup src
 * @ingroup src
 * @version 0.1 
 * @brief  TODO!!! write brief in 
 */
/*
 * Copyright (c) 2018 Snema Service
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 * SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT
 * OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 * This file is part of the sofi PLC.
 *
 * Author: Shoma Gane <shomagan@gmail.com>
 *         Ayrat Girfanov <girfanov.ayrat@yandex.ru>
 */
#ifndef DS18_C
#define DS18_C 1
#include "ds18.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "cmsis_os.h"
//#include "usbd_cdc_if.h"
extern IWDG_HandleTypeDef hiwdg;
Ds18b20Sensor_t	ds18b20[_DS18B20_MAX_SENSORS];

static OneWire_t one_wire;
static uint8_t	  one_wire_devices;

static uint8_t		ds18b20_start_converter=0;
static uint16_t	ds18b20_timeout=0;
static osThreadId 	Ds18b20Handle;
#define MAX_ERROR_READ_COUNTER 10
static uint8_t error_read_counter = 0;
static uint8_t find_device(void);
uint8_t 	temp_sensor_count=0;
uint8_t data_valid =0;
void ds18_task( const void *parameters){
    TickType_t ds18_time;
    (void)parameters;
    ds18_time = xTaskGetTickCount();
    //infinity find device
    find_device();
    error_read_counter = 0;
    while(1){
        if(error_read_counter >= MAX_ERROR_READ_COUNTER ){
            find_device();
            error_read_counter = 0;
            data_valid = 0;
        }
        ds18b20_timeout=_DS18B20_CONVERT_TIMEOUT_MS;
        ds18b20_start_calc(&one_wire);
        while (!ds18b20_calc_done(&one_wire)){
            osDelay(1);
            ds18b20_timeout-=1;
            if(ds18b20_timeout==0)
				break;
		}	
        if(ds18b20_timeout>0){
            for (uint8_t i = 0; i < temp_sensor_count; i++){
                ds18b20[i].data_validate = ds18b20_read(&one_wire, ds18b20[i].Address, &ds18b20[i].Temperature);
                if(!ds18b20[i].data_validate){
                    error_read_counter++;
                }else{
                    error_read_counter=0;
                    data_valid = 1;
                }
			}
		}else{
            error_read_counter++;
            for (uint8_t i = 0; i < temp_sensor_count; i++){
                ds18b20[i].data_validate = false;
            }
		}
        ds18b20_start_converter=0;
        HAL_IWDG_Refresh(&hiwdg);
        osDelay(_DS18B20_UPDATE_INTERVAL_MS);
	}
}
/*@brief infinity cycle while did't find device
 *
 * */
static uint8_t find_device(){
    do	{
        one_wire_init(&one_wire,_DS18B20_GPIO ,_DS18B20_PIN);
        temp_sensor_count = 0;
        while(HAL_GetTick() < 3000){
            osDelay(100);
        }
        one_wire_devices = one_wire_first(&one_wire);
        while (one_wire_devices){
            osDelay(100);
            temp_sensor_count++;
            one_wire_get_full_rom(&one_wire, ds18b20[temp_sensor_count-1].Address);
            one_wire_devices = one_wire_next(&one_wire);
        }
        if(temp_sensor_count>0){
            break;
        }
        HAL_IWDG_Refresh(&hiwdg);
    }while(temp_sensor_count==0);
    for (uint8_t i = 0; i < temp_sensor_count; i++){
        osDelay(50);
        ds18b20_set_resolution(&one_wire, ds18b20[i].Address, DS18B20_Resolution_12bits);
        osDelay(50);
        ds18b20_disable_alarm_temperature(&one_wire,  ds18b20[i].Address);
    }

    return temp_sensor_count;
}

//###########################################################################################
bool	Ds18b20_ManualConvert(void){
    ds18b20_start_converter=1;
    while(ds18b20_start_converter==1)
		osDelay(10);
    if(ds18b20_timeout==0)
		return false;
	else
		return true;	
}
//###########################################################################################
void Task_Ds18b20(void const * argument){
}
//###########################################################################################
uint8_t DS18B20_Start(OneWire_t* OneWire, uint8_t *ROM){
	/* Check if device is DS18B20 */
	if (!DS18B20_Is(ROM)) {
		return 0;
	}
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Start temperature conversion */
	OneWire_WriteByte(OneWire, DS18B20_CMD_CONVERTTEMP);
	return 1;
}

void ds18b20_start_calc(OneWire_t* OneWire){
	/* Reset pulse */
	OneWire_Reset(OneWire);
	/* Skip rom */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_SKIPROM);
	/* Start conversion on all connected devices */
	OneWire_WriteByte(OneWire, DS18B20_CMD_CONVERTTEMP);
}

bool ds18b20_read(OneWire_t* OneWire, uint8_t *ROM, float *destination) {
	uint16_t temperature;
	uint8_t resolution;
	int8_t digit, minus = 0;
	float decimal;
	uint8_t i = 0;
	uint8_t data[9];
	uint8_t crc;
	/* Check if device is DS18B20 */
    taskENTER_CRITICAL();
	if (!DS18B20_Is(ROM)) {
        taskEXIT_CRITICAL();
		return false;
	}
	/* Check if line is released, if it is, then conversion is complete */
	if (!OneWire_ReadBit(OneWire)) 	{
		/* Conversion is not finished yet */
        taskEXIT_CRITICAL();
		return false; 
	}
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Read scratchpad command by onewire protocol */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);
	
	/* Get data */
	for (i = 0; i < 9; i++) {
		/* Read byte by byte */
		data[i] = OneWire_ReadByte(OneWire);
	}
	/* Calculate CRC */
	crc = OneWire_CRC8(data, 8);
	/* Check if CRC is ok */
	if (crc != data[8]){
		/* CRC invalid */
        taskEXIT_CRITICAL();
		return 0;}
    taskEXIT_CRITICAL();
	/* First two bytes of scratchpad are temperature values */
	temperature = data[0] | (data[1] << 8);
	/* Reset line */

	OneWire_Reset(OneWire);
	/* Check if temperature is negative */
	if (temperature & 0x8000){
		/* Two's complement, temperature is negative */
		temperature = ~temperature + 1;
		minus = 1;
	}
	/* Get sensor resolution */
	resolution = ((data[4] & 0x60) >> 5) + 9;
	/* Store temperature integer digits and decimal digits */
	digit = temperature >> 4;
	digit |= ((temperature >> 8) & 0x7) << 4;
	/* Store decimal digits */
	switch (resolution) {
		case 9:
			decimal = (temperature >> 3) & 0x01;
			decimal *= (float)DS18B20_DECIMAL_STEPS_9BIT;
		break;
		case 10:
			decimal = (temperature >> 2) & 0x03;
			decimal *= (float)DS18B20_DECIMAL_STEPS_10BIT;
		 break;
		case 11: 
			decimal = (temperature >> 1) & 0x07;
			decimal *= (float)DS18B20_DECIMAL_STEPS_11BIT;
		break;
		case 12: 
			decimal = temperature & 0x0F;
			decimal *= (float)DS18B20_DECIMAL_STEPS_12BIT;
		 break;
		default: 
			decimal = 0xFF;
			digit = 0;
	}
	/* Check for negative part */
	decimal = digit + decimal;
	if (minus) {
		decimal = 0 - decimal;}
	/* Set to pointer */
	*destination = decimal;
	/* Return 1, temperature valid */
	return true;
}

uint8_t DS18B20_GetResolution(OneWire_t* OneWire, uint8_t *ROM){
	uint8_t conf;
	if (!DS18B20_Is(ROM)) {
		return 0;}
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Read scratchpad command by onewire protocol */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);
	/* Ignore first 4 bytes */
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);
	/* 5th byte of scratchpad is configuration register */
	conf = OneWire_ReadByte(OneWire);
	/* Return 9 - 12 value according to number of bits */
	return ((conf & 0x60) >> 5) + 9;
}

uint8_t ds18b20_set_resolution(OneWire_t* OneWire, uint8_t *ROM, DS18B20_Resolution_t resolution) {
	uint8_t th, tl, conf;
	if (!DS18B20_Is(ROM)) {
		return 0;}
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Read scratchpad command by onewire protocol */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);
	/* Ignore first 2 bytes */
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);
	th = OneWire_ReadByte(OneWire);
	tl = OneWire_ReadByte(OneWire);
	conf = OneWire_ReadByte(OneWire);
	if (resolution == DS18B20_Resolution_9bits) {
		conf &= ~(1 << DS18B20_RESOLUTION_R1);
		conf &= ~(1 << DS18B20_RESOLUTION_R0);
	}	else if (resolution == DS18B20_Resolution_10bits) {
		conf &= ~(1 << DS18B20_RESOLUTION_R1);
		conf |= 1 << DS18B20_RESOLUTION_R0;
	}	else if (resolution == DS18B20_Resolution_11bits){
		conf |= 1 << DS18B20_RESOLUTION_R1;
		conf &= ~(1 << DS18B20_RESOLUTION_R0);
	}else if (resolution == DS18B20_Resolution_12bits){
		conf |= 1 << DS18B20_RESOLUTION_R1;
		conf |= 1 << DS18B20_RESOLUTION_R0;
	}
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Write scratchpad command by onewire protocol, only th, tl and conf register can be written */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_WSCRATCHPAD);
	/* Write bytes */
	OneWire_WriteByte(OneWire, th);
	OneWire_WriteByte(OneWire, tl);
	OneWire_WriteByte(OneWire, conf);
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Copy scratchpad to EEPROM of DS18B20 */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_CPYSCRATCHPAD);
	return 1;
}

uint8_t DS18B20_Is(uint8_t *ROM) {
	/* Checks if first byte is equal to DS18B20's family code */
	if (*ROM == DS18B20_FAMILY_CODE) {
		return 1;
    }
	return 0;
}

uint8_t DS18B20_SetAlarmLowTemperature(OneWire_t* OneWire, uint8_t *ROM, int8_t temp) {
	uint8_t tl, th, conf;
	if (!DS18B20_Is(ROM)) {
		return 0;}
	if (temp > 125) {
		temp = 125;}
	if (temp < -55) {
		temp = -55;}
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Read scratchpad command by onewire protocol */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);
	/* Ignore first 2 bytes */
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);
	th = OneWire_ReadByte(OneWire);
	tl = OneWire_ReadByte(OneWire);
	conf = OneWire_ReadByte(OneWire);
	tl = (uint8_t)temp; 
    /* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Write scratchpad command by onewire protocol, only th, tl and conf register can be written */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_WSCRATCHPAD);
	/* Write bytes */
	OneWire_WriteByte(OneWire, th);
	OneWire_WriteByte(OneWire, tl);
	OneWire_WriteByte(OneWire, conf);
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Copy scratchpad to EEPROM of DS18B20 */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_CPYSCRATCHPAD);
	return 1;
}

uint8_t DS18B20_SetAlarmHighTemperature(OneWire_t* OneWire, uint8_t *ROM, int8_t temp) {
	uint8_t tl, th, conf;
	if (!DS18B20_Is(ROM)) {
		return 0;
    }
	if (temp > 125) {
		temp = 125;
    }
	if (temp < -55) {
		temp = -55;
    }
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Read scratchpad command by onewire protocol */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);
	/* Ignore first 2 bytes */
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);
	th = OneWire_ReadByte(OneWire);
	tl = OneWire_ReadByte(OneWire);
	conf = OneWire_ReadByte(OneWire);
	th = (uint8_t)temp; 
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Write scratchpad command by onewire protocol, only th, tl and conf register can be written */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_WSCRATCHPAD);
	
	/* Write bytes */
	OneWire_WriteByte(OneWire, th);
	OneWire_WriteByte(OneWire, tl);
	OneWire_WriteByte(OneWire, conf);
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Copy scratchpad to EEPROM of DS18B20 */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_CPYSCRATCHPAD);
	return 1;
}

uint8_t ds18b20_disable_alarm_temperature(OneWire_t* OneWire, uint8_t *ROM) {
	uint8_t tl, th, conf;
	if (!DS18B20_Is(ROM)) {
		return 0;
    }
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Read scratchpad command by onewire protocol */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_RSCRATCHPAD);
	/* Ignore first 2 bytes */
	OneWire_ReadByte(OneWire);
	OneWire_ReadByte(OneWire);
	th = OneWire_ReadByte(OneWire);
	tl = OneWire_ReadByte(OneWire);
	conf = OneWire_ReadByte(OneWire);
	th = 125;
	tl = (uint8_t)-55;
    /* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Write scratchpad command by onewire protocol, only th, tl and conf register can be written */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_WSCRATCHPAD);
	/* Write bytes */
	OneWire_WriteByte(OneWire, th);
	OneWire_WriteByte(OneWire, tl);
	OneWire_WriteByte(OneWire, conf);
	/* Reset line */
	OneWire_Reset(OneWire);
	/* Select ROM number */
	OneWire_SelectWithPointer(OneWire, ROM);
	/* Copy scratchpad to EEPROM of DS18B20 */
	OneWire_WriteByte(OneWire, ONEWIRE_CMD_CPYSCRATCHPAD);
	return 1;
}

uint8_t DS18B20_AlarmSearch(OneWire_t* OneWire){
	/* Start alarm search */
	return OneWire_Search(OneWire, DS18B20_CMD_ALARMSEARCH);
}

uint8_t ds18b20_calc_done(OneWire_t* OneWire){
	/* If read bit is low, then device is not finished yet with calculation temperature */
	return OneWire_ReadBit(OneWire);
}



#endif //DS18_C
