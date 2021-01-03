/**
 * @file ds18.h
 * @author Shoma Gane <shomagan@gmail.com>
 *         Ayrat Girfanov <girfanov.ayrat@yandex.ru>
 * @defgroup inc
 * @ingroup inc
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
#ifndef DS18_H
#define DS18_H 1
 
/*add includes below */
#include "main.h"
#include "ds18_config.h"
#include "onewire.h"
#include <stdbool.h>

/*add includes before */
#ifdef __cplusplus 
   extern "C" {
#endif
/*add functions and variable declarations below */
void ds18_task( const void *parameters);
//###################################################################################
typedef struct{
	uint8_t 	Address[8];
	float 		Temperature;
	bool			data_validate;	
}Ds18b20Sensor_t;
//###################################################################################

extern Ds18b20Sensor_t	ds18b20[_DS18B20_MAX_SENSORS];
extern uint8_t 	temp_sensor_count;
extern uint8_t data_valid;
//###################################################################################
/* Every onewire chip has different ROM code, but all the same chips has same family code */
/* in case of DS18B20 this is 0x28 and this is first byte of ROM address */
#define DS18B20_FAMILY_CODE						0x28
#define DS18B20_CMD_ALARMSEARCH				0xEC

/* DS18B20 read temperature command */
#define DS18B20_CMD_CONVERTTEMP				0x44 	/* Convert temperature */
#define DS18B20_DECIMAL_STEPS_12BIT		0.0625
#define DS18B20_DECIMAL_STEPS_11BIT		0.125
#define DS18B20_DECIMAL_STEPS_10BIT		0.25
#define DS18B20_DECIMAL_STEPS_9BIT		0.5

/* Bits locations for resolution */
#define DS18B20_RESOLUTION_R1					6
#define DS18B20_RESOLUTION_R0					5

/* CRC enabled */
#ifdef DS18B20_USE_CRC	
#define DS18B20_DATA_LEN							9
#else
#define DS18B20_DATA_LEN							2
#endif

//###################################################################################
typedef enum {
	DS18B20_Resolution_9bits = 9,   /*!< DS18B20 9 bits resolution */
	DS18B20_Resolution_10bits = 10, /*!< DS18B20 10 bits resolution */
	DS18B20_Resolution_11bits = 11, /*!< DS18B20 11 bits resolution */
	DS18B20_Resolution_12bits = 12  /*!< DS18B20 12 bits resolution */
} DS18B20_Resolution_t;

//###################################################################################
bool			Ds18b20_ManualConvert(void);
//###################################################################################
uint8_t 	DS18B20_Start(OneWire_t* OneWireStruct, uint8_t* ROM);
void 			ds18b20_start_calc(OneWire_t* OneWireStruct);
bool		 	ds18b20_read(OneWire_t* OneWireStruct, uint8_t* ROM, float* destination);
uint8_t 	DS18B20_GetResolution(OneWire_t* OneWireStruct, uint8_t* ROM);
uint8_t 	ds18b20_set_resolution(OneWire_t* OneWireStruct, uint8_t* ROM, DS18B20_Resolution_t resolution);
uint8_t 	DS18B20_Is(uint8_t* ROM);
uint8_t 	DS18B20_SetAlarmHighTemperature(OneWire_t* OneWireStruct, uint8_t* ROM, int8_t temp);
uint8_t 	DS18B20_SetAlarmLowTemperature(OneWire_t* OneWireStruct, uint8_t* ROM, int8_t temp);
uint8_t 	ds18b20_disable_alarm_temperature(OneWire_t* OneWireStruct, uint8_t* ROM);
uint8_t 	DS18B20_AlarmSearch(OneWire_t* OneWireStruct);
uint8_t 	ds18b20_calc_done(OneWire_t* OneWireStruct);
//###################################################################################

/*add functions and variable declarations before */
#ifdef __cplusplus
}
#endif
#endif //DS18_H
