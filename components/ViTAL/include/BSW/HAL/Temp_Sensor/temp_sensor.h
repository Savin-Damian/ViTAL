/*******************************************************************************
 * COPYRIGHT (C) VITESCO TECHNOLOGIES
 * ALL RIGHTS RESERVED.
 *
 * The reproduction, transmission or use of this document or its
 * contents is not permitted without express written authority.
 * Offenders will be liable for damages. All rights, including rights
 * created by patent grant or registration of a utility model or design,
 * are reserved.
 *******************************************************************************/

#include "BSW/MCAL/GPIO/gpio.h"
#include "global.h"

typedef struct
{
uint8_t u8IntegralHum;
uint8_t u8DecimalHum;
uint8_t u8IntegralTemp;
uint8_t u8DecimalTemp;
uint8_t u8CheckSum;
} DHT11_struct;
/* Errors */
#define DHT11_TIMEOUT -1
#define DHT11_OK 0
/* Time limits from datasheet + aprox 5 us */
#define DHT11_RESPONSE_START_US 45
#define DHT11_RESPONSE_LOW_US 85
#define DHT11_RESPONSE_HIGH_US 85
#define DHT11_BIT_START_US 55
#define DHT11_BIT_0_US 28
#define DHT11_BIT_1_US 75


void DHT11_vRequest();
int8_t DHT11_vResponse();
int8_t DHT11_i8Receive();
DHT11_struct DHT11_dht11Read();
void DHT11_vTaskTempAndHumCalculate();