/***************************
 * COPYRIGHT (C) VITESCO TECHNOLOGIES
 * ALL RIGHTS RESERVED.
 *
 * The reproduction, transmission or use of this document or its
 * contents is not permitted without express written authority.
 * Offenders will be liable for damages. All rights, including rights
 * created by patent grant or registration of a utility model or design,
 * are reserved.
 ***************************/
#include "BSW/HAL/Temp_Sensor/temp_sensor.h"
#include "BSW/HAL/Com/com.h"


extern COM_GET_struct g_GET_DataStructure;

static const char *TAG = "HAL TEMP SENSOR";

static uint8_t u8Counter = 0;

/***************************
 *  Function name    : DHT11_vRequest
 *
 *  Description      : Send pulse to activate the sensor
 *
 *  List of arguments: -
 *
 *  Return value     : -
 *
 ***************************/
void DHT11_vRequest(void)
{
	/* Set the direction to output for a new communication */
	GPIO_vSetDirection(DHT11_PIN, DIR_OUTPUT);

	/* H-L-H of at least 18ms */
	GPIO_vSetLevel(DHT11_PIN, HIGH_LEVEL);
	GPIO_vSetLevel(DHT11_PIN, LOW_LEVEL);
	ets_delay_us(20000);
	/* Set as input so 10k pull-up from circuit drives pin High */
	GPIO_vSetDirection(DHT11_PIN, DIR_INPUT);
}

/***************************
 *  Function name    : DHT11_i8Response
 *
 *  Description      : See if the sensor got the pulse
 *
 *  List of arguments: -
 *
 *  Return value     : int8_t -> 0 - OK; -1 for TIMEOUT
 *
 ***************************/
int8_t DHT11_i8Response(void)
{
	u8Counter = 0;
	while (GPIO_iGetLevel(DHT11_PIN))
	{
		if (u8Counter >= DHT11_RESPONSE_START_US)
		{
			return DHT11_TIMEOUT;
		}
		u8Counter++;
		ets_delay_us(1);
	}

	u8Counter = 0;
	while (GPIO_iGetLevel(DHT11_PIN) == 0)
	{
		if (u8Counter >= DHT11_RESPONSE_LOW_US)
		{
			return DHT11_TIMEOUT;
		}
		u8Counter++;
		ets_delay_us(1);
	}

	u8Counter = 0;
	while (GPIO_iGetLevel(DHT11_PIN))
	{
		if (u8Counter >= DHT11_RESPONSE_HIGH_US)
		{
			return DHT11_TIMEOUT;
		}
		u8Counter++;
		ets_delay_us(1);
	}

	return DHT11_OK;
}

/***************************
 *  Function name    : DHT11_i8Receive
 *
 *  Description      : Get 8 bits of data from the sensor
 *
 *  List of arguments: -
 *
 *  Return value     : int8_t -> Data received; -1 for TIMEOUT
 *
 ***************************/
int8_t DHT11_i8Receive(void)
{
	uint8_t u8Index = 0;
	/* Sensor measures data in the range of 0-50 degrees and 0-100(max) humidity
	 * so it is safe to type cast uint to int or int to uint */
	uint8_t u8Data = 0;

	for (; u8Index < 8; u8Index++)
	{
		/* Get the common 0 level for both 0 and 1 bits */
		u8Counter = 0;
		while (GPIO_iGetLevel(DHT11_PIN) == 0)
		{
			if (u8Counter >= DHT11_BIT_START_US)
			{
				return DHT11_TIMEOUT;
			}
			u8Counter++;
			ets_delay_us(1);
		}

		/* Measure the bit value */
		u8Counter = 0;
		while (GPIO_iGetLevel(DHT11_PIN))
		{
			/* Max time of bit */
			if (u8Counter >= DHT11_BIT_1_US)
			{
				{
					return DHT11_TIMEOUT;
				}
			}
			u8Counter++;
			ets_delay_us(1);
		}

		/* Time is greater than a 0 so it must be a one */
		if (u8Counter > DHT11_BIT_0_US)
		{
			u8Data = (u8Data << 1) | (0x01);
		}
		else
		{
			u8Data = (u8Data << 1);
		}
	}

	return (int8_t) u8Data;
}

DHT11_struct DHT11_dht11Read()
{
    DHT11_struct DHT11_result;
    int8_t result[6];

    for(int i=0; i < 5; i++)
    {
        result[i] = 0;
    }

    DHT11_vRequest();

    if(DHT11_i8Response() != OK )
    {
        ESP_LOGI(TAG, "Request timeout");
    }

    for(int i=0; i<5; i++)
    {
        result[i] = DHT11_i8Receive();
        if(result[i] == DHT11_TIMEOUT)
            ESP_LOGI(TAG, "Receive timeout");
    }

    DHT11_result.u8IntegralHum = result[0];
    DHT11_result.u8DecimalHum = result[1];
    
    DHT11_result.u8IntegralTemp = result[2];    
    DHT11_result.u8DecimalTemp = result[3];

    DHT11_result.u8CheckSum = result[4];

    if(result[0] + result[1] + result[2]  + result[3] != result[4])
    {
        ESP_LOGI(TAG, "Checksum timeout");
        DHT11_result.u8CheckSum = 0;
    }

    return DHT11_result;
}

void DHT11_vTaskTempAndHumCalculate()
{
	DHT11_struct de_afis;

	de_afis = DHT11_dht11Read();

	printf("temp: %d  \n  humidity: %d", de_afis.u8IntegralTemp, de_afis.u8IntegralHum);
}