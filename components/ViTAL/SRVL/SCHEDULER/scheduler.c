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

#include "SRVL/SCHEDULER/scheduler.h"

#include "BSW/MCAL/ADC/adc.h"
#include "BSW/MCAL/GPIO/gpio.h"
#include "BSW/HAL/Buzzer/buzzer.h"
#include "BSW/MCAL/PWM/pwm.h"
#include "BSW/MCAL/WIFI/wifi.h"
#include "BSW/HAL/Servo_Motor/servo_motor.h"
#include "BSW/HAL/Proximity_Sensor/proximity_sensor.h"

#include "RTE/rte.h"

#include "BSW/HAL/Com/com.h"
#include "BSW/HAL/Shift_Register/shift_register.h"
#include "ASW/Ambient_Light/ambient_light.h"

#include "nvs_flash.h"
#include "BSW/HAL/Photo_Resistor/photo_resistor.h"

extern COM_GET_struct g_GET_DataStructure;
extern COM_POST_struct g_POST_DataStructure;
static const char *TAG = "SRVL SCHEDULER";
static httpd_handle_t server = NULL;

double c1 = -8.78469475556;
double  c2 = 1.61139411;
double c3 = 2.33854883889;
double c4 = -0.14611605;
double c5 = -0.012308094;
double c6 = -0.0164248277778;
double c7 = 0.002211732;
double c8 = 0.00072546;
double c9 = -0.000003582;





void SYSTEM_vInit(void)
{
    /* Call these functions only when specific HW parts are connected */
    
	//Initialize NVS
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	/* Initialize All periferials */
	ADC_vInit();

	GPIO_vInit();

	PWM_vInit();
	
	WIFI_vInit(&server);
}

void Task_Req8(void)
{
	PHRES_vTaskCalculate();
	if(g_GET_DataStructure.u16PhotoRes < 7000)
	{
		RTE_vSetShiftRegisterOutput(HEAD_LIGHTS_POS, HIGH);
		//SHIFTREG_vOutput8Bits(0b00001110);
	}
	else
	{
		RTE_vSetShiftRegisterOutput(HEAD_LIGHTS_POS, LOW);
	}
	if(g_POST_DataStructure.bButtonDoorLock == false)
	{
		RTE_vSetAmbientalLightsState(true);
	}
}

void Task_Req4()
{
	uint16_t rezultat = PROX_u16Read();
	if ( rezultat <= 50)
	{
		g_GET_DataStructure.bIsOccupied =true;
	}
	else{
		g_GET_DataStructure.bIsOccupied =false;
	}
}


bool Soft_Req20()
{
	if(g_POST_DataStructure.bButtonSecurity &&  g_GET_DataStructure.bIsLocked && g_POST_DataStructure.bButtonDoorLock)
		return true;
	return false;	
}

void Task_Req5()
{
	if(Soft_Req20())
	{ 
		BUZZER_vChangeDutyCycle(90);
		RTE_vSetShiftRegisterOutput(HEAD_LIGHTS_POS , HIGH);         
		vTaskDelay(20);
		BUZZER_vChangeDutyCycle(0);
		RTE_vSetShiftRegisterOutput(HEAD_LIGHTS_POS, LOW);
	}
}

void Task_Req2()
{
	DHT11_struct de_afis;

	de_afis = DHT11_dht11Read();

	uint8_t T=de_afis.u8IntegralTemp;
	uint8_t R=de_afis.u8IntegralHum;

	uint8_t HI = c1 + c2*T + c3 * R + c4*T*R + c5 * T*T + c6 * R*R + c7*T*T*R + c8* T*R*R +c9 *T*T*R*R;
}


void vTask100ms(void)
{	
	/* Send and receive data to the HTML interface -- get and post functions*/
	COM_vTaskProcessServer();
	/* Call Shift register functionalty */
	ASW_vTaskShiftRegControlTest();
}

void vTask200ms(void)
{
	/* Call RGB led functioality */
	ASW_vTaskRGBLedControlTest();
	PROX_u16Read();
	Task_Req4();
	Task_Req5();	
}

void vTask500ms(void)
{
	//BUZZER_vChangeDutyCycle(3000);
	SERVO_vChangeAngle (550);
	DHT11_vTaskTempAndHumCalculate();

}

void vTask800ms(void)
{
	PHRES_vTaskCalculate();
} 


void vTask1000ms(void)
{
	PHRES_vTaskCalculate();
	Task_Req8();
}

void vTask2000ms(void)
{
	//Task_Req4();
}


void SYSTEM_vTaskScheduler(void)
{
	uint16_t u16TickCount = 0;

	while (1)
	{

		if (u16TickCount % TASK_100MS == 0)
		{
			vTask100ms();
		}

		if (u16TickCount % TASK_200MS == 0)
		{
			vTask200ms();	
			
		}

		if (u16TickCount % TASK_500MS == 0)
		{
			vTask500ms();	
		}
		
		if (u16TickCount % TASK_800MS == 0)
		{
			vTask800ms();	
		}

		if(u16TickCount % TASK_1000MS == 0)
		{
			vTask1000ms();
		}
		if(u16TickCount % TASK_2000MS == 0)
		{
			vTask2000ms();
		}
	
		
		u16TickCount++;
		if (u16TickCount >= TASK_2000MS)
		{
			u16TickCount = 0;
		}
			
		vTaskDelay(100); // 1 sec 
	
	}
}
