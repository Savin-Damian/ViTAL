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

#include "ASW/Ambient_Light/ambient_light.h"
#include "BSW/HAL/Shift_Register/shift_register.h"
#include "RTE/rte.h"

static const char *TAG = "ASW AL";

void ASW_vTaskShiftRegControlTest(void)
{
	//if the button is pressed
	if (RTE_bGet_ButtonShiftRegStatus())
	{
		int8_t i8Index = 1;
		uint8_t lastdata=0;
		for (; i8Index <= 8; i8Index++)
		{
			lastdata = lastdata + (1 << i8Index);
			SHIFTREG_vOutput8Bits(lastdata); 
			vTaskDelay(100);
		}
	}
	else if(!RTE_bGet_ButtonRLedStatus() && !RTE_bGet_ButtonGLedStatus() && !RTE_bGet_ButtonBLedStatus() && !RTE_bGet_ButtonAmbientalLightsStatus())
	{
		RTE_vSetShiftRegisterOutput(ALL_COLORS, LOW);
	}
}

void ASW_vTaskRGBLedControlTest(void)
{
	//if one of the buttons is pressed
	if(!RTE_bGet_ButtonAmbientalLightsStatus() && !RTE_bGet_ButtonShiftRegStatus())
	{
		RTE_vSetRGBLedState();
	}
}