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

#include "BSW/HAL/Shift_Register/shift_register.h"

#include "BSW/MCAL/GPIO/gpio.h"

static const char *TAG = "HAL SHIFT REGISTER";

/*******************************************************************************
 *  Function name    : SHIFTREG_vOutput8Bits
 *
 *  Description      : Output an 8 bit data value for each led-n you have value 2^n to turn it on 
 *                     you can also write in binary for example (0b00001110) will turn on only the first three leds
 *
 *  List of arguments: u8Data -> Data to be output MSb to LSb
 *
 *  Return value     : -
 *
 *******************************************************************************/
void SHIFTREG_vOutput8Bits(uint8_t u8Data)
{
    /* Set low all pins that are connected to the shift register */
	GPIO_vSetLevel(SN74HC595N_DS_PIN, LOW_LEVEL);
	GPIO_vSetLevel(SN74HC595N_SH_CP_PIN, LOW_LEVEL);
	GPIO_vSetLevel(SN74HC595N_ST_CP_PIN, LOW_LEVEL);

    /* Store data */
	int8_t i8Index = 7;
	/* Store data --- The 74HC595 has two 8-bit registers (which can be thought of as “memory containers”). 
	The first is referred to as the Shift Register, and the second as the Storage/Latch Register.*/
	for (; i8Index >= 0; i8Index--)
	{
		/* Store data --- The bits contained in u8Data are extracted one by one, starting with bit 7 and ending with bit 0, to be sent to the shift register*/
		GPIO_vSetLevel(SN74HC595N_DS_PIN, (u8Data & (1 << i8Index)) >> i8Index);

		/* Send storage clock */
		GPIO_vSetLevel(SN74HC595N_SH_CP_PIN, HIGH_LEVEL);
		GPIO_vSetLevel(SN74HC595N_SH_CP_PIN, LOW_LEVEL);
		vTaskDelay(1);
	}

	/* Send latch clock --- the contents of the shift register are copied to the storage/latch register.
	As a result, whenever the value in the storage register changes, the output changes. */
	GPIO_vSetLevel(SN74HC595N_ST_CP_PIN, HIGH_LEVEL);
	GPIO_vSetLevel(SN74HC595N_ST_CP_PIN, LOW_LEVEL);

}