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

#ifndef COMPONENTS_VITAL_BSW_HAL_SHIFT_REGISTER_H
#define COMPONENTS_VITAL_BSW_HAL_SHIFT_REGISTER_H

#include "global.h"



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
void SHIFTREG_vOutput8Bits(uint8_t u8Data);


#endif
