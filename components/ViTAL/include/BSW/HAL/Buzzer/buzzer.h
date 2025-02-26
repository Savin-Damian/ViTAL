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

#include "BSW/MCAL/PWM/pwm.h"



#define BUZZER_0 0
#define BUZZER_25 2500
#define BUZZER_50 5000
#define BUZZER_75 7500



void BUZZER_vChangeDutyCycle(uint32_t u32BuzzerDutyCycle);