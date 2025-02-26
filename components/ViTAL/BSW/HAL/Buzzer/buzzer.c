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
#include "BSW/HAL/Buzzer/buzzer.h"
#include "BSW/MCAL/PWM/pwm.h"





void BUZZER_vChangeDutyCycle(uint32_t u32BuzzerDutyCycle)
{
    
    if(u32BuzzerDutyCycle < 2500)
    {
        PWM_vSetDutyCycle(BUZZER_PWM_CHANNEL, BUZZER_0 );
    }
    if(u32BuzzerDutyCycle > 2500 &&  u32BuzzerDutyCycle < 5000)
    {
        PWM_vSetDutyCycle(BUZZER_PWM_CHANNEL, BUZZER_25 );
    }
    if(u32BuzzerDutyCycle > 5000 &&  u32BuzzerDutyCycle < 7500)
    {
        PWM_vSetDutyCycle(BUZZER_PWM_CHANNEL, BUZZER_50 );
    }
    if(u32BuzzerDutyCycle >7500)
    {
        PWM_vSetDutyCycle(BUZZER_PWM_CHANNEL, BUZZER_75 );
    }
    
   PWM_vSetDutyCycle(BUZZER_PWM_CHANNEL, u32BuzzerDutyCycle );

}