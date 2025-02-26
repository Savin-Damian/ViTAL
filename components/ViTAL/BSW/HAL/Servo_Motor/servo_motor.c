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
#include "BSW/HAL/Servo_Motor/servo_motor.h"


void SERVO_vChangeAngle (uint32_t u32ServoAngle)
{
    PWM_vSetDutyCycle(SERVO_MOTOR_PWM_CHANNEL, u32ServoAngle );
}