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

#ifndef COMPONENTS_VITAL_SRVL_SCHEDULER_H
#define COMPONENTS_VITAL_SRVL_SCHEDULER_H

#include "global.h"
#include "BSW/HAL/Temp_Sensor/temp_sensor.h"
#include "BSW/MCAL/GPIO/gpio.h"


void SYSTEM_vInit(void);

void Task_Req8(void);

void Task_Req4(void);

bool Soft_Req20();

void Task_Req5();

void vTask100ms(void);

void vTask200ms(void);

void vTask1000(void);

void vTask2000ms(void);
void Task_Req8(void);

void SYSTEM_vTaskScheduler(void);

#define TASK_LOOP_FREQ 100

#define TASK_100MS (100 / TASK_LOOP_FREQ)
#define TASK_200MS (200 / TASK_LOOP_FREQ)
#define TASK_500MS (500 / TASK_LOOP_FREQ)
#define TASK_800MS (800 / TASK_LOOP_FREQ)
#define TASK_1000MS (1000 / TASK_LOOP_FREQ)
#define TASK_2000MS (2000 / TASK_LOOP_FREQ)
#define TASK_5000MS (5000 / TASK_LOOP_FREQ)

#endif
