/*
 * roxyBl715s.c
 *
 *  Created on: Oct 5, 2021
 *      Author: efomp
 */
#include "MotorDriverInc/ROXXYBL715s.h"


void ROXXYBL715S_init(struct hmotor *motor, TIM_HandleTypeDef *htim, uint32_t Channel)
{
	motor_init(motor, htim, Channel, 1100, 1500);
	motor_percent(motor, 100);
	HAL_Delay(100);
	motor_percent(motor, 0);
	HAL_Delay(5000);
}


