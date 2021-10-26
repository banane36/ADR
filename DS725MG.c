/*
 * servo.c
 *
 *  Created on: 05.10.2021
 *      Author: benjamin
 */
#include "MotorDriverInc/DS725MG.h"

void DS725MG_init(struct hservo *servo, TIM_HandleTypeDef *timer_DS725MG,
		uint32_t Channel_DS725MG)
{
	servo_init(servo, timer_DS725MG, Channel_DS725MG, 900, 2100, 120);
}

