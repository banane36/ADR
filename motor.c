/*
 * motor.c
 *
 *  Created on: 7 Oct 2021
 *      Author: efomp
 */
#include  "MotorDriverInc/motor.h"


void motor_init(struct hmotor *motor, TIM_HandleTypeDef *htim, uint32_t Channel,
		uint16_t min_us, uint16_t max_us) {
	motor->htim = htim;
	motor->Channel = Channel;
	motor->min_us = min_us;
	motor->max_us = max_us;
	motor->range_us = max_us - min_us;
	__HAL_TIM_SET_AUTORELOAD(motor->htim, 20000);
	__HAL_TIM_SET_PRESCALER(motor->htim, 84);
	__HAL_TIM_SET_CLOCKDIVISION(motor->htim,TIM_CLOCKDIVISION_DIV4);
	HAL_TIM_PWM_Start(motor->htim, motor->Channel);
}

void motor_raw(struct hmotor *motor, uint16_t raw) {
	if (raw > motor->max_us)
		raw = motor->max_us;
	else if (raw < motor->min_us)
		raw = motor->min_us;
	__HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, raw);
}

void motor_percent(struct hmotor *motor, uint8_t percent) {
	if (percent > 100) percent = 100;
	uint16_t new_value = (uint32_t) (percent * motor->range_us) / 100
			+ motor->min_us;
	__HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, new_value);
}

void motor_emStop(struct hmotor *motor) {
	__HAL_TIM_SET_COMPARE(motor->htim, motor->Channel, motor->min_us);
	HAL_Delay(50);
	HAL_TIM_PWM_Stop(motor->htim, motor->Channel);
}
