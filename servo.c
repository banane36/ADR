/*
 * servo.c
 *
 *  Created on: Oct 8, 2021
 *      Author: efomp
 */

#include "MotorDriverInc/servo.h"

void servo_init(struct hservo *servo, TIM_HandleTypeDef *htim, uint32_t Channel,
		uint16_t min_us, uint16_t max_us, uint16_t range_deg) {
	__HAL_TIM_SET_COMPARE(htim, Channel, (max_us - min_us) / 2 + min_us);
	motor_init(&servo->motor, htim, Channel, min_us, max_us);
	servo->max_deg = range_deg / 2;
}

void servo_percent(struct hservo *servo, uint8_t percent) {
	motor_percent(&servo->motor, percent);
}

void servo_deg(struct hservo *servo, float deg) {

	//shift the values in the positive range
	float absolute_deg = deg + servo->max_deg;

	//check eventual violations of the range
	if (absolute_deg < 0)
		absolute_deg = 0;
	else if (absolute_deg > 2 * servo->max_deg)
		absolute_deg = 2 * servo->max_deg;

	//change degree values to a numbre of ticks
	uint16_t new_value = (uint16_t) ((absolute_deg * servo->motor.range_us)
			/ (2 * servo->max_deg) + servo->motor.min_us);

	//set the new value
	motor_raw(&servo->motor, new_value);
}

void servo_rad(struct hservo *servo, float rad) {

	//shift the value in the positive range
	float deg = (rad + 0.7853) * 180 / 3.14159;
	servo_deg(servo, deg);
}

void servo_emStop(struct hservo *servo) {
	motor_percent(&servo->motor, 50);
	HAL_Delay(50);
	HAL_TIM_PWM_Stop(servo->motor.htim, servo->motor.Channel);
}
