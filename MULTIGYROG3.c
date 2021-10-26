/*
 * MULTIGYROG3.c
 *
 *  Created on: Oct 8, 2021
 *      Author: efomp
 */

#include "MotorDriverInc/MULTIGYROG3.h"

void MULTIGYROG3_init(struct hMULTIGYROG3 *pwm, TIM_HandleTypeDef *htim,
		uint32_t Channel) {
	pwm->htim = htim;
	pwm->Channel = Channel;

	__HAL_TIM_SET_AUTORELOAD(pwm->htim, 20000);
	__HAL_TIM_SET_PRESCALER(pwm->htim, 84);
	__HAL_TIM_SET_CLOCKDIVISION(pwm->htim, TIM_CLOCKDIVISION_DIV4);
	MULTIGYROG3_setMode(pwm, MULTIGYROG3_NO_GYRO);
	HAL_TIM_PWM_Start(pwm->htim, pwm->Channel);

}


void MULTIGYROG3_Calibrate(struct hMULTIGYROG3 *pwm) {
	__HAL_TIM_SET_COMPARE(pwm->htim, pwm->Channel, 1100);
	HAL_Delay(30);
	__HAL_TIM_SET_COMPARE(pwm->htim, pwm->Channel, 1900);
	HAL_Delay(30);
	__HAL_TIM_SET_COMPARE(pwm->htim, pwm->Channel, 1100);
	HAL_Delay(30);
	__HAL_TIM_SET_COMPARE(pwm->htim, pwm->Channel, 1900);
	HAL_Delay(30);
	MULTIGYROG3_setMode(pwm, pwm->State);
}

void MULTIGYROG3_setMode(struct hMULTIGYROG3 *pwm, MULTIGYROG3_STATE state) {
	switch (state) {
	case MULTIGYROG3_NORMAL:
		__HAL_TIM_SET_COMPARE(pwm->htim, pwm->Channel, 1100);
		break;
	case MULTIGYROG3_NO_GYRO:
		__HAL_TIM_SET_COMPARE(pwm->htim, pwm->Channel, 1500);
		break;
	case MULTIGYROG3_HEADING:
		__HAL_TIM_SET_COMPARE(pwm->htim, pwm->Channel, 1900);
		break;
	default:
		pwm->State = MULTIGYROG3_ERROR;
		return;
	}
	pwm->State = state;
}

MULTIGYROG3_STATE MULTIGYROG3_getMode(struct hMULTIGYROG3 *pwm) {
	return pwm->State;
}
