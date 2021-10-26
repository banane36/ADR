/**
 ******************************************************************************
 * @file           : DS725MG.h
 * @brief          : Header for DS725MG.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 */

#ifndef INC_DS725MG_H_
#define INC_DS725MG_H_

#include "MotorDriverInc/servo.h"

/**
 * @fn void DS725MG_init(struct hservo*, TIM_HandleTypeDef*, uint32_t)
 * @brief Initialises a DS725MG servo
 *
 * @param motor Pointer to an instance of motor handler
 * @param htim Pointer to an  instance of timer handler
 * @param Channel Timer Channel use TIM_CHANNEL_X
 */
void DS725MG_init(struct hservo *servo, TIM_HandleTypeDef *timer_DS725MG,
		uint32_t Channel_DS725MG);

#endif /* INC_DS725MG_H_ */
