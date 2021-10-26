/**
 ******************************************************************************
 * @file           : ROXXYBL715s.h
 * @brief          : Header for ROXXYBL715s.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 */


/*
 * roxyBL715s.h
 *
 *  Created on: Oct 5, 2021
 *      Author: efomp
 */

#ifndef INC_ROXXYBL715S_H_
#define INC_ROXXYBL715S_H_

#include "MotorDriverInc/motor.h"

/**
 * @fn void ROXXYBL715S_init(struct hmotor*, TIM_HandleTypeDef*, uint32_t)
 * @brief Initialises a ROXXYBL715S motor
 *
 * @param motor Pointer to an instance of motor handler
 * @param htim Pointer to an  instance of timer handler
 * @param Channel Timer Channel use TIM_CHANNEL_X
 */
void ROXXYBL715S_init(struct hmotor *motor, TIM_HandleTypeDef *htim, uint32_t Channel);


#endif /* INC_ROXXYBL715S_H_ */
