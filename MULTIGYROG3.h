/**
 ******************************************************************************
 * @file           : MULTIGYROG3.h
 * @brief          : Header for MULTIGYROG3.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 */

#ifndef INC_MULTIGYROG3_H_
#define INC_MULTIGYROG3_H_

#include "main.h"

/**
 * @enum MULTIGYROG3_STATE
 * @brief States of a multigyro-board
 *
 */
typedef enum {
	MULTIGYROG3_NORMAL,
	MULTIGYROG3_NO_GYRO,
	MULTIGYROG3_HEADING,
	MULTIGYROG3_ERROR,
} MULTIGYROG3_STATE;

/**
 * @struct MULTIGYROG3
 * @brief PWM handler for a Multigyro G3 Board
 *
 */
struct hMULTIGYROG3 {
	TIM_HandleTypeDef *htim; /*! < timer handler								*/
	uint32_t Channel; /*! < timer channel										*/
	MULTIGYROG3_STATE State; /*!< State of Multigyro							*/
};

/**
 * @fn void MULTIGYROG3_init(struct MULTIGYROG3*, TIM_HandleTypeDef*, uint32_t)
 * @brief Initialises a Mulitygyro G3 in normal mode
 *
 * @param pwm Pointer to an instance of PWM handler
 * @param htim Pointer to an instance of timer handler
 * @param Channel Timer Channel use TIM_CHANNEL_X
 */
void MULTIGYROG3_init(struct hMULTIGYROG3 *pwm, TIM_HandleTypeDef *htim,
		uint32_t Channel);

/**
 * @fn void MULTIGYROG3_calibrate(struct hMULTIGYROG3*)
 * @brief Calibrates the gyro for heading mode
 *
 * @param pwm
 */
void MULTIGYROG3_calibrate(struct hMULTIGYROG3 *pwm);

/**
 * @fn void MULTIGYROG3_setMode(struct hMULTIGYROG3*, MULTIGYROG3_STATE)
 * @brief set the control mode of the Multigyro
 *
 * @param pwm Pointer to an instance of PWM handler
 * @param state Multigyro state from MULTIGYROG3_STATE enum
 */
void MULTIGYROG3_setMode(struct hMULTIGYROG3 *pwm, MULTIGYROG3_STATE state);

/**
 * @fn MULTIGYROG3_STATE MULTIGYROG3_setMode(struct hMULTIGYROG3*)
 * @brief get the control mode of the Multigyro
 *
 * @param pwm Pointer to an instance of PWM handler
 * @return actual state of the Multigyro-handler
 */
MULTIGYROG3_STATE MULTIGYROG3_getMode(struct hMULTIGYROG3 *pwm);

#endif /* INC_MULTIGYROG3_H_ */
