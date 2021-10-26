/**
 ******************************************************************************
 * @file           : motor.h
 * @brief          : Header for motor.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include "main.h"

/**
 * @struct hmotor
 * @brief  Handler structure for a generic servo.
 *
 */
struct hmotor {
	TIM_HandleTypeDef *htim; /*! < timer handler								*/
	uint32_t Channel; /*! < timer channel										*/
	uint16_t min_us; /*! <  us for zero throttle								*/
	uint16_t max_us; /*! < 	us for full throttle								*/
	uint16_t range_us; /*! < max_us-min_us										*/
};

/**
 * @fn void motor_init(struct hmotor*, TIM_HandleTypeDef*, uint32_t, uint16_t, uint16_t)
 * @brief Initialises a generic type PWM-Motor
 *
 * @param motor Pointer to an instance of motor handler
 * @param htim Pointer to an instance of timer handler
 * @param Channel Timer Channel use TIM_CHANNEL_X
 * @param min_us us dor zero throttle
 * @param max_us us for full throttle
 */
void motor_init(struct hmotor *motor, TIM_HandleTypeDef *htim, uint32_t Channel,
		uint16_t min_us, uint16_t max_us);

/**
 * @fn void motor_percent(struct hmotor*, uint8_t)
 * @brief Sets the motor speed as percentage
 *
 * @param motor Pointer to an instance of motor handler
 * @param percent Percentage of range
 * 					example: half throttle  -> 50
 */
void motor_percent(struct hmotor *motor, uint8_t percent);

/**
 * @fn void motor_raw(struct hmotor*, uint16_t)
 * @brief Sets the pulse width in us
 *
 * @param motor Pointer to an instance of motor handler
 * @param raw pulse width value in us. Out of Range Values get trimmed to range
 */
void motor_raw(struct hmotor *motor, uint16_t raw);

/**
 * @fn void motor_stop(struct hmotor*)
 * @brief Stops motor and halts timer. Re init is necessary for relaunch
 *
 * @param motor Pointer to an instance of motor handler
 */
void motor_emStop(struct hmotor *motor);


#endif /* INC_MOTOR_H_ */
