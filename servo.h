/**
 ******************************************************************************
 * @file           : servo.h
 * @brief          : Header for servo.c file.
 *                   This file contains the common defines of the application.
 ******************************************************************************
 */

#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include "MotorDriverInc/motor.h"

/**
 * @struct hservo
 * @brief Handler structure for a generic servo.
 *
 */
struct hservo {
	struct hmotor motor; /*!< standard PWM-Motors handler					*/
	uint8_t max_deg; /*! Half the degree Range								*/
};


/**
 * @fn void servo_init(struct hservo*, TIM_HandleTypeDef*, uint32_t, uint16_t, uint16_t, uint16_t)
 * @brief Initialises a generic type servo
 *
 * @param servo Pointer to an instance of servo handler
 * @param htim Pointer to an instance of timer handler
 * @param Channel Timer Channel use TIM_CHANNEL_X
 * @param min_us us at which servo is farthest to the left
 * @param max_us us at which servo is farthest to the right
 * @param range_deg Range of degrees the servo supports
 * 					example: -45°-45° -> 90
 */
void servo_init(struct hservo *servo, TIM_HandleTypeDef *htim, uint32_t Channel,
		uint16_t min_us, uint16_t max_us, uint16_t range_deg);

/**
 * @fn void servo_percent(struct hservo*, uint8_t)
 * @brief Sets the angle as function of percentage of range
 *
 * @param servo Pointer to an instance of servo handler
 * @param percent Percentage of range
 * 					example: Quarter Range to left -> 25
 */
void servo_percent(struct hservo *servo, uint8_t percent);


/**
 * @fn void servo_deg(struct hservo*, float)
 * @brief  Set the Servo to a specific angle in degrees
 *
 * @param  servo Pointer to an instance of servo handler
 * @param  deg Floating point angle. 0 is facing straight.
 * 				Values ouot of range are trimmed to range
 */
void servo_deg(struct hservo *servo, float deg);

/**
 * @fn void servo_rad(struct hservo*, float)
 * @brief  Set the Servo to a specific angle in degrees
 *
 * @param  servo Pointer to an instance of servo handler
 * @param  rad Floating point angle. 0 is facing straight.
 * 				Values ouot of range are trimmed to range
 */
void servo_rad(struct hservo *servo, float rad);

/**
 * @fn void servo_emStop(struct hservo*)
 * @brief Stops motor and halts timer. Re init is necessary for relaunch
 *
 * @param servo Pointer to an instance of servo handler
 */
void servo_emStop(struct hservo *servo);

#endif /* INC_SERVO_H_ */
