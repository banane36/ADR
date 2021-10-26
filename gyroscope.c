/*
 * gyroscope.c
 *
 *	This file contains functions for the gyroscope LSM6DS33
 *  Created on: Oct 11, 2021
 *      Author: Anna Gerig
 *      Version: 1.0
 *      Changes: none
 */

/*------------------------------------------------------------------------
  Includes
/-----------------------------------------------------------------------*/
#include "main.h"
#include "gyroscope.h"
#include <stdio.h>
#include <string.h>

/*------------------------------------------------------------------------
  Variables
/-----------------------------------------------------------------------*/
HAL_StatusTypeDef result; 						// Variable for the HAL status
I2C_HandleTypeDef hi2c1;						// Handler for the I2C communication
uint8_t buf[6];									// Buffer Array for I2C
int maxDps = 500;								// Gyroscope full-scale selection: 125 | 250 | 500 | 1000 | 2000 dps
uint16_t regMaxDps = 0b10000100;			    // Control Register 2: select frequency and dps

/*------------------------------------------------------------------------
  Information
/-----------------------------------------------------------------------*/
     /* CTRL2_G Register to select dps
      * The last 4 bit change the full-scale selection in dps (see below).
      * They can be entered in the "regMaxDps" above.
      *
      * 1.66kHz    = 1000 0000
	  * 250  dps   = 1000 0000	-> 0x80
	  * 500  dps   = 1000 0100	-> 0x84
	  * 1000 dps   = 1000 1000	-> 0x44
	  * 2000 dps   = 1000 1100	-> 0x8C
	  * 125  dps   = 1000 0010	-> 0x82
	  */

/*------------------------------------------------------------------------
  Functions
/-----------------------------------------------------------------------*/

/* @brief  Gyroscope Initialization Function
 * 		   Activation of gyroscope with frequency and degrees per second
 * 		   (maxDps and regMaxDps defined above)
 * @param  None
 * @retval None
 */
void Gyro_init() {

	// activate gyroscope
	Gyro_write(GYRO_ADDR, CTRL2_G, regMaxDps);	// writes to CTRL2_G register
}

/* @brief  Gyroscope Angle Calculation Function
 * 		   This function reads 6 register (two register for each axis) and
 * 		   combines them to three values for X-, Y- and Z-axis. The dps-values
 * 		   are converted to degrees
 * @param  Reference-by-value of x-, y- and z-axis
 * @retval None
 */
void getAngle(float *x, float *y, float *z){

	uint8_t block[6];							// variable for register values
	Gyro_read(GYRO_ADDR, OUTX_L_G, block, 6);	// reading 6 register, two for each axis

	// combine register for each axis
	int16_t xAxis = ((int16_t)block[0] | (block[1] << 8));
	int16_t yAxis = ((int16_t)block[2] | (block[3] << 8));
	int16_t zAxis = ((int16_t)block[4] | (block[5] << 8));

	// convert dps to degrees
	*x = (float)(xAxis) / 0x7FFF * maxDps;
	*y = (float)(yAxis) / 0x7FFF * maxDps;
	*z = (float)(zAxis) / 0x7FFF * maxDps;
}

/* @brief  Gyroscope Write Function
 * 		   This function writes 1byte of data to the gyroscope over I2C
 * @param  Device address, Register to write in, data of the byte
 * @retval None
 */
void Gyro_write(uint8_t address, uint8_t reg, uint8_t data){

	buf[0] = reg;								// buffer for register to write in
	buf[1] = data;								// buffer for data to write
	result = HAL_I2C_Master_Transmit(&hi2c1, address, buf, 2, HAL_MAX_DELAY);
	// error handling if I2C communication doesn't work
	if ( result != HAL_OK )
	{
		printf("I2C ERROR!\r\n");
	}
	else
	{
		printf("Register write done\r\n");
	}
}

/* @brief  Gyroscope Read Function
 * 		   This function reads data of the gyroscope over I2C
 * @param  Device address, Register to read, Array to save read data, Number of bytes to read
 * @retval None
 */
void Gyro_read(uint8_t address, uint8_t reg, uint8_t *data, uint8_t bytes){

	data[0] = reg;								// data-array to save read register
	result = HAL_I2C_Master_Transmit(&hi2c1, address, data, 1, HAL_MAX_DELAY);
	result = HAL_I2C_Master_Receive(&hi2c1, address, data, bytes, HAL_MAX_DELAY);
}
