/*
 * calibrate.h
 *	Header file for calibration of deflection sensor.
 *	Written specifically for use with STM32 cubeIDE HAL on MK3 deflection sensor board
 *  Created on: Nov 02, 2023
 *  Author: Gustav Friis Fléron
 */

#include "MK3_configuration.h"

#ifndef INC_CALIBRATE_H_
#define INC_CALIBRATE_H_

void calibrate_deflection(I2C_HandleTypeDef *i2c_handle);
void save_data(uint32_t Address, uint32_t data);
uint32_t read_data(uint32_t Address);






#endif /* INC_CALIBRATE_H_ */
