/*
 * MS5637.h
 *	Header file for barometric pressure sensor MS5637-02BA03
 *	Written specifically for use with STM32 cubeIDE HAL on MK3 deflection sensor board
 *  Created on: Oct 26, 2023
 *  Author: Gustav Friis Fléron
 */

#include "MK3_configuration.h"

#ifndef INC_MS5637_H_
#define INC_MS5637_H_


int8_t MS5637_RESET(I2C_HandleTypeDef *i2c_handle);	//Resets MS5637
uint16_t MS5637_PROM_READ(I2C_HandleTypeDef *i2c_handle, uint16_t PROM_ADDR);	//Returns value on specific PROM address

void MS5637_SET_PRESS_CONV(I2C_HandleTypeDef *i2c_handle);	//Sets the pressure conversion rate
void MS5637_SET_TEMP_CONV(I2C_HandleTypeDef *i2c_handle);	//Sets the temperature conversion rate

int32_t MS5637_GET_PRESSURE(I2C_HandleTypeDef *i2c_handle);	//Reads ADC and returns pressure raw value
int32_t MS5637_GET_TEMPERATURE(I2C_HandleTypeDef *i2c_handle);	//Reads ADC and returns temperature raw value

void MS5637_SETUP(I2C_HandleTypeDef *i2c_handle, uint8_t D1_INDEX, uint8_t D2_INDEX);	//Sets up global variables for sensor, using data from PROM
void MS5637_GET_PROM_VALUES(I2C_HandleTypeDef *i2c_handle);	//Reads all PROM on MS5637 and saves in global PROM_COEF array
uint8_t MS5637_CRC_CHECK(uint8_t crc);	//CRC Check that raises error flag and turns on LED1 if sensor is defect

void MS5637_STREAM_USB(I2C_HandleTypeDef *i2c_handle1, I2C_HandleTypeDef *i2c_handle2, uint8_t D_CONV);	//Streams data from two MS5637 sensors through USB serial.
int32_t MS5637_LOCAL_GET_TEMPERATURE(I2C_HandleTypeDef *i2c_handle, char tempArr[]);
void MS5637_LOCAL_GET_PRESSURE(I2C_HandleTypeDef *i2c_handle, int32_t dT, char presArr[]);
void MS5637_LOCAL_SET_TEMP_CONV(I2C_HandleTypeDef *i2c_handle, uint8_t D_index);
void MS5637_LOCAL_SET_PRESS_CONV(I2C_HandleTypeDef *i2c_handle, uint8_t D_index);


#endif /* INC_MS5637_H_ */
