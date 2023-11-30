/*
 * calibrate.c
 *	Source file for calibration of deflection sensor.
 *	Written specifically for use with STM32 cubeIDE HAL on MK3 deflection sensor board
 *  Created on: Nov 02, 2023
 *  Author: Gustav Friis Fléron
 */

#include "calibrate.h"

void calibrate_deflection(I2C_HandleTypeDef *i2c_handle){

	/*
	 * Writes a look-up table into FLASH memory, in order to calibrate and linearize deflection sensor.
	 * Look-up table is written into FLASH to prevent it from getting deleted on reset.
	 * PA0 is used to send pulse signals to stepper motor, which drives a linear actuator to press onto deflection sensor.
	 * To prevent issues with mechanical back-lash in the actuator, the system is set to only drive the stepper in one direction.
	 * After the initial data collection, a second while loop corrects for the initial error.
	 * The initial error is caused by not knowing when contact is established - interpolation between points displace all data points,
	 * to correct for this error.
	*/

	MS5637_SETUP(i2c_handle, MAX_RESOLUTION_INDEX, MAX_RESOLUTION_INDEX);

	uint32_t count = 0;
	uint64_t sum = 0;
	int32_t max_pressure_fluctuation = 0;	//NOTE: Change name into "max_value" or something
	int32_t pressure_baseline;
	int32_t pressure;

	while(pressure >= MAX_PRESSURE){

		//Send pulse to stepper and read pressure:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);	//PA0 on sensor board
		pressure = MS5637_GET_PRESSURE(i2c_handle);			//Use GET_PRESSURE as delay, since 17ms delay is required in function to wait for data arrival (given maximum resolution).
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);	//NOTE: Possibly add more delay or rearrange measurement line, since movement of motor will cause vibration?

		if(count < MEAN_MEASURE_N){
			sum += pressure;
			if(pressure > max_pressure_fluctuation){	//Used to (hopefully) find the greatest value, indicating how bad the noise floor is
				max_pressure_fluctuation = pressure;	//Unsigned, so no need for abs()
			}
		} else if(count == MEAN_MEASURE_N){
			pressure_baseline = sum/MEAN_MEASURE_N;			//Mean of pressure measurements
			save_data(FLASH_ADDR_END, pressure_baseline);	//NOTE: Use this value on start up to check for leakage
			//NOTE: let the pressure baseline define a "flat" level
			//When a measurement is significantly different from baseline, let THE NEXT measurement define the slope and the crossing with the flat baseline
			//This ensures that we can define exactly when contact is established!
		} else if(pressure > (pressure_baseline + (2*max_pressure_fluctuation))){	//VALID MEASUREMENT POINTS!

			save_data(FLASH_ADDR_END - (count - MEAN_MEASURE_N), pressure);	//Save measurement to FLASH
		}
		count += 1;
	}

	count = 3;	//3, since 1 and 2 are used to find start_error
	int32_t new_meas;
	int32_t first_meas = read_data(FLASH_ADDR_END - 1);
	int32_t second_meas = read_data(FLASH_ADDR_END - 2);

	int32_t start_error = DEFAULT_STEP_LENGTH + ((2*DEFAULT_STEP_LENGTH)-DEFAULT_STEP_LENGTH) * ((pressure_baseline-second_meas) / (second_meas-first_meas));
	first_meas = second_meas;

	while(second_meas <= MAX_PRESSURE){

		second_meas = read_data(FLASH_ADDR_END - count);	//Get new measurement

		//Do interpolation to find new (correct) displacement measurement:
		new_meas = first_meas + ((second_meas)-first_meas) * ((((DEFAULT_STEP_LENGTH*count)-start_error)-(DEFAULT_STEP_LENGTH*count)) / ((DEFAULT_STEP_LENGTH*(count+1))-(DEFAULT_STEP_LENGTH*count)));

		//Save new measurement in address of "first measurement"
		save_data((FLASH_ADDR_END - (count-1)), (uint32_t)new_meas);
		first_meas = second_meas;
	}
}

void save_data(uint32_t Address, uint32_t data){

	//Saves data in FLASH memory

    HAL_FLASH_Unlock();
	FLASH_EraseInitTypeDef EraseInitStruct;
	EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
	EraseInitStruct.PageAddress = Address;
	EraseInitStruct.NbPages = 1;

	uint32_t PageError;
	HAL_FLASHEx_Erase(&EraseInitStruct, &PageError);
	//if (HAL_FLASHEx_Erase(&EraseInitStruct, &PageError) != HAL_OK)		//Erase the Page Before a Write Operation
			//return HAL_ERROR;

	HAL_Delay(50);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,Address,(uint64_t)data);
	HAL_Delay(50);
	HAL_FLASH_Lock();

}

uint32_t read_data(uint32_t Address){

	//Reads data saved in FLASH memory

	__IO uint32_t read_data = *(__IO uint32_t *)Address;
	return (uint32_t)read_data;
}






