/*
 * MS5637.c
 *	Source file for barometric pressure sensor MS5637-02BA03
 *	Written for use with STM32 cubeIDE HAL
 *  Created on: Oct 26, 2023
 *  Author: Gustav Friis Fléron
 */

#include "MS5637.h"

uint16_t PROM_COEF[7];

uint8_t CONVERSION_DELAY[6] = {
		MS5637_CONVERSION_TIME_OSR_256,
		MS5637_CONVERSION_TIME_OSR_512,
		MS5637_CONVERSION_TIME_OSR_1024,
		MS5637_CONVERSION_TIME_OSR_2048,
		MS5637_CONVERSION_TIME_OSR_4096,
		MS5637_CONVERSION_TIME_OSR_8192
};

uint8_t D1_RESOLUTION[6] = {
		0x40,
		0x42,
		0x44,
		0x46,
		0x48,
		0x4A
};

uint8_t D2_RESOLUTION[6] = {
		0x50,
		0x52,
		0x54,
		0x56,
		0x58,
		0x5A
};

uint8_t ERROR_FLAG = 0;

int32_t GLOBAL_dT = 0;		//NOTE: Kinda sus to have a single global variable, when there's TWO sensors.

int8_t MS5637_RESET(I2C_HandleTypeDef *i2c_handle){
	//Takes i2c handle as argument, transmits reset command to the address of sensor.
	//If unsuccessful, return 0, else return 1.

	uint8_t ret = 9;
	uint8_t WData_BUFFER[1];
	WData_BUFFER[0] = MS5637_RESET_COMMAND;
	ret = HAL_I2C_Master_Transmit(i2c_handle, MS5637_ADDR, WData_BUFFER, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		ERROR_FLAG = 1;
		return 0;
	} else{
		return 1;
	}
}

uint16_t MS5637_PROM_READ(I2C_HandleTypeDef *i2c_handle, uint16_t PROM_ADDR){
	//Takes i2c handle and PROM address as arguments, reads value stored in PROM at given address
	//If successful, returns value stored in PROM as 16-bit unsigned integer.
	//If unsuccessful, returns 0

	HAL_StatusTypeDef ret;
	uint16_t PROM_DATA;
	uint8_t RData_BUFFER[2];

	ret = HAL_I2C_Mem_Read(i2c_handle, MS5637_ADDR, PROM_ADDR, 1, RData_BUFFER, 2, HAL_MAX_DELAY);

	if(ret != HAL_OK){
		return 0;
		ERROR_FLAG = 1;
	} else{
		PROM_DATA = (RData_BUFFER[0]<<8) | RData_BUFFER[1];
		return PROM_DATA;
	}
}

void MS5637_SET_PRESS_CONV(I2C_HandleTypeDef *i2c_handle){
	//Takes i2c handle as argument, transmits D1 set command to the address of sensor.
	//If unsuccessful, return 0, else return 1.

	HAL_StatusTypeDef ret;
	uint8_t WData_BUFFER[1];
	WData_BUFFER[0] = (D1_RESOLUTION[D1_RES_INDEX]);
	ret = HAL_I2C_Master_Transmit(i2c_handle, MS5637_ADDR, WData_BUFFER, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		ERROR_FLAG = 1;
	}
}

void MS5637_SET_TEMP_CONV(I2C_HandleTypeDef *i2c_handle){
	//Takes i2c handle as argument, transmits D1 set command to the address of sensor.
	//If unsuccessful, return 0, else return 1.

	HAL_StatusTypeDef ret;
	uint8_t WData_BUFFER[1];
	WData_BUFFER[0] = (D2_RESOLUTION[D2_RES_INDEX]);
	ret = HAL_I2C_Master_Transmit(i2c_handle, MS5637_ADDR, WData_BUFFER, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		ERROR_FLAG = 1;
	}
}

int32_t MS5637_GET_PRESSURE(I2C_HandleTypeDef *i2c_handle){

	HAL_StatusTypeDef ret;
	uint32_t ADC_DATA;
	int64_t OFF, SENS;
	int32_t P;
	uint8_t RData_BUFFER[3];

	MS5637_SET_PRESS_CONV(i2c_handle);
	HAL_Delay(CONVERSION_DELAY[D1_RES_INDEX]);	//NOTE: TESTING! MUST BE DELETED!!

	ret = HAL_I2C_Mem_Read(i2c_handle, MS5637_ADDR, MS5637_READ_ADC, 1, RData_BUFFER, 3, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 0;
		ERROR_FLAG = 1;
	} else{
		ADC_DATA = (RData_BUFFER[0]<<16) | (RData_BUFFER[1]<<8) | RData_BUFFER[2];

		// OFF = OFF_T1 + TCO * dT
		OFF = ((int64_t)(PROM_COEF[MS5637_PRESSURE_OFFSET_INDEX]) << 17) + (((int64_t)(PROM_COEF[MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * GLOBAL_dT) >> 6);
		 // Sensitivity at actual temperature = SENS_T1 + TCS * dT
		SENS = ((int64_t)PROM_COEF[MS5637_PRESSURE_SENSITIVITY_INDEX] << 16) + (((int64_t)PROM_COEF[MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * GLOBAL_dT) >> 7);
		// Temperature compensated pressure = D1 * SENS - OFF
		//P = (((ADC_DATA * SENS) >> 21) - OFF) >> 15;
		P = (((int32_t)ADC_DATA)*(SENS >> 21)-OFF)>>15;

		return P;
	}
}

int32_t MS5637_GET_TEMPERATURE(I2C_HandleTypeDef *i2c_handle){

	//Temperature is not going through second order compensation, since I assume we do not work at low temperatures (<20 degC).
	//Stacking is usually done with temperatures in range 20-200 degC

	HAL_StatusTypeDef ret;
	uint32_t ADC_DATA;
	int32_t dT;
	int32_t TEMP;
	uint8_t RData_BUFFER[3];

	MS5637_SET_TEMP_CONV(i2c_handle);

	HAL_Delay(CONVERSION_DELAY[D2_RES_INDEX]);	//NOTE: TESTING! MUST BE DELETED!!

	ret = HAL_I2C_Mem_Read(i2c_handle, MS5637_ADDR, MS5637_READ_ADC, 1, RData_BUFFER, 3, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 0;
		ERROR_FLAG = 1;
	} else{
		ADC_DATA = (RData_BUFFER[0]<<16) | (RData_BUFFER[1]<<8) | RData_BUFFER[2];

		dT = (int32_t)ADC_DATA - ((int32_t)PROM_COEF[MS5637_REFERENCE_TEMPERATURE_INDEX] << 8);
		TEMP = 2000 + (dT * (int32_t)PROM_COEF[MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23);

		GLOBAL_dT = dT;	//Update the global variable
		return TEMP;
	}
}


void MS5637_SETUP(I2C_HandleTypeDef *i2c_handle, uint8_t D1_INDEX, uint8_t D2_INDEX){

	MS5637_RESET(i2c_handle);			//Reset sensor
	HAL_Delay(50);						//Give sensor time to reset
	MS5637_GET_PROM_VALUES(i2c_handle);	//Fills PROM_COEF array with coefficient values from the sensor PROM.
	D1_RES_INDEX = D1_INDEX;			//Define conversion resolution by index (specific D1/D2 values found in global arrays)
	D2_RES_INDEX = D2_INDEX;

	MS5637_GET_TEMPERATURE(i2c_handle);	//Get temperature to update GLOBAL_dT. This is to ensure somewhat proper measurement of pressure, even when temperature hasn't been measured.

	if(ERROR_FLAG){
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13, GPIO_PIN_SET);	//NOTE: Maybe change LED to D3?
	}
}

void MS5637_GET_PROM_VALUES(I2C_HandleTypeDef *i2c_handle){

	for(int i = 0; i < MS5637_COEFFICIENT_COUNT; i++){
		PROM_COEF[i] = MS5637_PROM_READ(i2c_handle, MS5637_PROM_ADDRESS_READ_ADDRESS_0 + (i * 2));
	} if (!MS5637_CRC_CHECK((PROM_COEF[MS5637_CRC_INDEX] & 0xF000) >> 12)){
		ERROR_FLAG = 1;
	}
}

uint8_t MS5637_CRC_CHECK(uint8_t crc) {
	//
	uint8_t cnt, n_bit;
	uint16_t n_rem, crc_read;

	n_rem = 0x00;
	crc_read = PROM_COEF[0];
	PROM_COEF[MS5637_COEFFICIENT_COUNT] = 0;
	PROM_COEF[0] = (0x0FFF & (PROM_COEF[0])); // Clear the CRC byte

	for (cnt = 0; cnt < (MS5637_COEFFICIENT_COUNT + 1) * 2; cnt++) {
		// Get next byte
		if (cnt % 2 == 1){
			n_rem ^= PROM_COEF[cnt >> 1] & 0x00FF;
		}else{
			n_rem ^= PROM_COEF[cnt >> 1] >> 8;
		}

		for (n_bit = 8; n_bit > 0; n_bit--) {

			if (n_rem & 0x8000){
				n_rem = (n_rem << 1) ^ 0x3000;
			}else{
				n_rem <<= 1;
			}
		}
	}
	n_rem >>= 12;
	PROM_COEF[0] = crc_read;

	return (n_rem == crc);
}




void MS5637_STREAM_USB(I2C_HandleTypeDef *i2c_handle1, I2C_HandleTypeDef *i2c_handle2, uint8_t D_CONV){

	/*
	 * Takes two i2c handles (one for each MS5637 sensor) and a D_RES conversion int.
	 * Five char arrays corresponds to:
	 * Two char arrays for pressure and two for temperature (one for each sensor)
	 * One char array for timer value.
	 * Each char array is used to store a value that is sent through USB serial at the end of function.
	 * Timer provides the time it takes between sending USB packages. At high res this is about 38ms
	 * TIMER UNIT IS ms!		(Set by tweaking overflow val and prescaler in .ioc file)
	 * Only one package is sent in the end, this is due to serial being slow as fuck.
	 * After package is sent, timer is reset.
	 * Timer overflow handler in main stops timer if 10s passes and the current state is not "stream usb"
	 */

	char tempArr1[5];// = {',', ',', ',', ',', ','};	//Whoa so easy to count! (There's five commas)
	char tempArr2[5];// = {',', ',', ',', ',', ','};

	char presArr1[7];// = {',', ',', ',', ',', ',', ',', ','};
	char presArr2[7];// = {',', ',', ',', ',', ',', ',', ','};

	char TIM_VAL[11];

	uint8_t presL1 = 0;
	uint8_t presL2 = 0;
	uint8_t TIML = 0;

	int32_t dT1;
	int32_t dT2;

	MS5637_LOCAL_SET_TEMP_CONV(i2c_handle1, D_CONV);
	MS5637_LOCAL_SET_TEMP_CONV(i2c_handle2, D_CONV);

	HAL_Delay(CONVERSION_DELAY[D_CONV]); //NOTE: I should find a better way of doing this...

	dT1 = MS5637_LOCAL_GET_TEMPERATURE(i2c_handle1, tempArr1);
	dT2 = MS5637_LOCAL_GET_TEMPERATURE(i2c_handle2, tempArr2);

	MS5637_LOCAL_SET_PRESS_CONV(i2c_handle1, D_CONV);
	MS5637_LOCAL_SET_PRESS_CONV(i2c_handle2, D_CONV);

	HAL_Delay(CONVERSION_DELAY[D_CONV]); //NOTE: I should find a better way of doing this...

	MS5637_LOCAL_GET_PRESSURE(i2c_handle1, dT1, presArr1);
	MS5637_LOCAL_GET_PRESSURE(i2c_handle2, dT2, presArr2);

	itoa((int)TIM3->CNT, TIM_VAL, 10);

	for(uint8_t i = 0; i<sizeof(TIM_VAL);i++){
		if(TIM_VAL[i] == '\0' && TIML == 0){
			TIML = i+1;
			TIM_VAL[i] = ',';
		}
		if(presArr1[i] == '\0' && presL1 == 0){
			presL1 = i+1;
			presArr1[i] = ',';
		}
		if(presArr2[i] == '\0' && presL2 == 0){
			presL2 = i+1;
			presArr2[i] = ',';
		}
	}

	tempArr1[4] = ',';	//Because the value is always four digits and the fifth index has to be a comma
	uint8_t USB_data[TIML + presL1 + sizeof(tempArr1) + presL2 + sizeof(tempArr2) + 1];

	memcpy(USB_data, TIM_VAL, TIML);
	memcpy(USB_data + TIML, presArr1, presL1);
	memcpy(USB_data + TIML + presL1, tempArr1, sizeof(tempArr1));
	memcpy(USB_data + TIML + presL1 + sizeof(tempArr1), presArr2, presL2);
	memcpy(USB_data + TIML + presL1 + sizeof(tempArr1) + presL2, tempArr2, sizeof(tempArr2));

	char end[2] = {'\n', '\0'};
	memcpy(USB_data + TIML + presL1 + sizeof(tempArr1) + presL2 + sizeof(tempArr2) - 1, end, sizeof(end));

	CDC_Transmit_FS(USB_data, sizeof(USB_data));
	TIM3->CNT = 0;	//Reset counter :)
}

int32_t MS5637_LOCAL_GET_TEMPERATURE(I2C_HandleTypeDef *i2c_handle, char tempArr[]){

	/*
	 * Function takes i2c handle and pointer to array.
	 * Returns dT and uses itoa to fill temperature value into array
	 * DOES NOT INCLUDE delay!! Therefore caution must be taken when this is used, otherwise temperature value will be wrong.
	 * See MS5637 datasheet page 10.
	 */

	//Temperature is not going through second order compensation, since I assume we do not work at low temperatures (<20 degC).
	//Stacking is usually done with temperatures in range 20-200 degC

	HAL_StatusTypeDef ret;
	uint32_t ADC_DATA;
	int32_t dT;
	int32_t TEMP;
	uint8_t RData_BUFFER[3];

	//MS5637_SET_TEMP_CONV(i2c_handle);
	//HAL_Delay(CONVERSION_DELAY[D2_RES_INDEX]);	//NOTE: TESTING! MUST BE DELETED!!

	ret = HAL_I2C_Mem_Read(i2c_handle, MS5637_ADDR, MS5637_READ_ADC, 1, RData_BUFFER, 3, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		return 0;
		ERROR_FLAG = 1;
	} else{
		ADC_DATA = (RData_BUFFER[0]<<16) | (RData_BUFFER[1]<<8) | RData_BUFFER[2];

		dT = (int32_t)ADC_DATA - ((int32_t)PROM_COEF[MS5637_REFERENCE_TEMPERATURE_INDEX] << 8);
		TEMP = 2000 + (dT * (int32_t)PROM_COEF[MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX] >> 23);

		itoa(TEMP, tempArr, 10);

		//GLOBAL_dT = dT;	//Update the global variable
		return dT;
	}
}

void MS5637_LOCAL_GET_PRESSURE(I2C_HandleTypeDef *i2c_handle, int32_t dT, char presArr[]){

	/*
	 * Function takes i2c handle, dT value and pointer to array.
	 * Uses itoa to fill pressure value into array
	 * DOES NOT INCLUDE delay!! Therefore caution must be taken when this is used, otherwise pressure value will be wrong.
	 * See MS5637 datasheet page 10.
	 */

	HAL_StatusTypeDef ret;
	uint32_t ADC_DATA;
	int64_t OFF, SENS;
	int32_t P;
	uint8_t RData_BUFFER[3];

	//MS5637_SET_PRESS_CONV(i2c_handle);
	//HAL_Delay(CONVERSION_DELAY[D1_RES_INDEX]);	//NOTE: TESTING! MUST BE DELETED!!

	ret = HAL_I2C_Mem_Read(i2c_handle, MS5637_ADDR, MS5637_READ_ADC, 1, RData_BUFFER, 3, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		//return 0;
		ERROR_FLAG = 1;
	} else{
		ADC_DATA = (RData_BUFFER[0]<<16) | (RData_BUFFER[1]<<8) | RData_BUFFER[2];

		// OFF = OFF_T1 + TCO * dT
		OFF = ((int64_t)(PROM_COEF[MS5637_PRESSURE_OFFSET_INDEX]) << 17) + (((int64_t)(PROM_COEF[MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX]) * dT) >> 6);
		 // Sensitivity at actual temperature = SENS_T1 + TCS * dT
		SENS = ((int64_t)PROM_COEF[MS5637_PRESSURE_SENSITIVITY_INDEX] << 16) + (((int64_t)PROM_COEF[MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX] * dT) >> 7);
		// Temperature compensated pressure = D1 * SENS - OFF
		//P = (((ADC_DATA * SENS) >> 21) - OFF) >> 15;
		P = (((int32_t)ADC_DATA)*(SENS >> 21)-OFF)>>15;

		itoa(P, presArr, 10);

		//return P;
	}
}

void MS5637_LOCAL_SET_TEMP_CONV(I2C_HandleTypeDef *i2c_handle, uint8_t D_index){
	//Takes i2c handle as argument, transmits D1 set command to the address of sensor.
	//If unsuccessful, return 0, else return 1.

	HAL_StatusTypeDef ret;
	uint8_t WData_BUFFER[1];
	WData_BUFFER[0] = (D2_RESOLUTION[D_index]);
	ret = HAL_I2C_Master_Transmit(i2c_handle, MS5637_ADDR, WData_BUFFER, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		ERROR_FLAG = 1;
	}
}

void MS5637_LOCAL_SET_PRESS_CONV(I2C_HandleTypeDef *i2c_handle, uint8_t D_index){
	//Takes i2c handle as argument, transmits D1 set command to the address of sensor.
	//If unsuccessful, return 0, else return 1.

	HAL_StatusTypeDef ret;
	uint8_t WData_BUFFER[1];
	WData_BUFFER[0] = (D1_RESOLUTION[D_index]);
	ret = HAL_I2C_Master_Transmit(i2c_handle, MS5637_ADDR, WData_BUFFER, 1, HAL_MAX_DELAY);
	if(ret != HAL_OK){
		ERROR_FLAG = 1;
	}
}


