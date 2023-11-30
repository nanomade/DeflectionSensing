/*
 * MK3_configuration.h
 *
 *  Created on: Nov 3, 2023
 *      Author: gust0
 */

#include "main.h"
#include "stdlib.h"
#include "ssd1306.h"
#include "string.h"
#include "usbd_cdc_if.h"
#include "math.h"



#ifndef INC_MK3_CONFIGURATION_H_
#define INC_MK3_CONFIGURATION_H_

//State machine states:
  enum state{
	  Display,
	  Send_USB,
	  Menu,
	  Return_from_menu
  };


//Calibrate:
#define MAX_RESOLUTION_INDEX 5	//index defining the resolution at which MS5637 measures (5 is maximum)
#define MEAN_MEASURE_N 20		//Number of pressure measurements taken before setting the mean zero-deflection pressure value
#define DEFAULT_STEP_LENGTH 500 //nm
#define DEFAULT_TRAVEL_LENGTH 1000000 //1mm in nm
#define FLASH_ADDR_BEGIN 	0x08000000	//Available flash memory address beginning
#define FLASH_ADDR_END		0x0801FFFE	//Available flash memory address ending		SEE DATASHEET section 4 "Memory mapping"
		//IMPORTANT! Keep in mind that this program is stored in flash, beginning at FLASH_ADDR_BEGIN. Therefore: when saving data, we start from the end and move towards beginning.
#define MAX_PRESSURE 108000 //See MS5637 datasheet page 7
#define MIN_PRESSURE 100000



//MS5637:
#define MS5637_ADDR 0x76 << 1 //Shifted address for the MS5637

extern uint8_t D1_RES_INDEX;
extern uint8_t D2_RES_INDEX;

// MS5637 device commands
#define MS5637_RESET_COMMAND 0x1E
#define MS5637_START_PRESSURE_ADC_CONVERSION 0x40
#define MS5637_START_TEMPERATURE_ADC_CONVERSION 0x50
#define MS5637_READ_ADC 0x00

#define MS5637_CONVERSION_OSR_MASK 0x0F

// MS5637 commands
#define MS5637_PROM_ADDRESS_READ_ADDRESS_0 0xA0
#define MS5637_PROM_ADDRESS_READ_ADDRESS_1 0xA2
#define MS5637_PROM_ADDRESS_READ_ADDRESS_2 0xA4
#define MS5637_PROM_ADDRESS_READ_ADDRESS_3 0xA6
#define MS5637_PROM_ADDRESS_READ_ADDRESS_4 0xA8
#define MS5637_PROM_ADDRESS_READ_ADDRESS_5 0xAA
#define MS5637_PROM_ADDRESS_READ_ADDRESS_6 0xAC
#define MS5637_PROM_ADDRESS_READ_ADDRESS_7 0xAE

// Coefficients indexes for temperature and pressure computation
#define MS5637_CRC_INDEX 0
#define MS5637_PRESSURE_SENSITIVITY_INDEX 1
#define MS5637_PRESSURE_OFFSET_INDEX 2
#define MS5637_TEMP_COEFF_OF_PRESSURE_SENSITIVITY_INDEX 3
#define MS5637_TEMP_COEFF_OF_PRESSURE_OFFSET_INDEX 4
#define MS5637_REFERENCE_TEMPERATURE_INDEX 5
#define MS5637_TEMP_COEFF_OF_TEMPERATURE_INDEX 6

#define MS5637_COEFFICIENT_COUNT 7

//Strictly not necessary:
#define MS5637_CONVERSION_TIME_OSR_256 1
#define MS5637_CONVERSION_TIME_OSR_512 2
#define MS5637_CONVERSION_TIME_OSR_1024 3
#define MS5637_CONVERSION_TIME_OSR_2048 5
#define MS5637_CONVERSION_TIME_OSR_4096 9
#define MS5637_CONVERSION_TIME_OSR_8192 17



//CustomUI:
#define X_AXIS_LINE_X1 	0
#define X_AXIS_LINE_X2 	105
#define X_AXIS_LINE_Y 	55

#define Y_AXIS_LINE_X	2
#define Y_AXIS_LINE_Y1	16
#define Y_AXIS_LINE_Y2	57

#define TITLE_X 15
#define GRAPH_SIZE 20
#define GRAPH_X_LABEL 100
#define GRAPH_Y_LABEL 1200
#define GRAPH_VALUE_OFFSET 100000
#define NUMBER_OF_OPTIONS 4

extern uint8_t menu_cursor_placement;	//Menu cursor placement
extern uint8_t USB_EN;




#endif /* INC_MK3_CONFIGURATION_H_ */
