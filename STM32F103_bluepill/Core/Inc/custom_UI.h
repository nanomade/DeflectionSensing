/*
 * 	custom_UI.h
 * 	Header file for custom user interface with SSD1306 OLED display. Requires ssd1306 library
 * 	https://github.com/afiskon/stm32-ssd1306
 *
 *	Written for use with STM32 cubeIDE HAL
 *  Created on: Oct 30, 2023
 *  Author: Gustav Friis Fléron
 *
 */

#include "MK3_configuration.h"

#ifndef INC_CUSTOM_UI_H_
#define INC_CUSTOM_UI_H_

uint8_t count_digits(uint16_t number);
void setup_graph(uint16_t limit_x, uint16_t limit_y, char* title);	//Sets up graph background on display
void setup_std_graph(char* title); //Same as setup_graph, but with standard x/y labels
void update_graph(uint8_t length, uint32_t dataArr[2][length], uint32_t single_data);		//Updates graph with data
void setup_menu();
void menu_scroll();
void display_USB_stream();
void rescale_arr(uint16_t small_size, uint32_t small_arr[][small_size], uint32_t big_arr[], uint16_t big_size);


#endif /* INC_CUSTOM_UI_H_ */
