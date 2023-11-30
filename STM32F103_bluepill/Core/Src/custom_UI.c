/*
 * 	custom_UI.c
 * 	Source file for custom user interface with SSD1306 OLED display. Requires ssd1306 library
 * 	https://github.com/afiskon/stm32-ssd1306
 *
 *	Written for use with STM32 cubeIDE HAL
 *  Created on: Oct 30, 2023
 *  Author: Gustav Friis Fléron
 *
 */

#include "custom_UI.h"

uint8_t UI_ERROR_FLAG = 0;

uint8_t count_digits(uint16_t number){
	uint8_t count = 1;
	while(number > 10){
		number = number/10;
		count += 1;
	}
	return count;
}

void setup_std_graph(char* title){

	/*
	 * Initializes a basic graph setup, with a given 4-5 letter title and x/y axis limits.
	 * The limits are supposed to run up to maximum 999, in order to facilitate 999 micrometer deflection.
	 */

	char ssd1306_return;
	char x_str[5];
	char y_str[5];

	itoa(GRAPH_X_LABEL, x_str, 10);	//convert integer limits to strings and place within char arrays
	itoa(GRAPH_Y_LABEL, y_str, 10);

	uint8_t delta_x = count_digits(GRAPH_Y_LABEL);

	ssd1306_Fill(Black);	//clear display

	ssd1306_SetCursor(TITLE_X, 0);
	ssd1306_return = ssd1306_WriteString(title, Font_11x18, White);	//Write title

	//ssd1306_SetCursor(TITLE_X + (delta_x*5), 0);
	//ssd1306_return = ssd1306_WriteString(title, Font_11x18, White);	//Write title (the old one)

	//ssd1306_SetCursor((X_AXIS_LINE_X2 + 2), 50);
	//ssd1306_return = ssd1306_WriteString(x_str, Font_7x10, White);	//write limit

	//ssd1306_SetCursor((Y_AXIS_LINE_X + 1), 5);
	//ssd1306_return = ssd1306_WriteString(y_str, Font_7x10, White);	//write limit

	ssd1306_Line(X_AXIS_LINE_X1, X_AXIS_LINE_Y, X_AXIS_LINE_X2, X_AXIS_LINE_Y, White);	//x1, y1, x2, y2, colour	(x)		place lines
	ssd1306_Line(Y_AXIS_LINE_X, Y_AXIS_LINE_Y1, Y_AXIS_LINE_X, Y_AXIS_LINE_Y2, White);		//x1, y1, x2, y2, colour	(y)

	if(!ssd1306_return){	//raise global flag if error
		UI_ERROR_FLAG = 1;
	}

	ssd1306_UpdateScreen();	//Update OLED display
}

void setup_graph(uint16_t limit_x, uint16_t limit_y, char* title){

	/*
	 * Initializes a basic graph setup, with a given 4-5 letter title and x/y axis limits.
	 * The limits are supposed to run up to maximum 999, in order to facilitate 999 micrometer deflection.
	 */

	char ssd1306_return;
	char x_str[5];
	char y_str[5];

	itoa(limit_x, x_str, 10);	//convert integer limits to strings and place within char arrays
	itoa(limit_y, y_str, 10);

	uint8_t delta_x = count_digits(limit_y);

	ssd1306_Fill(Black);	//clear display

	ssd1306_SetCursor(TITLE_X + (delta_x*5), 0);
	ssd1306_return = ssd1306_WriteString(title, Font_11x18, White);	//Write title

	ssd1306_SetCursor((X_AXIS_LINE_X2 + 2), 50);
	ssd1306_return = ssd1306_WriteString(x_str, Font_7x10, White);	//write limit

	ssd1306_SetCursor((Y_AXIS_LINE_X + 1), 5);
	ssd1306_return = ssd1306_WriteString(y_str, Font_7x10, White);	//write limit

	ssd1306_Line(X_AXIS_LINE_X1, X_AXIS_LINE_Y, X_AXIS_LINE_X2, X_AXIS_LINE_Y, White);	//x1, y1, x2, y2, colour	(x)		place lines
	ssd1306_Line(Y_AXIS_LINE_X, Y_AXIS_LINE_Y1, Y_AXIS_LINE_X, Y_AXIS_LINE_Y2, White);		//x1, y1, x2, y2, colour	(y)

	if(!ssd1306_return){	//raise global flag if error
		UI_ERROR_FLAG = 1;
	}

	ssd1306_UpdateScreen();	//Update OLED display
}

void update_graph(uint8_t length, uint32_t dataArr[2][length], uint32_t single_data){

	/*
	 * Updates graph with data.
	 * dataArr is two dimensional array (2 rows, "length" columns), first row carries x-values, second row carries y-values.
	 * length is the length of dataArr (how many data points it contains)
	 * x/y offset ensures data points are placed within graph setup.
	 *
	 * Newest addition: It also writes the newest data point.
	 */

	char ssd1306_return;
	char live_data[count_digits(single_data)+1];
	itoa(single_data, live_data, 10);

	uint32_t newArr[length];

	//uint8_t delta_x = count_digits(GRAPH_Y_LABEL);	//Used for old "single_data" placement
	ssd1306_FillRectangle(3, 54, 104, 16, Black);	//clear graph area
	ssd1306_FillRectangle(4, 60, 104, 56, Black);	//clear graph area
	ssd1306_FillRectangle(TITLE_X+50, 14, 128, 0, Black);	//clear single_data area
	uint8_t max_index = 0;

	for(uint8_t i = 0; i<length; i++){				//find index containing max value :)
		if(dataArr[1][i]>dataArr[1][max_index]){
			max_index = i;
		}
	}
	for(uint8_t i = 0; i<length; i++){
		newArr[i] = ((dataArr[1][i]*(X_AXIS_LINE_Y-Y_AXIS_LINE_Y1))/dataArr[1][max_index]);
	}
	/*
	if(dataArr[1][max_index] >= (Y_AXIS_LINE_Y2-Y_AXIS_LINE_Y1)){	//Scales all array values to always fit within the graph
		for(uint8_t i = 0; i<length; i++){
			newArr[1][i] = (dataArr[1][i]*(Y_AXIS_LINE_Y2-Y_AXIS_LINE_Y1))/dataArr[1][max_index];
		}
	}*/

	uint8_t x_offset = (Y_AXIS_LINE_X + 1);
	uint8_t y_offset = X_AXIS_LINE_Y;

	for(int i=1; i < length; i++){	//place graph lines
		//ssd1306_Line(((dataArr[0][i-1])+x_offset), (y_offset-(dataArr[1][i-1])), ((dataArr[0][i])+x_offset), (y_offset-(dataArr[1][i])), White);	//x1, y1, x2, y2, colour
		ssd1306_Line(((dataArr[0][i-1])+x_offset), (y_offset-(newArr[i-1])), ((dataArr[0][i])+x_offset), (y_offset-(newArr[i])), White);
	}

	//New one:
	ssd1306_SetCursor(TITLE_X + 50, 0);	//Set cursor just after title
	ssd1306_return = ssd1306_WriteString(live_data, Font_11x18, White);	//Write newest data point

	//Old one:
	//ssd1306_SetCursor(TITLE_X + (delta_x*5) + 45, 0);	//Set cursor just after title
	//ssd1306_return = ssd1306_WriteString(live_data, Font_7x10, White);	//Write newest data point

	if(!ssd1306_return){	//raise global flag if error
			UI_ERROR_FLAG = 1;
	}

	ssd1306_UpdateScreen();		//update OLED display
}

void setup_menu(){
	ssd1306_Fill(Black);	//clear display

	char D1_str[2];		//NOTE: We don't differ between D1 and D2 resolution, so we just use D1 for now
	char USB_str[2];

	itoa(D1_RES_INDEX, D1_str, 10);
	itoa(USB_EN, USB_str, 10);

	char title[5] = "MENU";
	char option1[9] = "DEF RES:";
	char option2[8] = "USB EN:";
	char option3[11] = "USB Stream";
	char option4[12] = "Display DEF";
	char ssd1306_return;

	ssd1306_SetCursor(TITLE_X, 0);
	ssd1306_return = ssd1306_WriteString(title, Font_11x18, White);	//Write title

	ssd1306_SetCursor(TITLE_X, 20);
	ssd1306_return = ssd1306_WriteString(option1, Font_7x10, White);	//Write first option
	ssd1306_SetCursor(TITLE_X + 60, 20);
	ssd1306_return = ssd1306_WriteString(D1_str, Font_7x10, White);	//Write value for first option

	ssd1306_SetCursor(TITLE_X, 30);
	ssd1306_return = ssd1306_WriteString(option2, Font_7x10, White);	//Write second option
	ssd1306_SetCursor(TITLE_X + 60, 30);
	ssd1306_return = ssd1306_WriteString(USB_str, Font_7x10, White);	//Write value for second option

	ssd1306_SetCursor(TITLE_X, 40);
	ssd1306_return = ssd1306_WriteString(option3, Font_7x10, White);	//Write third option

	ssd1306_SetCursor(TITLE_X, 50);
	ssd1306_return = ssd1306_WriteString(option4, Font_7x10, White);	//Write fourth option

	ssd1306_UpdateScreen();	//Update OLED display

}

void menu_scroll(){

	setup_menu();

	char arrow[2] = ">";
	char ssd1306_return;

	if(menu_cursor_placement > NUMBER_OF_OPTIONS){
		menu_cursor_placement = NUMBER_OF_OPTIONS;
	} else if(menu_cursor_placement < 1){
		menu_cursor_placement = 1;
	}

	ssd1306_SetCursor(TITLE_X-10, menu_cursor_placement*10 + 10);
	ssd1306_return = ssd1306_WriteString(arrow, Font_7x10, White);	//Write title

	ssd1306_UpdateScreen();	//Update OLED display
}

void display_USB_stream(){
	ssd1306_Fill(Black);	//clear display
	char title1[10] = "Streaming";
	char title2[10] = "to USB...";
	char ssd1306_return;

	ssd1306_SetCursor(TITLE_X, 20);
	ssd1306_return = ssd1306_WriteString(title1, Font_11x18, White);	//Write title
	ssd1306_SetCursor(TITLE_X, 40);
	ssd1306_return = ssd1306_WriteString(title2, Font_11x18, White);	//Write title
	ssd1306_UpdateScreen();	//Update OLED display
}

void rescale_arr(uint16_t small_size, uint32_t small_arr[][small_size], uint32_t big_arr[], uint16_t big_size){

	uint32_t big_mean = 0; //I know uint32_t may be too small... But let's fit the memory allocation when we have values.
	uint16_t peaks_above_mean = 0;

	for(int16_t i=0; i<big_size; i++){
		big_mean += big_arr[i];
	}
	big_mean = big_mean/big_size;

	for(int16_t i=1; i<big_size-1; i++){
		if(big_arr[i] > big_mean && big_arr[i]>=big_arr[i-1] && big_arr[i]>=big_arr[i+1]){
			peaks_above_mean += 1;
		}
	}

	//These are here to making reading easier:
	float compression_factor = (float)small_size/(float)big_size;	//How much the array is compressed
	uint32_t peak_arr[peaks_above_mean][2];

	for(uint16_t i=0; i<peaks_above_mean; i++){
		peak_arr[i][0] = big_mean;
		peak_arr[i][1] = 0;
	}

	//peak_arr[peaks_above_mean-1][0] = 0;
	//peak_arr[peaks_above_mean-1][1] = 0;

	for(int16_t i=1; i<big_size-1; i++){
		if(big_arr[i] >= peak_arr[peaks_above_mean-1][0] && big_arr[i]>=big_arr[i-1] && big_arr[i]>=big_arr[i+1]){
			for(int16_t j=peaks_above_mean-1; j>0; j--){
				peak_arr[j][0] = peak_arr[j-1][0];
				peak_arr[j][1] = peak_arr[j-1][1];
			}
			peak_arr[0][0] = big_arr[i];
			peak_arr[0][1] = i;
		}
	}

	for(int16_t i=0; i<small_size; i++){
		small_arr[1][i] = big_mean;					//Assign all values in small_arr to be equal to mean of big_arr
		small_arr[0][i] = i*round(100/small_size);	//Normalize to 100 pixels
	}

	uint16_t new_index;

	for(int16_t i=0; i<peaks_above_mean; i++){
		new_index = round((float)peak_arr[i][1]*compression_factor);	//Compress index
		small_arr[1][new_index] = peak_arr[i][0];							//Assign new value to small_arr at compressed index
	}

}









