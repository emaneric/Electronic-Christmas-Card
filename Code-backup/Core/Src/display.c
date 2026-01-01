/*
 * display.c
 *
 *  Created on: Nov 1, 2024
 *      Author: eric
 */

#include <display.h>
//Should have 2 libraries, 1 to drive the multiplexed LED display.
//Another to create the pixel array with functions such as drawing text and shapes

//frame timer should have period of 42ms
//Multiplex timer needs to have a period of 41us
void display_init(display_HandleTypeDef *display, TIM_HandleTypeDef *multiplex_timer, const uint16_t pixel_channel) {
	display->multiplex_timer = multiplex_timer;
	display->pixel_channel = pixel_channel;
	display->pixel_index = 0;
}

void display_start(display_HandleTypeDef *display){
	HAL_TIM_Base_Start_IT(display->multiplex_timer);
	display->pixel_index = 0;
	__NOP();
}

void display_stop(display_HandleTypeDef *display){
	HAL_TIM_Base_Stop_IT(display->multiplex_timer);
	display->pixel_index = 0;
}

void multiplex_interrupt(display_HandleTypeDef *display){



	HAL_GPIO_WritePin(R0_GPIO_Port, R0_Pin, 1);
	HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, 1);
	HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, 1);
	HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, 1);
	HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, 1);
	HAL_GPIO_WritePin(R5_GPIO_Port, R5_Pin, 1);
	HAL_GPIO_WritePin(R6_GPIO_Port, R6_Pin, 1);
	HAL_GPIO_WritePin(R7_GPIO_Port, R7_Pin, 1);

	HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, 0);
	HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, 0);
	HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, 0);
	HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, 0);
	HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, 0);
	HAL_GPIO_WritePin(C5_GPIO_Port, C5_Pin, 0);
	HAL_GPIO_WritePin(C6_GPIO_Port, C6_Pin, 0);
	HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, 0);

	uint8_t row = display->pixel_index / 8;
	uint8_t column = display->pixel_index % 8;
	uint8_t value = display->buffer[display->pixel_index];

	if (display->pixel_index >= 64){
		display->pixel_index = 0;
	}
	else {
		display->pixel_index++;
	}

	//__NOP();

	uint8_t not_value = 0;

	if (value == 1){
		not_value = 0;
	}
	else {
		not_value = 1;
	}

	switch (row){
		case 0:
			HAL_GPIO_WritePin(R0_GPIO_Port, R0_Pin, not_value);
			break;
		case 1:
			HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, not_value);
			break;
		case 2:
			HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, not_value);
			break;
		case 3:
			HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, not_value);
			break;
		case 4:
			HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, not_value);
			break;
		case 5:
			HAL_GPIO_WritePin(R5_GPIO_Port, R5_Pin, not_value);
			break;
		case 6:
			HAL_GPIO_WritePin(R6_GPIO_Port, R6_Pin, not_value);
			break;
		case 7:
			HAL_GPIO_WritePin(R7_GPIO_Port, R7_Pin, not_value);
			break;
		default:

		}

	switch (column){
		case 0:
			HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, value);
			break;
		case 1:
			HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, value);
			break;
		case 2:
			HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, value);
			break;
		case 3:
			HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, value);
			break;
		case 4:
			HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, value);
			break;
		case 5:
			HAL_GPIO_WritePin(C5_GPIO_Port, C5_Pin, value);
			break;
		case 6:
			HAL_GPIO_WritePin(C6_GPIO_Port, C6_Pin, value);
			break;
		case 7:
			HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, value);
			break;
		default:

		}

	//__NOP();
}
