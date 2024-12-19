/*
 * display.c
 *
 *  Created on: Nov 1, 2024
 *      Author: eric
 */

#include <display.h>

//Should have 2 libraries, 1 to drive the multipexed LED display.
//Another to create the pixel array with functions such as drawing text and shapes


//frame timer should have period of 42ms
//Multiplex timer needs to have a period of 41us
void display_init(display_HandleTypeDef *display, TIM_HandleTypeDef *multiplex_timer, const uint16_t multiplex_timer_channel, TIM_HandleTypeDef *frame_timer, const uint16_t frame_timer_channel){
	display->multiplex_timer = multiplex_timer;
	display->multiplex_timer_channel = multiplex_timer_channel;
	display->frame_timer = frame_timer;
	display->frame_timer_channel = frame_timer_channel;
	display->frame_count = 0;
	display->frame_index = 0;
}


void update_pixels(display_HandleTypeDef *display){

}


//I should use DMA or PWM to eliminate the need for a software interrupt
//void multiplex_interrupt(display_HandleTypeDef *display){
//
//}

void display_start(display_HandleTypeDef *display){

	HAL_TIM_Base_Start_IT(display->frame_timer);	//start timer to automatically update frames at 24 Hz

	//Need to work on this.
	//Multiplex timer should use 2 channels 1 think
	//1 channel does the timing for each pixel, when it's interrupt fires, the next pixel is activated in the line
	//the other channel does the brightness for each pixel. It does PWM and directly controls the GPIO of column/row (need to decide) to turn LED on for right period according to brightness value
	//Brightness channel should be synced to pixel channel so at start of pixel they both start at same time.
	//HAL_TIM_PWM_Start(display->multiplex_timer , display->multiplex_timer_channel);
}

void display_stop(display_HandleTypeDef *display){

	HAL_TIM_Base_Stop_IT(display->frame_timer);
}

void frame_interrupt(display_HandleTypeDef *display){

	//If we have not reached the end of the animation
	if (display->frame_index < display->frame_count){
		display->frame_index++;
	}

	//We have reached the end of the animation
	else {
		display->frame_index = 0;
	}

	HAL_GPIO_TogglePin(R0_GPIO_Port, R0_Pin);	//For testing, make sure it is toggling at 24Hz
}












































////Needs to be called very often to allow the multiplexing to work
////Go through every pixel in the frame buffer and write it to the display
////maybe we need to have a small delay after each pixel is lit, or not?
//void multiplex_Display(uint8_t *display_buff){
//
//	//maybe add checking so if the current pixel number and value is the same as the previous cycle, don't do anything?
//
//
//	uint8_t pixel_value = 0;
//	for(uint8_t pixel; pixel <= 63; pixel++){
//		//Caluclate the row and column to light
//		uint8_t row = display_buff[pixel] / 8;
//		uint8_t column = display_buff[pixel] % 8;
//
//		//update all the rows to light the correct pixel
//		for(uint8_t current_row; current_row <= 7; current_row++){
//			//write LOW to turn on the LED at this row
//			if (current_row == row){
//				write_Row(current_row, 0);
//			}
//			//write HIGH to turn off the LED at this row
//			else {
//				write_Row(current_row, 1);
//			}
//		}
//		//update all the columns to light the correct pixel
//		for(uint8_t current_column; current_column <= 7; current_column++){
//			//write HIGH to turn on the LED at this column
//			if (current_column == column){
//				write_Column(current_column, 1);
//			}
//			//write LOW to turn off the LED at this column
//			else {
//				write_Column(current_column, 0);
//			}
//		}
//
//
//void write_Row(uint8_t rowNumber, uint8_t value){
//	switch (rowNumber){
//		case 0:
//			HAL_GPIO_WritePin(R0_GPIO_Port, R0_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(R1_GPIO_Port, R1_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(R2_GPIO_Port, R2_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(R3_GPIO_Port, R3_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(R4_GPIO_Port, R4_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(R5_GPIO_Port, R5_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(R6_GPIO_Port, R6_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(R7_GPIO_Port, R7_Pin, value);
//			break;
//		default:
//
//		}
//}
//
//void write_Column(uint8_t columnNumber, uint8_t value){
//	switch (columnNumber){
//		case 0:
//			HAL_GPIO_WritePin(C0_GPIO_Port, C0_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(C1_GPIO_Port, C1_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(C2_GPIO_Port, C2_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(C3_GPIO_Port, C3_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(C4_GPIO_Port, C4_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(C5_GPIO_Port, C5_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(C6_GPIO_Port, C6_Pin, value);
//			break;
//		case 1:
//			HAL_GPIO_WritePin(C7_GPIO_Port, C7_Pin, value);
//			break;
//		default:
//
//		}
//}
