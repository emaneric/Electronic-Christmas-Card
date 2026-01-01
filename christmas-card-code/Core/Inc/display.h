/*
 * display.h
 *
 *  Created on: Nov 1, 2024
 *      Author: eric
 */

#ifndef INC_DISPLAY_H_
#define INC_DISPLAY_H_

#include <main.h>

typedef struct __display_HandleTypeDef {
	TIM_HandleTypeDef *multiplex_timer;
	uint16_t pixel_channel;
	uint8_t pixel_index;
	uint8_t buffer[64];
} display_HandleTypeDef;

//idea: store animations as bytes. Each byte represents a row of pixels. Since each row is 8 pixels is fits perfectly
//Would be much more efficient to store animations compared to a byte for each pixel

void display_init(display_HandleTypeDef *display, TIM_HandleTypeDef *multiplex_timer, const uint16_t pixel_channel);
void display_start(display_HandleTypeDef *display);
void display_stop(display_HandleTypeDef *display);
void multiplex_interrupt(display_HandleTypeDef *display); //Triggers to multiplex LEDs

#endif /* INC_DISPLAY_H_ */
