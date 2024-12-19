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
	uint16_t multiplex_timer_channel;
	TIM_HandleTypeDef *frame_timer;
	uint16_t frame_timer_channel;
	uint16_t frame_count;	//stores the number of frames in the animation
	uint16_t frame_index;	//stores the current index of the playing animation
	uint8_t *buffer;
} display_HandleTypeDef;

//idea: store animations as bytes. Each byte represents a row of pixels. Since each row is 8 pixels is fits perfectly
//Would be much more efficient to store animations compared to a byte for each pixel
//If i want to add brightness data in future, could expand each row to be 4 bytes,
//Then each pixel has 4 bits to store brightness value
//Or 2 bytes gives 2 bits per peixel, 4 possible birghtness levels

void display_init(display_HandleTypeDef *display, TIM_HandleTypeDef *multiplex_timer, const uint16_t multiplex_timer_channel, TIM_HandleTypeDef *frame_timer, const uint16_t frame_timer_channel);
void update_pixels(display_HandleTypeDef *display); //Turns on and off the appropriate pixels
void multiplex_interrupt(display_HandleTypeDef *display); //Triggers to multiplex LEDs
void frame_interrupt(display_HandleTypeDef *display);	//Draws the next frame in sequence



#endif /* INC_DISPLAY_H_ */
