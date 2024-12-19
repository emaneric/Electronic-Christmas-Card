/*
 * buzzer.h
 *
 *  Created on: Dec 11, 2024
 *      Author: eric
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include <main.h>

typedef struct __buzzer_HandleTypeDef {
	TIM_HandleTypeDef *buzz_timer;
	uint16_t channel_1;
	uint16_t channel_2;
	TIM_HandleTypeDef *interrupt_timer;
	uint8_t melody_mode;
	uint16_t melody_index;
	uint8_t melody_loop_index;
	uint8_t melody_loops;
	uint16_t melody_length;
	uint16_t (*melody)[2]; // Pointer to a 2D array with 2 columns
} buzzer_HandleTypeDef;

void buzzer_init(buzzer_HandleTypeDef *buzzer, TIM_HandleTypeDef *buzz_timer, const uint16_t channel_1, const uint16_t channel_2, TIM_HandleTypeDef *interrupt_timer);
void buzzer_interrupt(buzzer_HandleTypeDef *buzzer);
void buzzer_off(buzzer_HandleTypeDef *buzzer);
void buzzer_write(buzzer_HandleTypeDef *buzzer, uint16_t frequency, uint16_t duration);
void buzzer_play_melody(buzzer_HandleTypeDef *buzzer, uint8_t loops);

#endif /* INC_BUZZER_H_ */
