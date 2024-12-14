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
	uint8_t state;	//may not be needed
	uint8_t melody_mode;
	uint16_t melody_index;
	uint16_t melody_length;
	uint16_t melody[6][2];	//need to make larger or dynamically size
} buzzer_HandleTypeDef;


//use interrupt timer for melody control

void buzzer_init(buzzer_HandleTypeDef *buzzer, TIM_HandleTypeDef *buzz_timer, const uint16_t channel_1, const uint16_t channel_2, TIM_HandleTypeDef *interrupt_timer);
void buzzer_off(buzzer_HandleTypeDef *buzzer);	//Called when timer interrupt is triggered from main. This will turn off the buzzer
void buzzer_write(buzzer_HandleTypeDef *buzzer, uint16_t frequency, uint16_t duration);
void buzzer_play_melody(buzzer_HandleTypeDef *buzzer, uint16_t *melody[6][2]);

#endif /* INC_BUZZER_H_ */
