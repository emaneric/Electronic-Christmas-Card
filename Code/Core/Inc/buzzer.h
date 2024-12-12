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
	uint16_t frequency;	//may not be needed since frequency is passed to finctions to be used immediatley anyway
	uint8_t state;	//may not be needed
	uint16_t melody_index;
} buzzer_HandleTypeDef;


void buzzer_init(buzzer_HandleTypeDef *buzzer, TIM_HandleTypeDef *buzz_timer, const uint16_t channel_1, const uint16_t channel_2, TIM_HandleTypeDef *interrupt_timer);
//void buzzer_tick(buzzer_HandleTypeDef *buzzer, uint16_t sys_tick);
void buzzer_interrupt(buzzer_HandleTypeDef *buzzer);	//Called when timer interrupt is triggered from main. This will turn off the buzzer
void buzzer_write(buzzer_HandleTypeDef *buzzer, uint16_t frequency, uint16_t duration);




#endif /* INC_BUZZER_H_ */
