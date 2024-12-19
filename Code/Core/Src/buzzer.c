/*
 * buzzer.c
 *
 *  Created on: Dec 11, 2024
 *      Author: eric
 */

#include <buzzer.h>

void buzzer_init(buzzer_HandleTypeDef *buzzer, TIM_HandleTypeDef *buzz_timer, const uint16_t channel_1, const uint16_t channel_2, TIM_HandleTypeDef *interrupt_timer){

	buzzer->buzz_timer = buzz_timer;
	buzzer->channel_1 = channel_1;
	buzzer->channel_2 = channel_2;
	buzzer->interrupt_timer = interrupt_timer;
	buzzer->melody_mode = 0;
	buzzer->melody_index = 0;
	buzzer->melody_loop_index = 0;
	buzzer->melody_loops = 0;
	buzzer->melody_length = 0;
}

void buzzer_interrupt(buzzer_HandleTypeDef *buzzer){

	if (buzzer->melody_mode == 1){
		//If end of melody is reached
		if (buzzer->melody_index >= buzzer->melody_length){

			__NOP();
			if (buzzer->melody_loop_index >= buzzer->melody_loops){
				buzzer->melody_mode = 0;
				buzzer_off(buzzer);
			}
			else {
				buzzer->melody_loop_index++;
				buzzer->melody_index = 0;
				buzzer_off(buzzer); //testing
				buzzer_write(buzzer, buzzer->melody[buzzer->melody_index][0], buzzer->melody[buzzer->melody_index][1]); //testing
				//This fixes delay but doesnt sound exactly right
			}
//			buzzer->melody_mode = 0;
//			buzzer_off(buzzer);
		}
		//Not the end of melody, continue playing
		else {
			buzzer_write(buzzer, buzzer->melody[buzzer->melody_index][0], buzzer->melody[buzzer->melody_index][1]);
			buzzer->melody_index++;
			//buzzer->melody_loop_index++;
		}
	}
	else {
		buzzer_off(buzzer);
	}
}

void buzzer_off(buzzer_HandleTypeDef *buzzer){

	HAL_TIM_Base_Stop_IT(buzzer->interrupt_timer);
	HAL_TIM_PWM_Stop(buzzer->buzz_timer , buzzer->channel_1);
	HAL_TIM_PWM_Stop(buzzer->buzz_timer , buzzer->channel_2);
}

void buzzer_write(buzzer_HandleTypeDef *buzzer, uint16_t frequency, uint16_t duration){

	//Valid frequency so update registers and turn on buzzer
	if (frequency != 0){
		uint16_t ARR_VAL = (HAL_RCC_GetSysClockFreq() / frequency) - 1;
		uint16_t CCR_VAL = (ARR_VAL + 1)/ 2;
		__HAL_TIM_SET_AUTORELOAD(buzzer->buzz_timer, ARR_VAL);
		__HAL_TIM_SET_COMPARE(buzzer->buzz_timer, buzzer->channel_1, CCR_VAL);
		__HAL_TIM_SET_COMPARE(buzzer->buzz_timer, buzzer->channel_2, CCR_VAL);
		HAL_TIM_PWM_Start(buzzer->buzz_timer , buzzer->channel_1);
		HAL_TIM_PWM_Start(buzzer->buzz_timer , buzzer->channel_2);
	}
	// Frequency is 0 so turn off buzzer
	else{
		HAL_TIM_PWM_Stop(buzzer->buzz_timer , buzzer->channel_1);
		HAL_TIM_PWM_Stop(buzzer->buzz_timer , buzzer->channel_2);
	}
	/* Update interrupt timer registers to turn off the buzzer after correct delay
	 * A duration of 0 will turn the buzzer on until manually turned off with buzzer_off */
	if (duration != 0){
	//if ((duration != 0) && (buzzer->melody_index != buzzer->melody_length)){
		__HAL_TIM_SET_AUTORELOAD(buzzer->interrupt_timer, duration); //Set ARR to the duration. Since timer period is 1ms, duration in ms can be used directly
		__HAL_TIM_SET_COUNTER(buzzer->interrupt_timer, 0);	//Reset timer count to 0
		__HAL_TIM_CLEAR_FLAG(buzzer->interrupt_timer, TIM_FLAG_UPDATE);	//Clear the interrupt flag to prevent the interrupt triggering immediately
		HAL_TIM_Base_Start_IT(buzzer->interrupt_timer);
	}
}

void buzzer_play_melody(buzzer_HandleTypeDef *buzzer, uint8_t loops){

	buzzer->melody_mode = 1;
	buzzer->melody_index = 0;
	buzzer->melody_loop_index = 0;
	buzzer->melody_loops = loops - 1;
	buzzer_off(buzzer);	//Stop any currently playing melody
	buzzer_write(buzzer, buzzer->melody[buzzer->melody_index][0], buzzer->melody[buzzer->melody_index][1]);
	buzzer->melody_index++;
}
