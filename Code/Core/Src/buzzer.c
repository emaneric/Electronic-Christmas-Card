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
	buzzer->state = 0;
	buzzer->melody_mode = 0;
	buzzer->melody_index = 0;
	buzzer->melody_length = 0;
	//buzzer->melody;
}

void buzzer_off(buzzer_HandleTypeDef *buzzer){

	HAL_TIM_Base_Stop_IT(buzzer->interrupt_timer);
	HAL_TIM_PWM_Stop(buzzer->buzz_timer , TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(buzzer->buzz_timer , TIM_CHANNEL_3);
	buzzer->state = 0;
}

void buzzer_write(buzzer_HandleTypeDef *buzzer, uint16_t frequency, uint16_t duration){

	/* Update buzzer timer values to get the correct PWM frequency and 50% duty cycle */
	uint16_t ARR_VAL = (HAL_RCC_GetSysClockFreq() / frequency) - 1;
	uint16_t CCR_VAL = (ARR_VAL + 1)/ 2;
	__HAL_TIM_SET_AUTORELOAD(buzzer->buzz_timer, ARR_VAL);
	__HAL_TIM_SET_COMPARE(buzzer->buzz_timer, buzzer->channel_1, CCR_VAL);
	__HAL_TIM_SET_COMPARE(buzzer->buzz_timer, buzzer->channel_2, CCR_VAL);
	HAL_TIM_PWM_Start(buzzer->buzz_timer , buzzer->channel_1);
	HAL_TIM_PWM_Start(buzzer->buzz_timer , buzzer->channel_2);

	/* Update interrupt timer registers to turn off the buzzer after correct delay
	 * A duration of 0 will turn the buzzer on until manually turned off with buzzer_off */
	if (duration != 0){
		__HAL_TIM_SET_AUTORELOAD(buzzer->interrupt_timer, duration); //Set ARR to the duration. Since timer period is 1ms, duration in ms can be used directly
		__HAL_TIM_SET_COUNTER(buzzer->interrupt_timer, 0);	//Reset timer count to 0
		__HAL_TIM_CLEAR_FLAG(buzzer->interrupt_timer, TIM_FLAG_UPDATE);	//Clear the interrupt flag to prevent the interrupt triggering immediately
		HAL_TIM_Base_Start_IT(buzzer->interrupt_timer);
	}

	buzzer->state = 1;
}

void buzzer_play_melody(buzzer_HandleTypeDef *buzzer, uint16_t *melody[6][2]){

	buzzer->melody_mode = 1;
	buzzer->melody_index = 0;
	//buzzer->melody_length = sizeof(melody);	//dont think the size will be correct, nneed to fix
	buzzer->melody_length = 5; //for tedting
	buzzer->melody = melody;

	//buzzer_write(&buzzer, test_melody[buzzer->melody_index][0], test_melody[buzzer->melody_index][1]);
	buzzer_write(buzzer, buzzer->melody[buzzer->melody_index][0], buzzer->melody[buzzer->melody_index][1]);

}
