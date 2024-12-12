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
	buzzer->frequency = 0;
	buzzer->state = 0;
	buzzer->melody_index = 0;
}

void buzzer_interrupt(buzzer_HandleTypeDef *buzzer){

	HAL_TIM_Base_Stop_IT(buzzer->interrupt_timer);
	//__HAL_TIM_SET_COUNTER(buzzer->interrupt_timer, 0); //need to reset the count of the register here to 0 TIM2->CNT = 0;
	HAL_TIM_PWM_Stop(buzzer->buzz_timer , TIM_CHANNEL_2);
	HAL_TIM_PWM_Stop(buzzer->buzz_timer , TIM_CHANNEL_3);
	buzzer->state = 0;
}

void buzzer_write(buzzer_HandleTypeDef *buzzer, uint16_t frequency, uint16_t duration){

	buzzer->frequency = frequency;
	//buzzer->buzz_end_tick = HAL_GetTick() + duration;

	//Need to figure out how to set ARR and pulse registers to get the desired frequency
	//For now, just using default testing values
	HAL_TIM_PWM_Start(buzzer->buzz_timer , TIM_CHANNEL_2);	//uncomment to hear noise
	HAL_TIM_PWM_Start(buzzer->buzz_timer , TIM_CHANNEL_3);

	__HAL_TIM_SET_AUTORELOAD(buzzer->interrupt_timer, duration); //Set ARR to the duration. Since timer period is 1ms, duration in ms can be used directly
	__HAL_TIM_SET_COUNTER(buzzer->interrupt_timer, 0);	//Reset count to 0
	__HAL_TIM_CLEAR_FLAG(buzzer->interrupt_timer, TIM_FLAG_UPDATE);	//Clear the interrupt flag. Don't understand exactly, but without this, interrupt triggers immediately
	HAL_TIM_Base_Start_IT(buzzer->interrupt_timer);
	buzzer->state = 1;
}
