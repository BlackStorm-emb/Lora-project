/*
 * Buzzer.c
 *
 *  Created on: Nov 9, 2020
 *      Author: Тлехас Алий
 */
#include <Buzzer.h>


void buzzer(uint16_t freq, uint8_t vol) {
	TIM2->PSC = (SYSCLK_FREQ  / (2 * BUZZER_VOLUME_MAX * freq)) - 1;
	if(vol > BUZZER_VOLUME_MAX) vol = BUZZER_VOLUME_MAX;
	TIM2->CCR1 = vol;
}

