/*
 * Button.c
 *
 *  Created on: Oct 27, 2020
 *      Author: Тлехас Алий
 */


#include <Button.h>

extern volatile Button_state buttons;

void delay(uint16_t ticks) {
	for (uint16_t i = 0; i < ticks; i++) {
		asm("NOP");
	}
}

void HAL_GPIO_EXTI_customIRQHandler() {
	delay(100);

	HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	HAL_TIM_Base_Start_IT(&htim16);
}

uint8_t ReadButtons() {
	if (buttons != NONE_Pressed) {
		Button_state buf = buttons;
		buttons = NONE_Pressed;
		return buf;
	}
	return NONE_Pressed;
}
