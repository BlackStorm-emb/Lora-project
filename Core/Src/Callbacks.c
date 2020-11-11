/*
 * Callbacks.c
 *
 *  Created on: Nov 9, 2020
 *      Author: Тлехас Алий
 */

#include "Callbacks.h"
volatile Button_state buttons;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if(htim->Instance == TIM16)
	{
		HAL_TIM_Base_Stop_IT(&htim16);
		__HAL_GPIO_EXTI_CLEAR_IT(but_1_Pin);
		__HAL_GPIO_EXTI_CLEAR_IT(but_2_Pin);

		if (HAL_GPIO_ReadPin(GPIOC, but_1_Pin) && HAL_GPIO_ReadPin(GPIOC, but_2_Pin))
				buttons = BOTH_Pressed;
		else if (HAL_GPIO_ReadPin(GPIOC, but_1_Pin) && !HAL_GPIO_ReadPin(GPIOC, but_2_Pin))
				buttons = LEFT_Pressed;
		else if (!HAL_GPIO_ReadPin(GPIOC, but_1_Pin) && HAL_GPIO_ReadPin(GPIOC, but_2_Pin))
				buttons = RIGHT_Pressed;

		NVIC_ClearPendingIRQ(EXTI15_10_IRQn);
		HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	}
}
