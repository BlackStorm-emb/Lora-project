/*
 * Button.h
 *
 *  Created on: Oct 27, 2020
 *      Author: Тлехас Алий
 */

#ifndef INC_BUTTON_H_
#define INC_BUTTON_H_
#include <main.h>

typedef enum {
	NONE_Pressed,
	LEFT_Pressed,
	RIGHT_Pressed,
	BOTH_Pressed,
} Button_state;

Button_state ReadButtons();

void HAL_GPIO_EXTI_customIRQHandler();
//void HAL_GPIO_EXTI_customIRQHandler();

#endif /* INC_BUTTON_H_ */
