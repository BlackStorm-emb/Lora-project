/*
 * Button.c
 *
 *  Created on: Oct 27, 2020
 *      Author: Тлехас Алий
 */


#include <Button.h>

uint8_t ReadButtons() {
	return HAL_GPIO_ReadPin(but_2_GPIO_Port, but_2_Pin) << 1 | HAL_GPIO_ReadPin(but_1_GPIO_Port, but_1_Pin);
}
