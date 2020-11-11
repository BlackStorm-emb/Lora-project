/*
 * Buzzer.h
 *
 *  Created on: Nov 9, 2020
 *      Author: Тлехас Алий
 */

#ifndef INC_BUZZER_H_
#define INC_BUZZER_H_

#include <main.h>

#define BUZZER_VOLUME_MAX	10
#define BUZZER_VOLUME_MUTE	0
#define SYSCLK_FREQ 16000000


void buzzer(uint16_t freq, uint8_t vol);


#endif /* INC_BUZZER_H_ */
