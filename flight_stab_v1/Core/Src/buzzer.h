/*
 * buzzer.h
 *
 *  Created on: Dec 2, 2020
 *      Author: d
 */

#ifndef SRC_BUZZER_H_
#define SRC_BUZZER_H_

#include <stdint.h>
#include "main.h"

#define BUZZER_QUEUE_SIZE 15

#define BUZZER_ON_ACTION HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_SET)
#define BUZZER_OFF_ACTION HAL_GPIO_WritePin(BEEPER_GPIO_Port, BEEPER_Pin, GPIO_PIN_RESET)

typedef enum {
	BUZZER_OFF,
	BUZZER_ON
}buzzer_state_type;

char buzzer_enqueue(buzzer_state_type state, uint16_t duration);
void buzzer_next();
void buzzer_wait();


#endif /* SRC_BUZZER_H_ */
