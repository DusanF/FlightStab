/*
 * buzzer.c
 *
 *  Created on: Dec 2, 2020
 *      Author: d
 */

#include "buzzer.h"

typedef struct buzzer_queue_type buzzer_queue_type;	//one item in queue
struct buzzer_queue_type{
	uint16_t duration;	//duration in ms
	buzzer_state_type state;	//buzzer on/off
};


volatile buzzer_queue_type queue[BUZZER_QUEUE_SIZE] = {0};	//buzzer queue, ring buffer
volatile uint8_t queue_rpos = 0;	//read and write position in queue
volatile uint8_t queue_wpos = 0;
volatile uint8_t queue_len = 0;		//queue length
volatile uint8_t running = 0;

extern TIM_HandleTypeDef htim6;

char buzzer_enqueue(buzzer_state_type state, uint16_t duration){	//add one item to buzzer queue
	if(queue_len < BUZZER_QUEUE_SIZE){
		queue[queue_wpos].duration = duration;
		queue[queue_wpos].state = state;
		queue_wpos++;	//incerase size
		queue_len++;
		queue_wpos %= BUZZER_QUEUE_SIZE;
		if(running == 0){
			running = 1;
			buzzer_next();
		}
		return 1;
	} else		//queue full, reject request
		return 0;
}

void buzzer_next(){	//process next item from queue
	if(queue_len == 0){	//queue empty, stop
		running = 0;
		return;
	}

	if(queue[queue_rpos].state == BUZZER_ON){	//do requested action
		BUZZER_ON_ACTION;
	}else{
		BUZZER_OFF_ACTION;
	}

	if(queue[queue_rpos].duration < 2)		//minimal time
		queue[queue_rpos].duration = 2;
	htim6.Instance->SR = 0;	//start timer with requested time-out
	htim6.Instance->ARR = queue[queue_rpos].duration-1;
	HAL_TIM_Base_Start_IT(&htim6);

	queue_rpos++;	//move to next item
	queue_rpos %= BUZZER_QUEUE_SIZE;
	queue_len--;
}

void buzzer_wait(){	//wait for empty queue
	while(running)
		HAL_Delay(1);
	return;
}

