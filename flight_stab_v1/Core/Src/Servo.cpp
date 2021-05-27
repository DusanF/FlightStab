/*
 * Servo.cpp
 *
 *  Created on: Sep 23, 2020
 *      Author: d
 */

#include "servo.h"

Servo::Servo(uint16_t min, uint16_t max, uint16_t center, volatile uint32_t *pwm_out){	//servo output - constructor
	min_rel = min - center;
	max_rel = max - center;
	this->center = center;
	this->pwm_out = pwm_out;	//timer register for PWM duty cycle

	*pwm_out = center;	//center servo at start
}

void Servo::set_us(uint16_t servo){
	*pwm_out = servo;	//set raw PWM
}

void Servo::set_norm(float servo){	//normalized output
	if(servo>1)	//clamp value
		servo = 1;
	else if(servo<-1)
		servo = -1;

	if (servo > 0) {
		*pwm_out = center + (max_rel * servo);
	} else {
		*pwm_out = center - (min_rel * servo);
	}
}

