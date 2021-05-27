/*
 * servo.h
 *
 *  Created on: Sep 23, 2020
 *      Author: d
 */

#ifndef SRC_SERVO_H_
#define SRC_SERVO_H_

#include <stdint.h>

class Servo {
public:
	Servo(uint16_t min, uint16_t max, uint16_t center, volatile uint32_t *pwm_out);
	void set_us(uint16_t servo);
	void set_norm(float servo);
private:
	int16_t min_rel, max_rel, center;
	volatile uint32_t *pwm_out;
};

#endif /* SRC_SERVO_H_ */
