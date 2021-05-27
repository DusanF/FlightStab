/*
 * PIDlib.cpp
 *
 *  Created on: Sep 22, 2020
 *      Author: d
 */

#include "PIDlib.h"

PID::PID(){	//constructor
	pid_Kp = 0;
	pid_Ki = 0;
	pid_Kd = 0;
	pid_windup_max = 0;
	pid_error_old = 0;
	pid_integral = 0;
}

float PID::update(float error){	//PID loop
	float out;

	out = pid_Kp * error;	//proportional

	pid_integral += error
	if(pid_integral > pid_windup_max)	//anti windup
		pid_integral = pid_windup_max;
	if(pid_integral < -pid_windup_max)
		pid_integral = -pid_windup_max;
	out += pid_Ki * pid_integral;;	//integral

	out += pid_Kd * (error - pid_error_old);	//derivative
	pid_error_old = error;

	return out;
}

float PID::getPidKd() const {
	return pid_Kd;
}

void PID::setPidKd(float pidKd) {
	pid_Kd = pidKd;
}

float PID::getPidKi() const {
	return pid_Ki;
}

void PID::setPidKi(float pidKi) {
	pid_Ki = pidKi;
}

float PID::getPidKp() const {
	return pid_Kp;
}

void PID::setPidKp(float pidKp) {
	pid_Kp = pidKp;
}

float PID::getPidWindupMax() const {
	return pid_windup_max;
}

void PID::setPidWindupMax(float pidWindupMax) {
	pid_windup_max = pidWindupMax;
}

void PID::setPidAll(const float Kp, const float Ki, const float Kd, const float windup_max){	//set all PID parameters
	pid_Kp = Kp;
	pid_Ki = Ki;
	pid_Kd = Kd;
	pid_windup_max = windup_max;
}
