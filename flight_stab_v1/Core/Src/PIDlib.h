/*
 * PIDlib.h
 *
 *  Created on: Sep 22, 2020
 *      Author: d
 */

#ifndef PIDLIB_H_
#define PIDLIB_H_


class PID {
	public:
		PID();
		float update(float error);
		float getPidKd() const;
		void setPidKd(float pidKd);
		float getPidKi() const;
		void setPidKi(float pidKi);
		float getPidKp() const;
		void setPidKp(float pidKp);
		void setPidAll(const float Kp, const float Ki, const float Kd, const float windup_max);
		float getPidWindupMax() const;
		void setPidWindupMax(float pidWindupMax);

	private:
		float pid_Kp, pid_Ki, pid_Kd;
		float pid_windup_max;
		float pid_error_old;
		float pid_integral;
};


#endif /* PIDLIB_H_ */
