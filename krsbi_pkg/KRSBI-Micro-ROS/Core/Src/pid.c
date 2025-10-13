/*
 * pid.c
 *
 *  Created on: Jul 20, 2025
 *      Author: musthofa
 */


#include "pid.h"
#include <math.h>
float error = 0, derivative;

void PID_Init(PID_Controller *pid, float kp, float ki, float kd,
		float min_output, float max_output) {
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->setpoint = 0.0;
	pid->integral = 0.0;
	pid->previous_error = 0.0;
	pid->output = 0.0;
	pid->max_output = max_output;
	pid->min_output = min_output;
	pid->error = 0.0;
}

float PID_Update(PID_Controller *pid, float current_value, float error) {
	pid->error = error;
	pid->integral += error;
	derivative = (pid->error - pid->previous_error);

	pid->output = (pid->kp * pid->error) + (pid->ki * pid->integral)
			+ (pid->kd * derivative);

	if (pid->integral > pid->max_output) {
		pid->integral = pid->max_output;
	} else if (pid->output < pid->min_output) {
		pid->integral = pid->min_output;
	}

	if (pid->output > pid->max_output) {
		pid->output = pid->max_output;
	} else if (pid->output < pid->min_output) {
		pid->output = pid->min_output;
	}

	pid->previous_error = pid->error;
	if (error == 0)
		pid->output = 0;
	return pid->output;
}
