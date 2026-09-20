/*
 * pid.c
 *
 *  Created on: Jun 11, 2025
 *      Author: martinhaugen
 */

#include "pid.h"

void PID_Init(PIDcontroller *pid, float kp, float ki, float kd, float dt){
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->dt = dt;

	pid->prev_error = 0.0f;
	pid->integral = 0.0f;

	pid->output_limit = 1000.0f;
	pid->integral_limit = 500.0f;
}

float PID_Compute(PIDcontroller *pid, float setpoint, float measurement){
	float error = setpoint - measurement;

	// proportional
	float Pout = pid->kp * error;

	// integral with anti-windup
	pid->integral += error * pid->dt;
	if (pid->integral > pid->integral_limit) pid->integral = pid->integral_limit;
	if (pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
	float Iout = pid->ki * pid->integral;

	// derivative
	float derivative = (error - pid->prev_error) / pid->dt;
	float Dout = pid->kd * derivative;

	// combine terms
	float output = Pout + Iout + Dout;

	// saturate output
	if (output > pid->output_limit) output = pid->output_limit;
	if (output < -pid->output_limit) output = -pid->output_limit;

	// store current error
	pid->prev_error = error;

	return output;
}
