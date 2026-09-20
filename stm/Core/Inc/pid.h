/*
 * pid.h
 *
 *  Created on: Jun 11, 2025
 *      Author: martinhaugen
 */

#ifndef INC_PID_H_
#define INC_PID_H_

typedef struct {
	float kp;
	float ki;
	float kd;

	float prev_error;
	float integral;

	float output_limit;		// max magnitude of output
	float integral_limit;	// max magnitude of integral term

	float dt;				// time step in seconds
} PIDcontroller;

void PID_Init(PIDcontroller *pid, float kp, float ki, float kd, float dt);
float PID_Compute(PIDcontroller *pid, float setpoint, float measurement);

#endif /* INC_PID_H_ */
