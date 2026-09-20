/*
 * kalman_filter.c
 *
 *  Created on: Jun 10, 2025
 *      Author: martinhaugen
 */

#include "kalman_filter.h"

void Kalman_Init(Kalman_t *kf){
	kf->angle = 0.0f;
	kf->bias = 0.0f;
	kf->P[0][0] = 0.0f;
	kf->P[0][1] = 0.0f;
	kf->P[1][0] = 0.0f;
	kf->P[1][1] = 0.0f;

	kf->Q_angle = 0.01f;
	kf->Q_bias = 0.003f;
	kf->R_measure = 0.01f;
}

float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt){
	// prediction step
	kf->rate = newRate - kf->bias;
	kf->angle += dt * kf->rate;

	kf->P[0][0] += dt * (dt * kf->P[1][1] - kf->P[0][1] - kf->P[1][0] + kf->Q_angle);
	kf->P[0][1] -= dt * kf->P[1][1];
	kf->P[1][0] -= dt * kf->P[1][1];
	kf->P[1][1] += kf->Q_bias * dt;


	// update step
	float S = kf->P[0][0] + kf->R_measure;
	float K0 = kf->P[0][0] / S;
	float K1 = kf->P[1][0] / S;

	float y = newAngle - kf->angle;

	kf->angle += K0 * y;
	kf->bias += K1 * y;

	float P00_temp = kf->P[0][0];
	float P01_temp = kf->P[0][1];

	kf->P[0][0] -= K0 * P00_temp;
	kf->P[0][1] -= K0 * P01_temp;
	kf->P[1][0] -= K1 * P00_temp;
	kf->P[1][1] -= K1 * P01_temp;

	return kf->angle;
}



