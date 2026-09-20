/*
 * kalman_filter.h
 *
 *  Created on: Jun 10, 2025
 *      Author: martinhaugen
 */

#ifndef INC_KALMAN_FILTER_H_
#define INC_KALMAN_FILTER_H_

typedef struct {
	float angle;		// filtered angle
	float bias;			// gyro bias
	float rate;			// unbiased rate

	float P[2][2];		// error covariance matrix

	float Q_angle;		// process noise variance for angle
	float Q_bias;		// process noise variance for gyro bias
	float R_measure;	// measurement noise variance
} Kalman_t;

void Kalman_Init(Kalman_t *kf);
float Kalman_GetAngle(Kalman_t *kf, float newAngle, float newRate, float dt);

#endif /* INC_KALMAN_FILTER_H_ */
