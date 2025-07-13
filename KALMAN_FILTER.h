#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#include <stdio.h>
#include <math.h>
typedef struct {
    float Q_angle; // Process noise variance for the accelerometer
    float Q_bias; // Process noise variance for the gyro bias
    float R_measure; // Measurement noise variance - this is actually the variance of the measurement

    float angle; // Ouput- part of the 2x1 state vector
    float bias; // Ouput- gyro bias- part of the 2x1 state vector
    float rate; // Unbiased rate calculated from the rate and the calculated bias - you have to call getAngle to update the rate

    float P[2][2]; // Error covariance matrix
} Kalman_t;

void kalman_init(Kalman_t *k);
float kalman_getAngle(Kalman_t *k, float newAngle, float newRate, float dt);

void kalman_setAngle(Kalman_t *k, float angle);

//Tune
void kalman_setQangle(Kalman_t *k, float Q_angle);
void kalman_setQbias(Kalman_t *k, float Q_bias);
void kalman_setRmeasure(Kalman_t *k, float R_measure);
#endif

