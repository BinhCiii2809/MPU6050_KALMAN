#include "KALMAN_FILTER.h"

void kalman_init(Kalman_t *k) {
    k->Q_angle = 0.01f;
    k->Q_bias = 0.003f;
    k->R_measure = 0.04f;

    k->angle = 0.0f;
    k->bias = 0.0f;
    k->rate = 0.0f;

    k->P[0][0] = 0.0f;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 0.0f;
}

float kalman_getAngle(Kalman_t *k, float newAngle, float newRate, float dt) {
  
    // Prediction
    k->rate = newRate - k->bias;
    k->angle += dt * k->rate;
  
    // Update estimation error covariance
    k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    // Kalman gain
    float Estimate_error = k->P[0][0] + k->R_measure;
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = k->P[0][0] / Estimate_error;
    K[1] = k->P[1][0] / Estimate_error;

    // Calculate angle and bias - Update estimate with measurement newAngle
    // Update estimate
    float y = newAngle - k->angle;
    k->angle += K[0] * y;
    k->bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    // Update covariance matrix
    float P00_covariance = k->P[0][0];
    float P01_covariance = k->P[0][1];

    k->P[0][0] -= K[0] * P00_covariance;
    k->P[0][1] -= K[0] * P01_covariance;
    k->P[1][0] -= K[1] * P00_covariance;
    k->P[1][1] -= K[1] * P01_covariance;

    return k->angle;
}

//Set as the starting angle
void kalman_setAngle(Kalman_t *k, float angle) { k->angle = angle; }

//Tune
void kalman_setQangle(Kalman_t *k, float Q_angle) { k->Q_angle = Q_angle; }
void kalman_setQbias(Kalman_t *k, float Q_bias) { k->Q_bias = Q_bias; }
void kalman_setRmeasure(Kalman_t *k, float R_measure) { k->R_measure = R_measure; }
