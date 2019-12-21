/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "MiAMEurobot/KalmanFilter.h"

void kalman_init(Kalman *k, double angle)
{
    // Default dynamics covariance.
    k->Q_angle = 0.003;
    k->Q_bias = 0.1;

    // Default sensor covariance.
    k->R_measure = 0.0005;

    // Initial angle and bias - the gyro is considered to have a 0 average offset.
    k->angle = angle;
    k->bias = 0.0;

    // Initial value of P: we trust the initial guess.
    k->P[0][0] = 0.0001f;
    k->P[0][1] = 0.0f;
    k->P[1][0] = 0.0f;
    k->P[1][1] = 0.01f;
};

double kalman_updateEstimate(Kalman *k, double angleMeasurement, double rateMesurement, double dt)
{

    // Prediction: state dynamics
    k->rate = rateMesurement - k->bias;
    double predictedAngle = k->angle + dt * k->rate;

    // Update covariance
    k->P[0][0] += dt * (dt*k->P[1][1] - k->P[0][1] - k->P[1][0] + k->Q_angle);
    k->P[0][1] -= dt * k->P[1][1];
    k->P[1][0] -= dt * k->P[1][1];
    k->P[1][1] += k->Q_bias * dt;

    // Innovation
    double innovation = angleMeasurement - predictedAngle;

    // Calculate Kalman gain
    double S = k->P[0][0] + k->R_measure;

    double K[2];
    K[0] = k->P[0][0] / S;
    K[1] = k->P[1][0] / S;

    // Update state and covariance.
    k->angle = predictedAngle + K[0] * innovation;
    k->bias = k->bias + K[1] * innovation;
    //~ printf("%f\n", k->bias);
    double P00_temp = k->P[0][0];
    double P01_temp = k->P[0][1];
    k->P[0][0] -= K[0] * P00_temp;
    k->P[0][1] -= K[0] * P01_temp;
    k->P[1][0] -= K[1] * P00_temp;
    k->P[1][1] -= K[1] * P01_temp;

    return k->angle;
}
