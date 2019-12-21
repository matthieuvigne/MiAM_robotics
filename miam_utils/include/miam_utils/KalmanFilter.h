/// \file KalmanFilter.h
/// \brief This file implements a linear Kalman filter for robot positioning.
///
/// \details To find the current angular position of the robot, a linear kalman filter
///             is used. This filter uses data from the encoders and the gyroscope to
///             give an estimate of the current robot angle and the gyro bias.
/// \note     What is actually implemented here is not exactly a Kalman filter. This
///             solution is from Kristian Lauszu (http://www.tkjelectronics.com).
///             The state is (theta, theta_b), where theta_b is the gyro bias.
///             System dynamics is: dot_theta = theta_gyro, dot_theta_b = 0, and measurments are theta_odom.
///             Thus, gyro measures appear in the system dynamics: this should not be the case for a Kalman filter.
///             A more classical approach would be to set the state to (theta, dot_theta, theta_b), with
///             ddot_theta = dot_theta_b = 0 as a dynamics. Even better results might be achieved if we add
///             motor input as a control term for ddot_theta. Measurments would then be (theta_odom, theta_gyro).
///             My guess is that, to keep things simple, Kristian uses gyro measures in order to avoid expressin the
///             dynamics of the system (that would be by the way system dependant, which is not the case here). His
///             approach also means no matrix inversion (S is of dimension 1).
///             If we have the time, it might be worth testing a more classical approach and compare both results.
///    \note     All functions in this header should be prefixed with kalman_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef KALMAN_H
    #define KALMAN_H

    /// Structure to store the covariance matrices and the results of the filter.
    typedef struct{
    /* Kalman filter variables */
    double Q_angle; ///< Process noise covariance for angle integration.
    double Q_bias;     ///< Process noise covariance for gyro bias evolution.
    double R_measure; ///< Measurment noise covariance.

    double angle; ///< Current angle estimate of the filter.
    double bias;  ///< Current bias estimate of the filter.
    double rate;  ///< Current angular speed estimate of the filter.

    double P[2][2]; ///< State covariance matrix.
    }Kalman;

    /// \brief Initialises the Kalman filter.
    /// \details This functions sets all covariances to default, hard-coded values
    ///             and sets the initial angle estimate.
    ///
    /// \param[out] k A Kalman structure to fill.
    /// \param[in] angle The initial position of the robot.
    void kalman_init(Kalman *k, double angle);


    /// \brief Perform one iteration of the filter.
    /// \details This computes the next angle estimate based on the new measures, and the time elapsed.
    ///             The time is put there because the time between two calls to the timeout function might
    ///             not always be the same.
    /// \note     The unit of angleMeasurement and rateMesurement is arbitrary (deg or rad), but must be the
    ///             same throughout the code. As it is much more convenient to use with trigonometric functions,
    ///             we only work with radians.
    ///
    /// \param[in, out] k A Kalman structure to use.
    /// \param[in] angleMeasurement The new angle, as measured by the encoders, in rad.
    /// \param[in] rateMesurement The angular rate, as measured by the gyro, in rad/s.
    /// \param[in] dt The time elapsed since the last estimation.
    /// \returns The new angle estimate - this is also available through the angle member of the Kalman structure.
    double kalman_updateEstimate(Kalman *k, double angleMeasurement, double rateMesurement, double dt);



#endif
