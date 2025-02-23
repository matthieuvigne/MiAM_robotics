#ifndef RAIL_SERVO_H
#define RAIL_SERVO_H

#include <miam_utils/drivers/STSServoDriver.h>

/// \brief Control for a continuous rotation servo mounted on a vertical rail
class RailServo
{
    public:

        /// @brief Constructor
        /// @param driver Driver to use
        /// @param servoId Id of the servo
        /// @param gpioId Id of the limit switch GPIO
        /// @param distance Total travel distance
        /// @param inverted Invert servo rotation
        RailServo(STSServoDriver *driver, int const& servoId, int const& gpioId, int const& distance, bool inverted=false, bool calibrateBottom=false);

        void startCalibration();

        bool isCalibrated();

        void move(double const& targetPosition);

        bool isMoving();

        double getCurrentPosition();


    private:
        void calibration();

        STSServoDriver *driver_;
        int servoId_;
        int gpio_;
        int travelDistance_;
        int sign_;
        bool calibrateBottom_;

        bool isCalibrated_ = false;
        double currentPosition_ = 0.0;

};

#endif
