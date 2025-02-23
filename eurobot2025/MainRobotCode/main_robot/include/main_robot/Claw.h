#ifndef CLAW_H
#define CLAW_H

#include <miam_utils/drivers/STSServoDriver.h>
#include "RailServo.h"


/// \brief Control for a continuous rotation servo mounted on a vertical rail
class Claw
{
    public:

        /// @brief Constructor
        /// @param driver Driver to use
        /// @param servoId Id of the servo
        /// @param gpioId Id of the limit switch GPIO
        /// @param distance Total travel distance
        /// @param inverted Invert servo rotation
        Claw(STSServoDriver *driver,
             RailServo rail,
             int const& wristServoId,
             int const& clawServoId,
             int const& clawCloseValue,
             bool mirror=false);


        void openClaw();
        void closeClaw();

        void fold();
        void unfold();

        RailServo rail_;

    private:
        STSServoDriver *driver_;

        int wristServoId_;
        int clawServoId_;
        int clawCloseValue_;

        int sign_;
};

#endif
