#ifndef CLAW_H
#define CLAW_H

#include <miam_utils/drivers/STSServoDriver.h>
#include "RailServo.h"

enum ClawPosition
{
    FORWARD,
    SIDE,
    FOLDED
};

/// \brief Control for a continuous rotation servo mounted on a vertical rail
class Claw
{
    public:

        /// @brief Constructor
        /// @param servoId Id of the servo
        /// @param gpioId Id of the limit switch GPIO
        /// @param distance Total travel distance
        /// @param inverted Invert servo rotation
        Claw(RailServo rail,
             int const& wristServoId,
             int const& clawServoId,
             int const& clawOpenValue,
             bool mirror=false);

        void init(STSServoDriver *driver);

        void openClaw();
        void closeClaw();
        void foldClaw();
        bool isClawFull(int &error);

        void move(ClawPosition const& clawPos);

        RailServo rail_;

    private:
        STSServoDriver *driver_;

        int wristServoId_;
        int clawServoId_;
        int clawOpenValue_;

        int defaultCloseTarget_;
        int lastCloseTarget_;

        int sign_;

        int mirror(int const& pos);

};

#endif
