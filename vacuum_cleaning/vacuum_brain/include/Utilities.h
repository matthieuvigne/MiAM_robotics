/// \file Utilities.h
/// \brief Utility functions for the inverted pendulum.

/// \author Matthieu Vigne
/// \copyright GNU GPLv3

#ifndef UTILITIES_H
     #define UTILITIES_H

    ///< Global includes
    #include <miam_utils/miam_utils.h>
    #include <miam_utils/raspberry_pi/RaspberryPi.h>

    int const BUTTON_GPIO = 21; ///< GPIO number of button
    double const MAX_ANGLE = 0.4; ///< Maximum pendulum angle before motor shutdown.

    class LedWrapper;

    /// \brief Send a command to both motors.
    ///
    /// \details Command is normalized between -1 and 1: the input value is clamped to fit between these numbers.
    ///
    /// \param[in, out] rightMotorCommand Command to send to the right motor. Value is updated to be between -1 and 1.
    /// \param[in, out] leftMotorCommand Command to send to the left motor. Value is updated to be between -1 and 1.
    void sendCommandToMotors(double const& rightMotorCommand, double const& leftMotorCommand);

    /// \brief Init GPIO to talk to motors.
    void initMotors();

    /// \brief Update led color based on current angle.
    /// \param[in] leds LedWrapper object.
    /// \param[in] angle Current angle.
    void updateLedColor(LedWrapper & leds, double const& angle, bool isControllerRunning);


    /// \brief Return true if button was released since last call.
    bool wasButtonReleased();
 #endif
