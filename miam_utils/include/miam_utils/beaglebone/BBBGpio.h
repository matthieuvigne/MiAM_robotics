/// \file beaglebone/BBBGpio.h
/// \brief Functions concerning the Beaglebone GPIOs.
///
/// \details This file allows configuration and interface to the Beaglebone GPIOs.
///    \note     All functions in this header should be prefixed with gpio_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef BBBGPIO_H
#define BBBGPIO_H

    #include <stdint.h>
    #include <string>

    /// \brief Export a gpio pin, setting the direction.
    /// \details Exporting a gpio pin maks it available in user-space. In practice, this create a folder in
    ///             /sys/class/gpio/gpioX, where X is the gpio number. However, this does not necessarily give control
    ///             of the pin : this is done at pinmux level in the overlay (see Eurobot overlay). As such, if the pin
    ///             is used for something else (serial communication for example), an operation done on the gpio is meaningless.
    ///             Note also that, while exporting the gpio does set its direction, there is no way in user space to
    ///             change the pullups on the pin.
    ///             This function never fails, but one again, it is up to the user to check that the pin is configured correctly.
    ///
    /// \param[in] pin gpio pin number (kernel representation).
    /// \param[in] direction gpio direction, either "in" or "out"
    ///
    /// \return 0 on success, -1 on file access error.
    int gpio_exportPin(int const& pin, std::string const& direction);

    /// \brief Read a digital input.
    /// \details The pin must be configured as an input for the return value to make sense.
    ///
    /// \param[in] pin gpio pin number (kernel representation).
    ///
    /// \return Positive value on success : 0 for low, 1 for high.
    ///            Negative value on failure: -1: reading error (probably pin not enabled). -2: pin enabled but not an input.
    int gpio_digitalRead(int const& pin);


    /// \brief Write to a digital output.
    /// \details The pin must be configured as an input for the return value to make sense.
    ///
    /// \param[in] pin gpio pin number (kernel representation).
    /// \param[in] value value to write: 0 for low, else high
    ///
    /// \return 0 on success
    ///            Negative value on failure: -1: reading error (probably pin not enabled). -2: pin enabled but not an output.
    int gpio_digitalWrite(int const& pin, int const& value);

    /// \brief Init reader for analog port.
    ///
    /// \details To speed up reading, in_voltage files remain open: this function opens them. It should be called
    ///          before any call to gpio_analogRead. Note that BBB_enableCape calls this function.
    void gpio_initAnalogReaders();

    /// \brief Read value from analog port.
    ///
    /// \param[in] pin analog port number, from 0 to 6.
    ///
    /// \return port voltage, between 0 and 4095 on success.
    ///         Negative value on failre: -1 on invalid pin number, -2 on any other failure.
    int gpio_analogRead(int const& pin);

    /// \brief Configure PWM output.
    ///
    /// \param[in] pin Pin number, 0 or 1 to represent EHRPWM2A or B port.
    /// \param[in] period_ns Period, in nanoseconds.
    /// \param[in] duty_ns Duty cycle, i.e. time high, in ns.
    /// \return Negative value on failre: -1 on invalid pin number, -2 on any other failure.
    int gpio_setPWM(int const& port, int const& period_ns, int const& duty_ns);

    /// \brief Generate pwm signal for a servomotor.
    ///
    /// \param[in] pin Pin number, 0 or 1 to represent EHRPWM2A or B port.
    /// \param[in] servoPosition Servo pulse width, in microseconds.
    /// \return Negative value on failre: -1 on invalid pin number, -2 on any other failure.
    int gpio_servoPWM(int const& port, int const& servoPosition);
#endif
