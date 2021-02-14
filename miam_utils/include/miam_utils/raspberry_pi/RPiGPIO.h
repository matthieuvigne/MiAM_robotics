/// \file raspberry_pi/RPiGPIO.h
/// \brief Access to the Raspberry Pi 3 GPIOs.
///
/// \details GPIO access is done through direct register access, for efficiency. Because the processor changed, this code
///          will not work as-is for older versions of the raspberry. See BCM2837 Peripherals Manual, p.89 onward,
///          for more informations.
///          A quick speed test shows that this code can read and write to this GPIO in under 175ns (2.9MHz frequency).
///          In order to prevent messing up with serial port configuration, only GPIOs 4 to 26 are available.
///          Note that GPIOs 7-11 are used for SPI, 12-13 for PWM and 14-15 for UART:
///          setting them as GPIOs would disable the corresponding port.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef RPI_GPIO
#define RPI_GPIO

    typedef enum
    {
      PI_GPIO_OUTPUT               = 0x00,  ///<  Output.
      PI_GPIO_INPUT_PULLUP         = 0x01,   ///<  Input, pullup.
      PI_GPIO_INPUT_PULLDOWN       = 0x02,   ///<  Input, pulldown.
      PI_GPIO_INPUT_NOPULL         = 0x03,   ///<  Input, no pull up/down resistor.
    }
    PiGPIOMode;

    typedef enum
    {
      F4_6875kHz = 1,
      F9_375kHz = 2048,
      F18_75kHz = 1024,
      F37_5kHz = 512,
      F75kHz = 256,
      F150kHz = 128,
      F300kHz = 64,
      F600kHz = 32,
      F1200kHz = 16,
      F2400kHz = 8,
      F4800kHz = 4,
      F9600kHz = 2
    }PiPWMClockFrequency;

    /// Defines for pin direction.
    #define LOW false
    #define HIGH true

    /// \brief Access the GPIO registers.
    ///
    /// \details This function must be called once before any other GPIO function is called.
    ///          Failing to do so would cause the code to crash with a segmentation fault (trying to access undefined memory).
    ///
    /// \return TRUE if memory access was successful, false otherwise.
    bool RPi_enableGPIO();

    /// \brief Setup a GPIO pin as input or output.
    ///
    /// \details If gpioPin does not correspond to one of the Raspberry Pi exposed GPIO, this function has no effect.
    /// \param[in] gpioPin A valid GPIO pin number.
    /// \param[in] direction The mode of the GPIO (input, output, with pull resistor settings.
    void RPi_setupGPIO(unsigned int const& gpioPin, PiGPIOMode const& direction);

    /// \brief Write to a GPIO output.
    ///
    /// \details If gpioPin does not correspond to one of the Raspberry Pi exposed GPIO, this function has no effect.
    ///          If the pin is not an output, this function has no effect.
    /// \param[in] gpioPin A valid GPIO pin number.
    /// \param[in] value FALSE (0) for low, TRUE for high.
    void RPi_writeGPIO(unsigned int const& gpioPin, bool const& value);

    /// \brief Read current GPIO status.
    ///
    /// \details If gpioPin does not correspond to one of the Raspberry Pi exposed GPIO, this function has no effect.
    /// \param[in] gpioPin A valid GPIO pin number.
    /// \return The current pin status.
    bool RPi_readGPIO(unsigned int const& gpioPin);

    /// \brief Configure PWM output.
    ///
    /// \details channel 0 stands for GPIO 12, channel 1 for GPIO 13.
    ///          GPIO must have been enabled in user space, see https://librpip.frasersdev.net/peripheral-config/pwm0and1/.
    ///
    /// \param[in] channel Channel number, 0 or 1.
    /// \param[in] period_ns Period, in nanoseconds.
    /// \param[in] duty_ns Duty cycle, i.e. time high, in ns.
    /// \return Negative value on failre: -1 on invalid pin number, -2 on any other failure.
    // int RPi_setPWM(int const& channel, int const& period_ns, int const& duty_ns);

    /// \brief Set PWM clock frequency.
    /// \param[in] frequency Clock frequency to use.
    void RPi_setPWMClock(PiPWMClockFrequency const& frequency);

    /// \brief Enable PWM output.
    ///
    /// \details channel 0 stands for GPIO 12, channel 1 for GPIO 13.
    ///
    /// \param[in] enableFirstChannel Enable / disable first channel (GPIO 12).
    /// \param[in] enableSecondChannel Enable / disable second channel (GPIO 13).
    void RPi_enablePWM(bool const& enableFirstChannel, bool const& enableSecondChannel);

    /// \brief Set PWM output.
    ///
    /// \param[in] channel Channel number, 0 (GPIO 12) or 1 (GPIO 13).
    /// \param[in] dutyCycle Duty cycle, from 0 to periodCS.
    /// \param[in] periodCS Period of the signal, in number of clock samples. Use clock frequency to compute corresponding signal frequency.
    void RPi_setPWM(int const& channel, int const& dutyCycle, int const& periodCS);

#endif
