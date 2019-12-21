/// \file drivers/dualL6470Driver.h
/// \brief Driver for two L6470 stepper motor drivers daisy-chained on a single SPI port.
///
/// \details Several L6470 drivers can be daisy chained together - this file implements communication
///          with exactly two L6470 (as is the case for instance for the X-NUCLEO-IHM02A1).
///          The API is targetted toward driving a robot chassis, thus constraining some symmetry
///          between both drivers. All functions are thread-safe.
///    \note     All functions in this header should be prefixed with dualL6470_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef dualL6470_DRIVER
#define dualL6470_DRIVER

    #include <vector>
    #include <mutex>

    namespace miam{

        class L6470{

            public:
                /// \brief Default constructor.
                L6470();

                /// \brief Constructor.
                /// \details This function only builds the object, but does not perform any operation on the
                ///          SPI port.
                /// \param[in] portName Name of the port, in the file system (i.e. a string "/dev/spidevx").
                /// \param[in] numberOfSlaves Number of devices to drive.
                /// \param[in] speed Bus clock frequency. Default: 4Mhz.
                L6470(std::string const& portName, int const& numberOfDevices, int const& busFrequency = 4000000);

                /// \brief Assignment operator.
                L6470& operator=(L6470 const& l);

                /// \brief Try to init all devices.
                ///
                /// \details This function tries, for each device, to reset it, then set its speed and velocity
                ///          profile, and its back-emf force. It then asks the device for a specific value, to
                ///          check the connection.
                ///
                /// \param[in] maxSpeed Maximum motor speed, in steps/s (from 15.25 to 15610, resolution 15.25 step/s).
                /// \param[in] maxAcceleration Maximum motor acceleration and deceleration, in steps/s^2 (from 14.55 to 59590,
                ///                            resolution 14.55 step/s^2).
                /// \param[in] k_hld Value of the SPIN_KVAL_HOLD register.
                /// \param[in] k_mv Value of the SPIN_KVAL_ACC, SPIN_KVAL_DEC and SPIN_KVAL_RUN registers.
                /// \param[in] int_spd Value of the SPIN_INT_SPD register.
                /// \param[in] st_slp Value of the SPIN_ST_SLP register.
                /// \param[in] slp_acc Value of the SPIN_FN_SLP_ACC and SPIN_FN_SLP_DEC registers.
                /// \param[in] hasCrystal True to use external 16MHz crystal as clock. Otherwise, the internal oscillator
                ///                       is used. Note that this oscillator can be wrong by around 5%.
                /// \return true is all devices responded, false otherwise.
                bool init(uint32_t const& maxSpeed, const uint32_t& maxAcceleration, uint32_t const& k_hld, uint32_t const& k_mv,
                          uint32_t  const& int_spd, uint32_t const& st_slp, uint32_t const& slp_acc,
                          bool hasCrystal = true);

                /// \brief Set a parameter register.
                ///
                ///
                /// \param[in] param The parameter to set.
                /// \param[out] paramValues Values of the parameter for each device.
                void setParam(uint8_t const& param, std::vector<uint32_t> const& parameterValues);

                /// \brief Set a parameter register to the same value for each device.
                ///
                ///
                /// \param[in] param The parameter to set.
                /// \param[out] paramValue The value for all devices.
                void setParam(uint8_t const& param, uint32_t const& parameterValue);

                /// \brief Read a parameter register.
                ///
                ///
                /// \param[in] param The parameter to set.
                /// \return Vector of parameter value from each device.
                std::vector<uint32_t> getParam(uint8_t const& param);

                /// \brief Perform a soft motor stop on both motors.
                /// \details A soft stop means that the motor will decelerate at given deceleration until it stops (contrary to a
                ///             hard stop where the motor stops instantaneously). Only soft stop is exposed to prevent damage to the
                ///             motor.
                void softStop();

                /// \brief Perform a hard motor stop on both motors: motor will try to stop instantaneously.
                void hardStop();

                /// \brief Place both motors in a high impedance state, i.e the motor shaft will now spin freely.
                void highZ();

                /// \brief Get current position of both motors.
                ///
                /// \return Vector of position of the each motor, in steps.
                std::vector<int32_t> getPosition();

                /// \brief Get current motor velocity (unsigned, i.e. always positive).
                ///
                /// \return Vector of velocity of the each motor, in steps/s.
                std::vector<double> getSpeed();

                /// \brief Set current motor speed.
                ///
                /// \param[in] motorSpeeds Vector of motors speeds, in steps/s (between -15625 and 15625).
                ///            Value will be clamped internally by the maximum  motor speed.
                ///            If given vector is shorter than the number of devices, zeros are appended for the last
                ///            devices.
                void setSpeed(std::vector<double> const& motorSpeeds);

                /// \brief Set motor max speed and acceleration. The same value is used for all motors.
                ///
                /// \param[in] maxSpeed Maximum motor speed, in steps/s (from 15.25 to 15610, resolution 15.25 step/s).
                /// \param[in] acc Target motor acceleration, in steps/s^2 (from 14.55 to 59590, resolution 14.55 step/s^2).
                /// \param[in] dec Target motor deceleration, in steps/s^2 (from 14.55 to 59590, resolution 14.55 step/s^2).
                void setVelocityProfile(double const& maxSpeed, double const& accel, double const& decel);

                /// \brief Get the last error from all devices.
                /// \details If an error is present, it will be printed in the terminal. Note that this function clears
                ///          the status error flags.
                ///
                /// \return Vector of errors from devices.
                std::vector<uint32_t> getError();

                /// \brief Check if at least one of the motor is running.
                /// \details Note that this function clears the status error flags.
                ///
                /// \returns true if at least one motor is moving.
                bool isBusy();

                /// \brief Move each motor a given number of steps.
                ///
                /// \param[in] nSteps The number of steps to move each motor (negative means backward motion).
                ///                   If the vector is too short, the remaining motors do not move.
                void moveNSteps(std::vector<int32_t> nSteps);


            private:

                /// \brief Send and receives an array of data over spi.
                /// \param[in] data Data to send - the buffer is overwritten with new data.
                /// \param[in] len Length of the input buffer.
                /// \return <0 on error.
                int spiReadWrite(uint8_t* data, uint8_t const& len);

                /// \brief Send a command to the devices, and read corresponding response.
                ///    \details If the length of one of the two input vectors is not numberOfDevices_, this function
                ///          returns immediately.
                ///
                /// \param[in] command Command to send.
                /// \param[in] parameters Parameters for each device.
                /// \param[in] paramLength Length of parameters, in bytes. This length should be the max of the length
                ///                        of all parameters.
                /// \return Vector of response from the devices. Possibly full of zeros if no response is aquired.
                std::vector<uint32_t> sendCommand(std::vector<uint8_t> const& commands, std::vector<uint32_t> const& parameters, uint8_t paramLength);


                /// \brief Send a command without parameters to all devices.
                /// \details Command without parameters don't return a value.
                ///
                /// \param[in] command Command to send.
                void sendCommand(uint8_t const& command);

                /// \brief Get the device status, also clearing the corresponding flags.
                ///
                /// \return Vector of errors from devices.
                std::vector<uint32_t> getStatus();

                std::string portName_;
                uint numberOfDevices_;
                int frequency_;
                std::recursive_mutex mutex_;    ///< Mutex, for thread safety. recursive_mutex that can be locked several
                                                /// times by the same thread: is this to prevent deadlock when sending a kill signal to the code.
        };
    }
#endif
