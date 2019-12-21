/// \file drivers/ADNS9800Driver.h
/// \brief Driver for the ADNS9800 mouse sensor.
///
/// \details This file implements all the functions to talk to the stepper motor controller.
///          It is adapted from Sparkfun's driver for Arduino. All functions are thread-safe.
///    \note     All functions in this header should be prefixed with ANDS9800_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef ADNS9800_DRIVER
#define ADNS9800_DRIVER

    #include <stdint.h>
    #include <string>


    /// L6470 structure.
    typedef struct
    {
        std::string portName;    ///< String representing the name (in the file system) of the SPI port. This must be set by hand before using the structure.
        int port;            ///< File descriptor of the SPI port.
        int frequency;        ///< SPI clock frequency, in Hz. This is set by default to 800kHz (100000byte/s), but can be changed if needed.
        double resolution;    ///< Sensor resolution, in mm/counts.
    }ADNS9800;

    /// \brief Initialize the ANDS9800.
    /// \details This function checks that the sensor is present, configure its registers, and write the firmware.
    ///
    /// \param[inout] a An ANDS9800 structure to use to talk to the sensor.
    /// \param[in] portName Name of the port, in the file system (i.e. a string "/dev/spidevx").
    /// \return true on success, false on failure.
    bool ANDS9800_init(ADNS9800 *a, std::string const& portName);

    /// \brief Get the motion of the mouse since the last call, in mouse counts.
    /// \param[in] a An ANDS9800 structure to use to talk to the sensor.
    /// \param[out] deltaX Position increment on X axis since last call, in mouse counts.
    /// \param[out] deltaY Position increment on Y axis since last call, in mouse counts.
    void ADNS9800_getMotionCounts(ADNS9800 a, int *deltaX, int *deltaY);

    /// \brief Get the motion of the mouse since the last call, in mm.
    /// \param[in] a An ANDS9800 structure to use to talk to the sensor.
    /// \param[out] deltaX Position increment on X axis since last call, in mm.
    /// \param[out] deltaY Position increment on Y axis since last call, in mm.
    void ADNS9800_getMotion(ADNS9800 a, double *deltaX, double *deltaY);
#endif
