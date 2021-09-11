/// \file uCListener.h
/// \brief Communication with a slave micro-controller to access some sensors.
///
/// \details This uC constantly broadcast the status of the sensor. This file runs a thread that listen to the serial
///          port, update the value as needed and makes it available through a specific data structure.
///
///    \note     All functions in this header should be prefixed with uCListener_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef ARDUINOSLAVE_H
     #define ARDUINOSLAVE_H

    ///< Global includes
    #include <miam_utils/miam_utils.h>

    typedef struct {
        double encoderValues[2]; ///<< Current position of the two encoders, in rad.
        int potentiometerPosition; ///<< Position of the vertical rail potentiometer.
    }uCData;

    /// \brief Start a background thread listening to the arduino microcontroller.
    ///
    /// \details This function starts the thread, and waits for a reply from the Arduino. This may take up to 2s.
    ///
    /// \param[in] portName Name of the serial port to connect to.
    /// \note By default, the port name to which the Arduino connects can change (/dev/ttyACMx). To bind to a fix
    /// path, create a rule in /etc/udev/rules.d/10-local.rules. For instance, for an Arduino Uno board add the following line:
    /// SUBSYSTEM=="tty", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0043", SYMLINK+="arduinoUno"
    /// This makes any arduino board match the symlink /dev/arduinoUno
    /// \return True if communication with Arduino is successful.
    bool uCListener_start(std::string const& portName);

    /// \brief Get the last value read from the sensors.
    uCData uCListener_getData();

 #endif
