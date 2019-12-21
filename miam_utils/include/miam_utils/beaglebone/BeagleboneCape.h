/// \file beaglebone/BeagleboneCape.h
/// \brief Code description of the PCB BeagleboneCape.
///
/// \details This file defines constants to easily access the exposed ports of the Beaglebone through the
///             BeagleboneCape. These constants map to output ports defined from the document BeagleboneCapeDoc below :
/// \image html BeagleboneCapeDoc.svg
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef BBB_CAPE
#define BBB_CAPE

    #include "MiAMEurobot/drivers/I2C-Wrapper.h"
    #include <string>

    #define CAPE_N_ANALOG 7
    #define CAPE_N_DIGITAL 7
    #define CAPE_N_LED 2

    extern const std::string SPI_0;    ///<  First SPI port.
    extern const std::string SPI_10; ///<  Second SPI port.
    extern const std::string SPI_11; ///<  Third SPI port.
    extern I2CAdapter I2C_1; ///<  First I2C port.
    extern const int CAPE_ANALOG[CAPE_N_ANALOG]; ///<  Analong input.
    extern const int CAPE_DIGITAL[CAPE_N_DIGITAL]; ///<  Digital GPIO.
    extern const int CAPE_LED[CAPE_N_LED]; ///<  LED output.

    /// \brief Enable the Beaglebone serial ports and gpio that are exposed through the cape.
    /// \details This function must be called at the start of the program, before any access to a
    ///          serial port or gpio. This function returns nothing, but stops the program execution is uncessussful.
    ///
    /// \warning This function uses the device tree overlay file "Eurobot-00A0.dtbo". This file must exist in
    ///          /lib/firmware, else this function fails. This function only activates this overlay : the behavior
    ///          of the system entirely depends on this file being properly written.
    void BBB_enableCape();

#endif
