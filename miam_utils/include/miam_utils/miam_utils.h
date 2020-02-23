/// \file miam_utils.h
/// \brief Master header file.
///
/// \details This header simply includes all the other headers from the library, as well as the glib header,
///          in order to ease library loading.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef MIAM_EUROBOT
#define MIAM_EUROBOT

    #include <miam_utils/AbstractRobot.h>
    #include <miam_utils/KalmanFilter.h>
    #include <miam_utils/Logger.h>
    #include <miam_utils/Metronome.h>
    #include <miam_utils/PID.h>

    #include <miam_utils/trajectory/ArcCircle.h>
    #include <miam_utils/trajectory/PointTurn.h>
    #include <miam_utils/trajectory/StraightLine.h>
    #include <miam_utils/trajectory/Utilities.h>
    #include <miam_utils/trajectory/DrivetrainKinematics.h>
    #include <miam_utils/trajectory/ThreeWheelsKinematics.hpp>

    #include <miam_utils/drivers/ADNS9800Driver.h>
    #include <miam_utils/drivers/L6470Driver.h>
    #include <miam_utils/drivers/I2C-Wrapper.h>
    #include <miam_utils/drivers/IMUDriver.h>
    #include <miam_utils/drivers/LCDDriver.h>
    #include <miam_utils/drivers/MaestroServoDriver.h>
    #include <miam_utils/drivers/PCA9635Driver.h>
    #include <miam_utils/drivers/SPI-Wrapper.h>
    #include <miam_utils/drivers/TCS3472ColorSensorDriver.h>
    #include <miam_utils/drivers/UART-Wrapper.h>
    #include <miam_utils/drivers/VL53L0XDriver.h>

#endif
