/// \file IMUV3Driver.h
/// \brief Driver for Pololu MiniIMU v3 (L3GD20H and LSM303D).
///
/// \details This file contains all the functions related to communication
///             with the IMU, via I2C.
///    \note     Sensor precision setting is hardcoded, see source code directly.
/// \author Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef IMUV3DRIVER_H
    #define IMUV3DRIVER_H
    #include <miam_utils/drivers/I2C-Wrapper.h>

#ifndef IMUVECTOR
    #define IMUVECTOR
    ///< Basic structure holding a vector in R3.
    struct vector3D{
        vector3D():
            x(0.0),
            y(0.0),
            z(0.0)
        {}

        double x;
        double y;
        double z;
    };
#endif

    class IMUV3{
        public:
            /// \brief Constructor.
            IMUV3();

            /// \brief Init IMU sensor.
            /// \details This function also sets sensor resolution, which is hard-coded for now.
            ///
            /// \param[in] device I2C port.
            /// \param[in] turnOnMagneto Whether to turn on the magnetometer or not.
            /// \param[in] jumperShorted If jumper to select adress has been shorted or not.
            /// \return True on sensor init success.
            bool init(I2CAdapter *device, bool turnOnMagneto = false, bool jumperShorted = false);

            /// \brief Get gyroscope readings.
            /// \return Gyroscope readings on all three axis, in rad/s.
            vector3D getGyroscopeReadings();

            /// \brief Get accelerometer readings.
            /// \return Accelerometer readings on all three axis, in m/s2.
            vector3D getAccelerometerReadings();

            /// \brief Get magnetometer readings.
            /// \return Magnetometer readings on all three axis, in mgauss.
            vector3D getMagnetometerReadings();

        private:
            I2CAdapter *adapter_;
            unsigned char gyroAddress_; ///< Gyroscope I2C address.
            unsigned char accelAddress_; ///< Accelerometer I2C address.

            double gyroScaling_;    ///< Gyroscope scaling, from counts to rad/s.
            double accelScaling_;   ///< Accelerometer scaling, from counts to m/s2.
            double magnetoScaling_; ///< Magnetometer scaling, from counts to mgauss.

            bool isInit_;   ///< Status of initialization.
    };
#endif
