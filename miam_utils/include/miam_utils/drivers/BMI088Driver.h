/// \file BMI088Driver.h
/// \brief Driver for BMI-088 6 axis IMU
#ifndef BMI088DRIVER_H
    #define BMI088DRIVER_H
    #include <miam_utils/drivers/I2C-Wrapper.h>
    #include <eigen3/Eigen/Core>

    class BMI088{
        public:
            /// \brief Constructor.
            BMI088();

            /// \brief Init IMU sensor.
            /// \details This function also sets sensor resolution, which is hard-coded for now.
            ///
            /// \param[in] device I2C port.
            /// \param[in] accelAddress Accelerometer I2C address
            /// \param[in] gyroAddress Gyroscope I2C address
            /// \return True on sensor init success.
            bool init(I2CAdapter *device, unsigned char const& accelAddress = 0x19, unsigned char const& gyroAddress = 0x69);

            /// \brief Get gyroscope readings.
            /// \return Gyroscope readings on all three axis, in rad/s.
            Eigen::Vector3f getGyroscopeReadings();

            /// \brief Get accelerometer readings.
            /// \return Accelerometer readings on all three axis, in m/s2.
            Eigen::Vector3f getAccelerometerReadings();

        private:
            I2CAdapter *adapter_;
            unsigned char accelAddress_; ///< Accelerometer I2C address.
            unsigned char gyroAddress_; ///< Gyroscope I2C address.

            double accelScaling_;   ///< Accelerometer scaling, from counts to m/s2.
            double gyroScaling_;    ///< Gyroscope scaling, from counts to rad/s.

            bool isInit_;   ///< Status of initialization.
    };
#endif
