/// \file drivers/SparkfunOdometryDriver.h
/// \brief Driver for the Sparkfun odometry sensor, based on the PAA5160E1 + LSM6DSO
///
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef SparkfunOdometry_DRIVER
    #define SparkfunOdometry_DRIVER
    #include "miam_utils/drivers/I2C-Wrapper.h"
    #include "miam_utils/trajectory/RobotPosition.h"

    class SparkfunOdometry{
        public:
            SparkfunOdometry();

            /// \brief Init and test communication with driver.
            /// \return True if communication was successful.
            bool init(I2CAdapter *device, unsigned char const& address = 0x17);

            void resetPosition(miam::RobotPosition const& resetPos);

            /// \brief Reade position
            miam::RobotPosition read();


        private:
            I2CAdapter *adapter_;       ///< I2C port file descriptor.
            unsigned char address_;    ///< Led driver address.
            miam::RobotPosition positionOffset_;
    };
#endif
