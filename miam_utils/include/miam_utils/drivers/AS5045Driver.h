/// \file AS5045Driver.h
/// \brief Driver for the AS5045 SPI magnetic encoder

#ifndef AS5045_DRIVER_H
#define AS5045_DRIVER_H

#include <miam_utils/drivers/SPI-Wrapper.h>

class AS5045{
    public:
        /// \brief Default constructor.
        AS5045();

        /// \brief Constructor.
        /// \details This function only builds the object, but does not perform any operation on the
        ///          SPI port.
        /// \param[in] spiDriver SPI driver.
        AS5045(SPIWrapper *spiDriver);

        bool init();

        /// \brief Read new position from sensor and update internal counter
        /// \return New position (rad)
        double updatePosition();

        /// \brief Get last-read position
        /// \return Position (rad)
        double getPosition() const;

    private:
        SPIWrapper* spiDriver_;
        bool isInit_;
        double previousSingleTurnPos_;
        double currentPosition_;

        /// \brief Read SPI message from the encoder
        ///
        /// \param[out] position Read position, in rad ; return previousSingleTurnPos_ on failure
        /// \return True if a valid position was read
        bool readSPI(double & position) const;

};


#endif
