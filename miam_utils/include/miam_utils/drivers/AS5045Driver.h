/// \file AS5045Driver.h
/// \brief Driver for the AS5045 SPI magnetic encoder

#ifndef AS5045_DRIVER_H
#define AS5045_DRIVER_H

#include <miam_utils/drivers/SPI-Wrapper.h>
#include <vector>

class AS5045{
    public:
        /// \brief Default constructor.
        AS5045();

        /// \brief Constructor.
        /// \details This function only builds the object, but does not perform any operation on the
        ///          SPI port.
        /// \param[in] spiDriver SPI driver.
        /// \param[in] nEncoders Number of daisy-chained encoders.
        AS5045(SPIWrapper *spiDriver, uint32_t const& nEncoders = 1);

        bool init();

        /// \brief Read new position from sensor and update internal counter
        /// \return New position (rad)
        std::vector<double> updatePosition();

        /// \brief Get last-read position
        /// \return Position (rad)
        std::vector<double> getPosition() const;

    private:
        SPIWrapper* spiDriver_;
        bool isInit_;
        std::vector<double> previousSingleTurnPos_;
        std::vector<double> currentPosition_;

        /// \brief Read SPI message from the encoder
        ///
        /// \param[out] position Read position, in rad ; return previousSingleTurnPos_ on failure
        /// \return True if a valid position was read
        bool readSPI(std::vector<double> & position) const;

};


#endif
