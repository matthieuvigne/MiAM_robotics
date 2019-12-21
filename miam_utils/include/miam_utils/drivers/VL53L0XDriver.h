/// \file drivers/LCDDriver.h
/// \brief Driver for the VL53L0X time-of-flight range sensor.
///
/// \details This driver is a simplified transcript of the code presented by pololu at
///          https://github.com/pololu/vl53l0x-arduino
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef VL53L0X_DRIVER_H
    #define VL53L0X_DRIVER_H
    #include "MiAMEurobot/drivers/I2C-Wrapper.h"

    ///< VL53L0X class
    class VL53L0X{
        public:
            /// \brief Default constructor, does nothing.
            VL53L0X();

            /// \brief Initialize the chip.
            ///
            /// \details This function tests the communication with the VL53L0X, and if successful, configures and
            ///          starts up the device.
            ///
            /// \param[in] adapter Pointer to a valid I2CAdapter to choose the I2C port (as returned by the i2c_open function,
            ///                    see I2C-Wrapper.h).
            /// \param[in] slaveAddress Address of the I2C slave.
            /// \returns   true on success, false otherwise.
            bool init(I2CAdapter *adapter, int const& slaveAddress = 0b0101001);

            /// \brief Get range measurement.
            ///
            /// \return Range measurement, in mm.
            int getMeasurement();

        private:
     enum vcselPeriodType { VcselPeriodPreRange, VcselPeriodFinalRange };

        uint8_t last_status; // status of last I2C transmission


        void setAddress(uint8_t new_addr);
        inline uint8_t getAddress(void) { return address; }

        bool init(bool io_2v8 = true);

        void writeReg(uint8_t reg, uint8_t value);
        void writeReg16Bit(uint8_t reg, uint16_t value);
        void writeReg32Bit(uint8_t reg, uint32_t value);
        uint8_t readReg(uint8_t reg);
        uint16_t readReg16Bit(uint8_t reg);

        void writeMulti(uint8_t reg, uint8_t const * src, uint8_t count);
        void readMulti(uint8_t reg, uint8_t * dst, uint8_t count);

        bool setMeasurementTimingBudget(uint32_t budget_us);
        uint32_t getMeasurementTimingBudget(void);

        bool setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);
        uint8_t getVcselPulsePeriod(vcselPeriodType type);


        bool timeoutOccurred(void);

        // TCC: Target CentreCheck
        // MSRC: Minimum Signal Rate Check
        // DSS: Dynamic Spad Selection

        struct SequenceStepEnables
        {
          bool tcc, msrc, dss, pre_range, final_range;
        };

        struct SequenceStepTimeouts
        {
          uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;

          uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
          uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
        };

        uint8_t address;
        uint16_t io_timeout;
        bool did_timeout;
        uint16_t timeout_start_ms;

        uint8_t stop_variable; // read by init and used when starting measurement; is StopVariable field of VL53L0X_DevData_t structure in API
        uint32_t measurement_timing_budget_us;

        bool getSpadInfo(uint8_t * count, bool * type_is_aperture);

        void getSequenceStepEnables(SequenceStepEnables * enables);
        void getSequenceStepTimeouts(SequenceStepEnables const * enables, SequenceStepTimeouts * timeouts);

        bool performSingleRefCalibration(uint8_t vhv_init_byte);

        static uint16_t decodeTimeout(uint16_t value);
        static uint16_t encodeTimeout(uint16_t timeout_mclks);
        static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
        static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);

        private:
            I2CAdapter *adapter_; ///< I2C port adatper.
            int address_; ///< Slave address
    };

#endif
