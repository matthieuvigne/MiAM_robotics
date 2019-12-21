/// \file drivers/LCDDriver.h
/// \brief Driver for Adafruit RBG LCD shield.
///
/// \details This shield is made of an MCP23017 I2C IO expander, wired to an HD44780-compatible LCD screen.
///    \note     All functions in this header should be prefixed with lcd_.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef LCD_DRIVER_H
    #define LCD_DRIVER_H
    #include "MiAMEurobot/drivers/MPC23017Driver.h"

    ///< Mapping of board buttons.
    typedef enum
    {
      LCD_BUTTON_SELECT    = 0,
      LCD_BUTTON_RIGHT     = 1,
      LCD_BUTTON_DOWN      = 2,
      LCD_BUTTON_UP        = 3,
      LCD_BUTTON_LEFT   = 4
    }LCDButton;


    ///< LCD class
    class LCD{
        public:
            /// \brief Default constructor, does nothing.
            LCD();

            /// \brief Initialize LCD structure.
            ///
            /// \details This function tests the communication with the LCD, and, if successful, inits the structure.
            ///
            /// \param[in] adapter Pointer to a valid I2CAdapter to choose the I2C port (as returned by the i2c_open function,
            ///                    see I2C-Wrapper.h).
            /// \param[in] slaveAddress Address of the I2C slave.
            /// \returns   true on success, false otherwise.
            bool init(I2CAdapter *adapter, int const& slaveAddress = 0x20);

            /// \brief Set the text of a given LCD line, centering it.
            ///        This function operates asyncronously and retuns immediately.
            /// \note  Writing a full line to the LCD currently takes about 70ms.
            ///
            /// \param[in] text Text to display. Only the first 16 characters will fit the screen.
            ///                 The string should be null-terminated, otherwise it will not be centered correctly,
            ///                    and garbage will be displayed.
            /// \param[in] line Line number (0 or 1).
            void setTextCentered(std::string const& text, int line);

            /// \brief Set LCD backlight
            ///
            /// \param[in] red Whether to turn on the red led.
            /// \param[in] green Whether to turn on the green led.
            /// \param[in] blue Whether to turn on the blue led.
            void setBacklight(bool red, bool green, bool blue);

            /// \brief Get button status.
            ///
            /// \param[in] button Member of LCDButton enum specifying the button to read.
            /// \returns   true is button is pressed, False otherwise
            bool isButtonPressed(LCDButton button);

        private:
            void pulseEnable();
            void sendData(unsigned char const& data, unsigned char const& rsValue);
            void sendCommand(unsigned char value);
            void sendChar(unsigned char value);

            MPC mpc_;
            std::mutex mutex_;

            std::string lines_[2];  ///< Buffer: lines to send to the screen.
            int backlight_;         ///< Buffer: backlight to set.
            int buttons_;           ///< Buffer: last button status.
            bool updateScreen_;     ///< Variable set when a screen update is required.

            /// \brief Background thread handling communication with LCD screen.
            void lcdLoop();

            /// \brief Set the text of a given LCD line, left-aligned.
            /// \note  Writing a full line to the LCD currently takes about 70ms.
            ///
            /// \param[in] text Text to display. Only the first 16 characters will fit the screen.
            ///                 The string should be null-terminated, otherwise random garbage will be displayed.
            ///                    The text is left-aligned and padded by spaces to clear the display line.
            /// \param[in] line Line number (0 or 1).
            void setText(std::string const& text, int line);
    };

#endif
