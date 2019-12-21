/// \file drivers/USBLCDDriver.h
/// \brief Driver for the robot USB human interface.
///
/// \details The physical board is a modified version of Adafruit USB LCD shield (https://www.adafruit.com/product/782)
///          with three added buttons and LEDs. The firmware was modified to include the extra sensors. Note that
///          the serial interface was dissabled to support these features.
/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#ifndef USBLCD_DRIVER_H
    #define USBLCD_DRIVER_H

    #include <string>

    namespace lcd{
        enum {
            LEFT_LED = 0b001,
            MIDDLE_LED = 0b010,
            RIGHT_LED = 0b100,
            LEFT_BUTTON = 0b001,
            MIDDLE_BUTTON = 0b010,
            RIGHT_BUTTON = 0b100
        };
    }
    class USBLCD{
        public:
            /// \brief Default contstructor.
            USBLCD();

            /// \brief Initialize communication with the screen.
            ///
            /// \param[in] fileName Name of the USB file to open.
            /// \return true if init went fine, false otherwise.
            bool init(std::string const& fileName);

            /// \brief Set the text of a given LCD line, left-aligned.
            ///
            /// \param[in] text Text to display. Only the first 16 characters will fit the screen.
            ///                    The text is left-aligned and padded by spaces to clear the display line.
            /// \param[in] line Line number (0 or 1).
            /// \param[in] centered If the text should be centered. Default is true.
            void setText(std::string const& text, int const& line = 0, bool centered = true);


            /// \brief Set LCD backlight
            ///
            /// \param[in] red Red value, 0-255.
            /// \param[in] green Green value, 0-255.
            /// \param[in] blue Blue value, 0-255.
            void setLCDBacklight(uint const& red, uint const& green, uint const& blue);

            /// \brief Set all three LEDs state.
            /// \detail Use lcd enum value to select the LED to turn on. These values can be combined together:
            ///         turnOnLed(LEFT_LED | MIDDLE_LED) will turn on both leds, and turn off the right one.
            /// \param[in] leds Desired led status.
            void setLED(uint8_t const& leds);

            /// \brief Turn on one or several LEDs.
            /// \detail Use lcd enum value to select the LED to turn on. These values can be combined together:
            ///         turnOnLed(LEFT_LED | MIDDLE_LED) will turn on both leds, and leave the last one untouched.
            /// \param[in] leds LEDs id of the leds to turn on.
            void turnOnLED(uint8_t const& leds);

            /// \brief Turn off one or several LEDs.
            /// \detail Use lcd enum value to select the LED to turn on. These values can be combined together:
            ///         turnOffLed(LEFT_LED | MIDDLE_LED) will turn off both leds, and leave the last one untouched.
            /// \param[in] leds LEDs id of the leds to turn on.
            void turnOffLED(uint8_t const& leds);

            /// \brief Get the state of each button.
            /// \return The state of the three buttons: use lcd enum to get individual button press.
            uint8_t getButtonState();
        private:

            int port_; ///< File descriptor of the port used to talk to the board.
            uint8_t LEDState_; ///< Current LED state - only lower three bits are used.

            /// \brief Get button state, return 255 if unexpected result is obtained.
            uint8_t getButtonStateRaw();
    };
#endif
