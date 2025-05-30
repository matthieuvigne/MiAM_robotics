/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/drivers/USBLCDDriver.h"
#include "miam_utils/drivers/UART-Wrapper.h"

#include <iostream>
#include <cstring>
#include <unistd.h>
// Microcontroller commands.
// These commands come from the modified Adafruit code, to include ways to interact with the buttons and LEDs.
// A command should be prefixed by 0xFE

#define EXTENDED_RGBBACKLIGHT 0xD0
#define MATRIX_SETCURSOR_POSITION 0x47
#define MATRIX_CLEAR 0x58
#define MATRIX_DISPLAY_ON 0x42

#define ADDON_GET_BUTTON_STATE 0xAA
#define ADDON_SET_LED_STATE 0xAB

USBLCD::USBLCD():
    port_(-1),
    LEDState_(0)
{
}

bool USBLCD::init(std::string const& fileName)
{
    port_ = uart_open(fileName.c_str(), B500000);
    if(port_ < 0)
        return false;

    // Reset display.
    unsigned char message[2];
    message[0] = 0xFE;
    message[1] = MATRIX_CLEAR;
    int result = write(port_, message, 2);

    // Check that the display is present by doing a read button command.
    if(result < 0 || getButtonStateRaw() == 255)
    {
        #ifdef DEBUG
            std::cout << "USBLCD error: a USB device is connected but does not respond like expected." << std::endl;
        #endif
        return false;
    }
    for (int i = 0; i < 3; i++)
    {
        lastButtonState_[i] = true;
    }
    return true;

}


void USBLCD::setText(std::string const& text, int const& line, bool centered)
{
    // Set cursor position to start of line.
    unsigned char message[4];
    message[0] = 0xFE;
    message[1] = MATRIX_SETCURSOR_POSITION;
    message[2] = 1;
    message[3] = (line == 1 ? 2 : 1);
    int result = write(port_, message, 4);
    if (result < 0)
    {
        #ifdef DEBUG
            std::cout << "USBLCD error: failed to write: "  << std::strerror(errno) << std::endl;
        #endif
    }

    // Send the data, padded with space to fill the line.
    int const lineLength = 16;
    std::string paddedText = text;
    if(paddedText.length() < lineLength)
    {
        if(centered)
            paddedText.insert(paddedText.begin(), (lineLength - paddedText.length()) / 2, ' ');

        paddedText.insert(paddedText.end(), lineLength - paddedText.length(), ' ');
    }
    result = write(port_, paddedText.c_str(), lineLength);
    if (result < 0)
    {
        #ifdef DEBUG
            std::cout << "USBLCD error: failed to write: "  << std::strerror(errno) << std::endl;
        #endif
    }
}


void USBLCD::setLCDBacklight(uint const& red, uint const& green, uint const& blue)
{
    if(port_ < 0)
        return;

    unsigned char message[5];
    message[0] = 0xFE;
    message[1] = EXTENDED_RGBBACKLIGHT;
    message[2] = red & 0xFF;
    message[3] = green & 0xFF;
    message[4] = blue & 0xFF;
    int result = write(port_, message, 5);
    if (result < 0)
    {
        #ifdef DEBUG
            std::cout << "USBLCD error: failed to write: "  << std::strerror(errno) << std::endl;
        #endif
    }
}


void USBLCD::setLED(uint8_t const& leds)
{
    LEDState_ = leds;
    unsigned char message[3];
    message[0] = 0xFE;
    message[1] = ADDON_SET_LED_STATE;
    message[2] = LEDState_;
    int result = write(port_, message, 3);
    if (result < 0)
    {
        #ifdef DEBUG
            std::cout << "USBLCD error: failed to write: "  << std::strerror(errno) << std::endl;
        #endif
    }
}


void USBLCD::turnOnLED(uint8_t const& leds)
{
    LEDState_ |= leds;
    setLED(LEDState_);
}


void USBLCD::turnOffLED(uint8_t const& leds)
{
    LEDState_ &= ~leds;
    setLED(LEDState_);
}


uint8_t USBLCD::getButtonState()
{
    uint8_t state = getButtonStateRaw();
    if(state == 255)
        state = 0;
    return state;
}


bool USBLCD::wasButtonPressedSinceLastCall(lcd::button const& button)
{
    int const b = static_cast<int>(button);
    uint8_t mask = 1;
    if (b > 0)
        mask = mask << b;
    uint8_t state = getButtonStateRaw();
    if (state == 0xFF)
        return false;
    bool const currentState = state & mask;
    bool const wasPressed = currentState & !lastButtonState_[b];
    lastButtonState_[b] = currentState;
    return wasPressed;
}


uint8_t USBLCD::getButtonStateRaw()
{
    // Flush port.
    tcflush(port_,TCIOFLUSH);

    unsigned char message[2];
    message[0] = 0xFE;
    message[1] = ADDON_GET_BUTTON_STATE;
    int result = write(port_, message, 2);
    if (result < 0)
    {
        #ifdef DEBUG
            std::cout << "USBLCD error: failed to write: "  << std::strerror(errno) << std::endl;
        #endif
    }
    message[0] = 0;

    // Read with 20ms timeout - on failure set message to 0xFF.
    if(read_timeout(port_, message, 1, 20) != 1)
        message[0] = 255;

    // Check that what we receive is a valid button value, otherwise return 255.
    if((message[0] & 0b11111000) != 0b10000000)
        return 255;
    return ~(message[0] & 0b00000111);
}
