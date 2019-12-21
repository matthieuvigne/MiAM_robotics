/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "miam_utils/raspberry_pi/RPiGPIO.h"

#include <fstream>
#include <string>

#include <fcntl.h>

#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>

#include <unistd.h>

#include <errno.h>

#include <iostream>
#include <cstring>


// Register address / offsets from base address.
// See BCM2837 manual, p.89 onward, for more information.

// Address of GPIO controller.
// Change to 0x20000000 for a raspberry pi 2 (BCM2835).
#define GPIO_BASE_ADDRESS (0x3F000000 + 0x200000)

#define GPSET0 7
#define GPCLR0 10

#define GPLEV0 13
#define GPPUD 37
#define GPPUDCLK0 38

// Pointer to GPIO register
volatile unsigned int *gpio_register;

bool RPi_enableGPIO()
{
    // Open /dev/mem and connect to it.
    int memoryFile = open("/dev/gpiomem", O_RDWR|O_SYNC);
    if(memoryFile < 0)
    {
        #ifdef DEBUG
            std::cout << "Error opening /dev/gpiomem: " << errno << " " << strerror(errno) << std::endl;
        #endif
        return false;
    }

    // Map GPIO peripheral into memory. Second argument is block size, fifth is /dev/mem file, GPIO_BASE_ADDRESS gives
    // the target address.
    void *map = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, memoryFile, GPIO_BASE_ADDRESS);

    if(map == MAP_FAILED)
    {
        #ifdef DEBUG
            std::cout << "Error mapping GPIO memory: " << errno << " " << strerror(errno) << std::endl;
        #endif
        return false;
    }

    // Make gpio_register point to this address.
    gpio_register = (volatile unsigned int *)map;
    return true;
}

void RPi_setupGPIO(unsigned int const& gpioPin, PiGPIOMode const& direction)
{
    // Check that GPIO number is valid.
    if(gpioPin < 4 || gpioPin > 26)
        return;
    // GPFSELn register corresponding to the current pin: set the three corresponding bits to 0.
    *(gpio_register + (gpioPin/10)) &=  ~( 0b111 << ((gpioPin % 10)*3));
    // If defined as output, set corresponding bits to 001
    if(direction == PI_GPIO_OUTPUT)
        *(gpio_register + (gpioPin/10)) |= 0b001 << ((gpioPin % 10)*3);
    // Setup pullups.
    // Setup GPPUD register.
    *(gpio_register + GPPUD) = 0;
    if(direction == PI_GPIO_INPUT_PULLUP)
        *(gpio_register + GPPUD) = 0b10;
    else if(direction == PI_GPIO_INPUT_PULLDOWN)
        *(gpio_register + GPPUD) = 0b01;
    usleep(1);
    // Pulse corresponding GPPUDCLK bit.
    *(gpio_register + GPPUDCLK0) |= 1 << gpioPin;
    usleep(1);
    *(gpio_register + GPPUDCLK0) &= ~(1 << gpioPin);
}

void RPi_writeGPIO(unsigned int const& gpioPin, bool const& value)
{
    // Check that GPIO number is valid.
    if(gpioPin < 4 || gpioPin > 26)
        return;

    if(value == LOW)
        *(gpio_register + GPCLR0) = 1 << gpioPin;
    else
        *(gpio_register + GPSET0) = 1 << gpioPin;
}

bool RPi_readGPIO(unsigned int const& gpioPin)
{
    // Check that GPIO number is valid.
    if(gpioPin < 4 || gpioPin > 26)
        return LOW;
    // Get corresponding bit from GPLEV0
    return (*(gpio_register + GPLEV0) & (1 << gpioPin) ? HIGH : LOW);
}


int RPi_setPWM(int const& channel, int const& period_ns, int const& duty_ns)
{
    if(channel != 0 && channel != 1)
        return -1;
    std::string folderName = "/sys/class/pwm/pwmchip0/pwm" + std::to_string(channel) + "/";
    // Set period.
    std::ofstream file;
    file.open(folderName + "period", std::fstream::app);
    if(!file.is_open())
        return -1;
    file << period_ns << std::endl;
    file.close();
    // Set duty cycle.
    file.open(folderName + "duty_cycle", std::fstream::app);
    if(!file.is_open())
        return -1;
    file << duty_ns << std::endl;
    file.close();
    // Enable.
    file.open(folderName + "enable", std::fstream::app);
    if(!file.is_open())
        return -1;
    file << "1" << std::endl;
    file.close();
    return 0;
}

