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

#include <memory>

// Register address / offsets from base address.
// See BCM2837 manual, p.89 onward, for more information.

// Address of GPIO controller.
// Change to 0x20000000 for a raspberry pi 2 (BCM2835).
// Change to 0x3F000000 for a raspberry pi 3 (BCM2837).
// Change to 0x7E000000 for a raspberry pi 4 (BCM2711).
long int  GPIO_BASE_ADDRESS =  0x3F000000 + 0x200000;

int RPI_VERSION_NUMBER = 3;

#define GPSET0 7
#define GPCLR0 10

#define GPLEV0 13
#define GPPUD 37
#define GPPUDCLK0 38

#define GPIO_PUP_PDN_CNTRL 0xe4 / 4

// PWM registers.
#define PWM_BASE_ADDRESS (0x3F000000 + 0x20C000)
#define CLOCK_BASE_ADDRESS (0x3F000000 + 0x101000)

int const PWM_RANGE[2] = {4, 8};
int const PWM_DATA[2] = {5, 9};

#define PWM_CLK_CTRL 40
#define PWM_CLK_DIV 41

// Pointer to registers
volatile unsigned int *gpio_register = (volatile unsigned int *)MAP_FAILED;
volatile unsigned int *pwm_register = (volatile unsigned int *)MAP_FAILED;
volatile unsigned int *clock_register = (volatile unsigned int *)MAP_FAILED;


std::string exec(const char* cmd) {
    std::array<char, 128> buffer;
    std::string result;
    std::unique_ptr<FILE, decltype(&pclose)> pipe(popen(cmd, "r"), pclose);
    if (!pipe) {
        throw std::runtime_error("popen() failed!");
    }
    while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
        result += buffer.data();
    }
    return result;
}

bool RPi_enableGPIO()
{
    if (exec("cat /proc/cpuinfo | grep 'Model' | awk '{print $5}'") == "4\n")
    {
        RPI_VERSION_NUMBER = 4;
        GPIO_BASE_ADDRESS = 0x7E000000 + 0x200000;
    }


    gpio_register = (volatile unsigned int *)MAP_FAILED;
    pwm_register = (volatile unsigned int *)MAP_FAILED;
    clock_register = (volatile unsigned int *)MAP_FAILED;
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

    // Check: are we running as root ? If so, try to load PWM registers.
    if (getuid() == 0)
    {
        memoryFile = open("/dev/mem", O_RDWR|O_SYNC);
        if(memoryFile < 0)
        {
            #ifdef DEBUG
                std::cout << "Error opening /dev/mem: " << errno << " " << strerror(errno) << std::endl;
            #endif
            return false;
        }

        // Get PWM register
        map = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, memoryFile, PWM_BASE_ADDRESS);
        if(map == MAP_FAILED)
        {
            #ifdef DEBUG
                std::cout << "Error mapping PWM memory: " << errno << " " << strerror(errno) << std::endl;
            #endif
            return false;
        }
        pwm_register = (volatile unsigned int *)map;

        // Reset pwm
        *pwm_register = 0;
        usleep(10);

        // Get clock register
        map = mmap(NULL, 4096, PROT_READ | PROT_WRITE, MAP_SHARED, memoryFile, CLOCK_BASE_ADDRESS);
        if(map == MAP_FAILED)
        {
            #ifdef DEBUG
                std::cout << "Error mapping PWM memory: " << errno << " " << strerror(errno) << std::endl;
            #endif
            return false;
        }
        clock_register = (volatile unsigned int *)map;

        RPi_setPWMClock(PiPWMClockFrequency::F2400kHz);
    }
    else
    {
        #ifdef DEBUG
            std::cout << "Warning: not running as root, PWM is not available." << std::endl;
        #endif
    }


    return true;
}

void RPi_setupGPIO(unsigned int const& gpioPin, PiGPIOMode const& direction)
{
    if (gpio_register == MAP_FAILED)
    {
        #ifdef DEBUG
            std::cout << "[miam_utils] GPIO not initialized." << std::endl;
        #endif
        return;
    }
    // Check that GPIO number is valid.
    if(gpioPin < 4 || gpioPin > 26)
        return;
    // GPFSELn register corresponding to the current pin: set the three corresponding bits to 0.
    *(gpio_register + (gpioPin/10)) &=  ~( 0b111 << ((gpioPin % 10)*3));
    // If defined as output, set corresponding bits to 001
    if(direction == PI_GPIO_OUTPUT)
        *(gpio_register + (gpioPin/10)) |= 0b001 << ((gpioPin % 10)*3);
    // Setup pullups.

    if (RPI_VERSION_NUMBER == 4)
    {
        *(gpio_register + GPIO_PUP_PDN_CNTRL + (gpioPin/16)) &=  ~( 0b11 << ((gpioPin % 16)*2));
        if(direction == PI_GPIO_INPUT_PULLUP)
            *(gpio_register + GPIO_PUP_PDN_CNTRL + (gpioPin/16)) |=  ( 0b01 << ((gpioPin % 16)*2));
        else if(direction == PI_GPIO_INPUT_PULLDOWN)
            *(gpio_register + GPIO_PUP_PDN_CNTRL + (gpioPin/16)) |=  ( 0b10 << ((gpioPin % 16)*2));
    }
    else
    {
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
}

void RPi_writeGPIO(unsigned int const& gpioPin, bool const& value)
{
    if (gpio_register == MAP_FAILED)
    {
        #ifdef DEBUG
            std::cout << "[miam_utils] GPIO not initialized." << std::endl;
        #endif
        return;
    }
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
    if (gpio_register == MAP_FAILED)
    {
        #ifdef DEBUG
            std::cout << "[miam_utils] GPIO not initialized." << std::endl;
        #endif
        return false;
    }
    // Check that GPIO number is valid.
    if(gpioPin < 4 || gpioPin > 26)
        return LOW;
    // Get corresponding bit from GPLEV0
    return (*(gpio_register + GPLEV0) & (1 << gpioPin) ? HIGH : LOW);
}

void RPi_setPWMClock(PiPWMClockFrequency const& frequency)
{
    if (pwm_register == MAP_FAILED)
    {
        #ifdef DEBUG
            std::cout << "Error: PWM requires root priviledge." << std::endl;
        #endif
        return;
    }
    // Reset clock
    *(clock_register + PWM_CLK_CTRL) = 0x5A000001;
    usleep(100);
    // Set scaling
    *(clock_register + PWM_CLK_DIV) = 0x5A000000 | (frequency << 12);
    usleep(100);
    // Enable clock
    *(clock_register + PWM_CLK_CTRL) = 0x5A000011;
}

void RPi_enablePWM(bool const& enableFirstChannel, bool const& enableSecondChannel)
{
    if (pwm_register == MAP_FAILED)
    {
        #ifdef DEBUG
            std::cout << "Error: PWM requires root priviledge." << std::endl;
        #endif
        return;
    }
    // Set GPIO pins accordingly, and reset data.
    if (enableFirstChannel)
    {
        int gpioPin = 12;
        *(gpio_register + (gpioPin/10)) &=  ~( 0b111 << ((gpioPin % 10)*3));
        *(gpio_register + (gpioPin/10)) |=  0b100 << ((gpioPin % 10)*3);

        *(pwm_register + PWM_DATA[0]) = 0;
    }
    if (enableSecondChannel)
    {
        int gpioPin = 13;
        *(gpio_register + (gpioPin/10)) &=  ~( 0b111 << ((gpioPin % 10)*3));
        *(gpio_register + (gpioPin/10)) |= 0b100 << ((gpioPin % 10)*3);

        *(pwm_register + PWM_DATA[1])= 0;
    }

    // Activate PWM  - run in MS mode since the targeted application is motor control.
    *pwm_register = (enableFirstChannel ? 0x0081 : 0) | (enableSecondChannel ? 0x8100 : 0);
}

void RPi_setPWM(int const& channel, int const& dutyCycle, int const& periodCS)
{
    if (pwm_register == MAP_FAILED)
    {
        #ifdef DEBUG
            std::cout << "Error: PWM requires root priviledge." << std::endl;
        #endif
        return;
    }
    if(channel != 0 && channel != 1)
        return;

    *(pwm_register + PWM_RANGE[channel]) = periodCS;
    *(pwm_register + PWM_DATA[channel]) = dutyCycle;
}

void RPi_setPWMServo(int const& channel, int const& signal)
{
    int sig = std::max(500, std::min(signal, 2500));
    // At 1200kHz, 24000 ticks = 20ms
    RPi_setPWM(channel, static_cast<int>(1.2 * sig), 24000);
}