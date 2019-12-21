/// \author MiAM Robotique, Matthieu Vigne
/// \copyright GNU GPLv3
#include "MiAMEurobot/beaglebone/BBBGpio.h"

#include <fstream>
#include <string>

// Internal function : get the content of the first line of the file. Returns TRUE on success, FALSE on error.
bool getFileContent(std::string const& filename, std::string& output)
{
    std::ifstream file(filename);
    if(!file.is_open())
        return false;

    getline(file, output);
    return true;
}

int gpio_digitalRead(int const& pin)
{
    // Get path to gpio direction file.
    std::string filename = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::string filecontent;
    bool returnCode = getFileContent(filename, filecontent);
    if(returnCode == false)
        return -1;

    // Check file is an input.
    if (filecontent != "in")
        return -2;

    // Read gpio value.
    filename = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    returnCode = getFileContent(filename, filecontent);
    if(returnCode == false)
        return -1;

    int value = -1;
    try
    {
        value = std::stoi(filecontent);
    }
    catch(...) {}
    return value;
}

int gpio_digitalWrite(int const& pin, int const& value)
{
    std::string filename = "/sys/class/gpio/gpio" + std::to_string(pin) + "/direction";
    std::string filecontent;
    bool returnCode = getFileContent(filename, filecontent);
    if(returnCode == false)
        return -1;

    // Check file is an output.
    if (filecontent != "out")
        return -2;

    filename = "/sys/class/gpio/gpio" + std::to_string(pin) + "/value";
    std::ofstream file;
    file.open(filename);
    if(!file.is_open())
        return -1;
    file << (value == 0 ? 0 : 1) << std::endl;
    file.close();
    return 0;
}


int gpio_exportPin(int const& pin, std::string const& direction)
{
    // Export pin value.
    std::ofstream file;
    file.open("/sys/class/gpio/export", std::fstream::app);
    if(!file.is_open())
        return -1;
    file << pin << std::endl;
    file.close();

    // Set gpio direction.
    file.open("/sys/class/gpio/gpio" + std::to_string(pin) + "/direction");
    if(!file.is_open())
        return -1;
    file << direction << std::endl;
    file.close();
    return 0;
}


int gpio_analogRead(int const& pin)
{
    if(pin < 0 || pin > 6)
        return -2;

    std::string filename = "/sys/bus/iio/devices/iio:device0/in_voltage" + std::to_string(pin) + "_raw";
    std::string filecontent;
    bool returnCode = getFileContent(filename, filecontent);
    if(returnCode == false)
        return -1;

    int value = -1;
    try
    {
        value = std::stoi(filecontent);
    }
    catch(...) {}
    return value;
}


int gpio_setPWM(int const& port, int const& period_ns, int const& duty_ns)
{
    if(port != 0 && port != 1)
        return -1;
    std::string folderName = "/sys/class/pwm/pwm" + std::to_string(port) + "/";
    // Set period.
    std::ofstream file;
    file.open(folderName + "period_ns", std::fstream::app);
    if(!file.is_open())
        return -1;
    file << period_ns << std::endl;
    file.close();
    // Set duty cycle.
    file.open(folderName + "duty_ns", std::fstream::app);
    if(!file.is_open())
        return -1;
    file << duty_ns << std::endl;
    file.close();
    // Enable.
    file.open(folderName + "run", std::fstream::app);
    if(!file.is_open())
        return -1;
    file << "1" << std::endl;
    file.close();
    return 0;
}


int gpio_servoPWM(int const& port, int const& servoPosition)
{
    return gpio_setPWM(port, 20000000, 1000 * servoPosition);
}

