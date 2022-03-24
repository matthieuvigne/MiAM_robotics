#include <miam_utils/raspberry_pi/RPiGPIO.h>
#include <module/servo_handler.hpp>

namespace module {

//--------------------------------------------------------------------------------------------------
// Constructor
//--------------------------------------------------------------------------------------------------

ServoHandler::ServoHandler(){}

//--------------------------------------------------------------------------------------------------

ServoHandler::~ServoHandler(){}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

bool ServoHandler::init(std::string const& port_name)
{
  unsigned int const gpio_pin_number = 20; // Adapt
  PiGPIOMode const gpio_mode = PI_GPIO_OUTPUT;
  RPi_setupGPIO(gpio_pin_number, gpio_mode);
  return this->maestro_.init(port_name);
}

//--------------------------------------------------------------------------------------------------

void ServoHandler::rotateCamera(double angle_deg)
{
  // TODO
  int const servo_index = 1;
  double const position = angle_deg;
  this->maestro_.setPosition(servo_index, angle_deg); 
}

//--------------------------------------------------------------------------------------------------

void ServoHandler::shutdownServos()
{
  // TODO
}

//--------------------------------------------------------------------------------------------------

} // namespace module
