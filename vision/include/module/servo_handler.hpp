#ifndef MODULE_SERVO_HANDLER_HPP
#define MODULE_SERVO_HANDLER_HPP

#include <common/macros.hpp>
#include <miam_utils/drivers/MaestroServoDriver.h>

namespace module {

class ServoHandler {

public:

  POINTER_TYPEDEF(ServoHandler);
  ServoHandler();
  virtual ~ServoHandler();

public:

  bool init(std::string const& port_name);
  void rotateCamera(double angle_deg);
  void shutdownServos();

private:

  MaestroDriver maestro_;

}; // class ServoHandler

} // namespace module

#endif // MODULE_SERVO_HANDLER_HPP
