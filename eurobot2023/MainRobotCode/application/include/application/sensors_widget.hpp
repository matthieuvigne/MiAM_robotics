#ifndef APPLICATION_SENSOR_HPP
#define APPLICATION_SENSOR_HPP

#include <gtkmm.h>

#include <common/macros.hpp>

namespace application {

class SensorsWidget : public Gtk::Widget {

  public:
    SensorsWidget();
    virtual ~SensorsWidget();
    POINTER_TYPEDEF(SensorsWidget);
  
  public:
  
  private:

}; // class SensorsWidget  
  
} // namespace application

#endif // APPLICATION_SENSOR_HPP
