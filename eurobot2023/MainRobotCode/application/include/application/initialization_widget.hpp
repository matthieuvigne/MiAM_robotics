#ifndef APPLICATION_INITIALIZATION_HPP
#define APPLICATION_INITIALIZATION_HPP

#include <gtkmm.h>

#include <common/macros.hpp>

namespace application {
  
class InitializationWidget : public Gtk::Widget {

  public:
    InitializationWidget();
    virtual ~InitializationWidget();
    POINTER_TYPEDEF(InitializationWidget);
  
  public:
  
  private:

}; // class InitializationWidget

} // namespace application

#endif // APPLICATION_INITIALIZATION_HPP
