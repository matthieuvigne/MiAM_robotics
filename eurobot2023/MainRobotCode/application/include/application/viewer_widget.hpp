#ifndef APPLICATION_VIEWER_HPP
#define APPLICATION_VIEWER_HPP

#include <gtkmm.h>

#include <common/macros.hpp>

namespace application {

class ViewerWidget : public Gtk::Widget {

  public:
    ViewerWidget();
    virtual ~ViewerWidget();
    POINTER_TYPEDEF(ViewerWidget);
  
  public:
  
  private:

}; // class ViewerWidget
  
} // namespace application

#endif // APPLICATION_VIEWER_HPP
