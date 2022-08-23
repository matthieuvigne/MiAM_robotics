#ifndef APPLICATION_PREPARATION_HPP
#define APPLICATION_PREPARATION_HPP

#include <gtkmm.h>

#include <common/macros.hpp>

namespace application {

class PreparationWidget : public Gtk::Widget {

  public:
    PreparationWidget();
    virtual ~PreparationWidget();
    POINTER_TYPEDEF(PreparationWidget);
  
  public:
  
  private:

}; // class PreparationWidget
  
} // namespace application

#endif // APPLICATION_PREPARATION_HPP
