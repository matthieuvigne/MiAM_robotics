#ifndef APPLICATION_MENU_HPP
#define APPLICATION_MENU_HPP

#include <gtkmm.h>

#include <common/macros.hpp>

namespace application {

class MenuWidget : public Gtk::Frame {
  
  public:
    MenuWidget();
    virtual ~MenuWidget();
    POINTER_TYPEDEF(MenuWidget);
  
  public:
  
  private:
  
    // Initialization
    void initButton(Gtk::Button* button_ptr);
  
    // Signal handler
    void on_button_clicked();
  
  private:
  
    // Layout items
    Gtk::Grid grid_;
    Gtk::ButtonBox above_box_;
    Gtk::ButtonBox below_box_;
  
    // Available buttons from the main menu
    Gtk::Button initiazation_;
    Gtk::Button preparation_;
    Gtk::Button scoring_;
    Gtk::Button sensors_;
    Gtk::Button viewer_;
  
}; // class MenuWidget
  
} // namespace application

#endif // APPLICATION_MENU_HPP
