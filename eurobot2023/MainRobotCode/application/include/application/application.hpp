#ifndef APPLICATION_APPLICATION_HPP
#define APPLICATION_APPLICATION_HPP

#include <iostream>

#include <gtkmm.h>

#include <application/initialization_widget.hpp>
#include <application/menu_widget.hpp>
#include <application/preparation_widget.hpp>
#include <application/sensors_widget.hpp>
#include <application/scoring_widget.hpp>
#include <application/viewer_widget.hpp>
#include <common/macros.hpp>

namespace application {

//--------------------------------------------------------------------------------------------------
// Class declaration
//--------------------------------------------------------------------------------------------------

class RobotApplication : public Gtk::Application {
  
  protected:
    RobotApplication();

  public:
    virtual ~RobotApplication();
    
  public:
    static Glib::RefPtr<RobotApplication> create();
  
  protected:

    /* The application is activated when the 'run' function is called>
     * It thus calls the signal handle 'on_activate' which we override below.
     */
    void on_activate() override;
  
  protected:
  
    // Application window
    Glib::RefPtr<Gtk::Builder> builder_;
    Gtk::Window* main_window_;
  
    // Menu widgets
    MenuWidget main_menu_;
    InitializationWidget initialization_;
    PreparationWidget preparation_;
    ScoringWidget scoring_;
    SensorsWidget sensors_;
    ViewerWidget viewer_;
  
}; // class RobotApplication

//--------------------------------------------------------------------------------------------------

} // namespace application

#endif // APPLICATION_APPLICATION_HPP
