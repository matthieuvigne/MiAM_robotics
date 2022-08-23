#include <gtkmm/window.h>
#include <gtkmm/widget.h>

#include <application/application.hpp>

namespace application {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

RobotApplication::RobotApplication()
: Gtk::Application(),
  builder_ {Gtk::Builder::create()}
{
  // Load the main window from the corresponding builder
  try
  {
    builder_->add_from_file("../config/main_window.glade");
  } catch(Glib::FileError const& e){
    std::cerr << "File error: " << e.what() << std::endl;
    return;
  } catch(Glib::MarkupError const& e){
    std::cerr << "Markup error: " << e.what() << std::endl;
    return;
  } catch(Gtk::BuilderError const& e) {
    std::cerr << "Builder error: " << e.what() << std::endl;
    return;
  }
  
  // Create and the main window
  // Don't add any window to the application before it gets initialized.
  // The application gets initialized when it is run, via the on_activate signal handler.
  //~ main_window_.set_default_size(800,480);
  //~ main_window_.set_title("Initialization of the robot");
  
  // Launch the main menu
  //~ main_window_.add(main_menu_);
}

//--------------------------------------------------------------------------------------------------

RobotApplication::~RobotApplication()
{
  // [EMPTY DESTRUCTOR]
}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

Glib::RefPtr<RobotApplication> RobotApplication::create()
{
  return Glib::RefPtr<RobotApplication>(new RobotApplication());
}

//--------------------------------------------------------------------------------------------------

void RobotApplication::on_activate()
{
  builder_->get_widget<Gtk::Window>("main_window",main_window_);
  add_window(*main_window_);
  main_window_->present();
  //~ main_window_->show_all_children();
}

//--------------------------------------------------------------------------------------------------

} // namespace application
