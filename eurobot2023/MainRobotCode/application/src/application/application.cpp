#include <gtkmm/window.h>
#include <gtkmm/widget.h>

#include <application/application.hpp>

namespace application {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

RobotApplication::RobotApplication()
: Gtk::Application(),
  builder_     {Gtk::Builder::create()}
{
  // Load the main window from the corresponding builder
  try
  {
    builder_->add_from_file("./config/main_window.glade");
    builder_->add_from_file("./config/init_panel.glade");
    builder_->add_from_file("./config/match_panel.glade");
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
  
  // Get the sidebar buttons and add callbacks
  
  Gtk::Button* init_button;
  builder_->get_widget<Gtk::Button>("init_button",init_button);
  init_button->signal_clicked().connect(
    sigc::mem_fun(*this,&RobotApplication::on_init_button_clicked));
  
  Gtk::Button* match_button;
  builder_->get_widget<Gtk::Button>("match_button",match_button);
  match_button->signal_clicked().connect(
    sigc::mem_fun(*this,&RobotApplication::on_match_button_clicked));
  
  Gtk::Button* score_button;
  builder_->get_widget<Gtk::Button>("score_button",score_button);
  score_button->signal_clicked().connect(
    sigc::mem_fun(*this,&RobotApplication::on_score_button_clicked));
  
  Gtk::Button* sensor_button;
  builder_->get_widget<Gtk::Button>("sensor_button",sensor_button);
  sensor_button->signal_clicked().connect(
    sigc::mem_fun(*this,&RobotApplication::on_sensor_button_clicked));
  
  Gtk::Button* strategy_button;
  builder_->get_widget<Gtk::Button>("strategy_button",strategy_button);
  strategy_button->signal_clicked().connect(
    sigc::mem_fun(*this,&RobotApplication::on_strategy_button_clicked));
  
  // Initialization panel
  init_tree_model_ = Gtk::ListStore::create(init_columns_);
  init_tree_view_.set_model(init_tree_model_);
  Gtk::TreeModel::Row row = *(init_tree_model_->append());
  row[init_columns_.col_type_] = "Sensor 1";
  row[init_columns_.col_status_] = "OK";
  row[init_columns_.col_comment_] = "Activated";
  row = *(init_tree_model_->append());
  row[init_columns_.col_type_] = "Sensor 2";
  row[init_columns_.col_status_] = "NOK";
  row[init_columns_.col_comment_] = "Sensor not found";
  init_tree_view_.append_column("Type",init_columns_.col_type_);
  init_tree_view_.append_column("Status",init_columns_.col_status_);
  init_tree_view_.append_column("Comment",init_columns_.col_comment_);
  
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

RobotApplication::InitModelColumns::InitModelColumns()
{
  add(col_type_);
  add(col_status_);
  add(col_comment_);
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
  // Display the main window
  builder_->get_widget<Gtk::Window>("main_window",main_window_);
  add_window(*main_window_);
  main_window_->present();
  //~ main_window_->show_all_children();
}

//--------------------------------------------------------------------------------------------------

void RobotApplication::on_init_button_clicked()
{
  std::cout << "Init button clicked" << std::endl;
  
  // Set the menu name
  Gtk::Label* window_name;
  builder_->get_widget<Gtk::Label>("window_name",window_name);
  window_name->set_text("Initialization of the components:");

  // Check whether the frame already holds children and remove then if so
  Gtk::Alignment* window_frame;
  builder_->get_widget<Gtk::Alignment>("window_alignment",window_frame);
  Gtk::Widget* child_ptr = window_frame->get_child();
  if(child_ptr) window_frame->remove();
  window_frame->add(init_tree_view_);
  window_frame->show_all_children();
}

//--------------------------------------------------------------------------------------------------

void RobotApplication::on_match_button_clicked()
{
  std::cout << "Match button clicked" << std::endl;  

  // Set the menu name
  Gtk::Label* window_name;
  builder_->get_widget<Gtk::Label>("window_name",window_name);
  window_name->set_text("Choose your team:");
  
  // Get the menu widget
  Gtk::Grid* match_grid;
  builder_->get_widget<Gtk::Grid>("match_grid",match_grid);
  
  // Check whether the frame already holds children and remove then if so
  Gtk::Alignment* window_frame;
  builder_->get_widget<Gtk::Alignment>("window_alignment",window_frame);
  Gtk::Widget* child_ptr = window_frame->get_child();
  if(child_ptr) window_frame->remove();
  window_frame->add(*match_grid);
}

//--------------------------------------------------------------------------------------------------

void RobotApplication::on_score_button_clicked()
{
  std::cout << "Score button clicked" << std::endl;
  
  // Get the window frame and set the score menu
  Gtk::Frame* window_frame;
  builder_->get_widget<Gtk::Frame>("window_frame",window_frame);
}

//--------------------------------------------------------------------------------------------------

void RobotApplication::on_sensor_button_clicked()
{
  std::cout << "Sensor button clicked" << std::endl;
  
  // Get the window frame and set the sensor menu
  Gtk::Frame* window_frame;
  builder_->get_widget<Gtk::Frame>("window_frame",window_frame);
}

//--------------------------------------------------------------------------------------------------

void RobotApplication::on_strategy_button_clicked()
{
  std::cout << "Strategy button clicked" << std::endl;
  
  // Get the window frame and set the strategy menu
  Gtk::Frame* window_frame;
  builder_->get_widget<Gtk::Frame>("window_frame",window_frame);
}

//--------------------------------------------------------------------------------------------------

} // namespace application
