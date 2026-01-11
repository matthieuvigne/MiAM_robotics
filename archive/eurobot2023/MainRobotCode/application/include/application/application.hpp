#ifndef APPLICATION_APPLICATION_HPP
#define APPLICATION_APPLICATION_HPP

#include <iostream>

#include <gtkmm.h>

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
    void on_init_button_clicked();
    void on_match_button_clicked();
    void on_score_button_clicked();
    void on_sensor_button_clicked();
    void on_strategy_button_clicked();
  
  protected:
  
    // Application window
    Glib::RefPtr<Gtk::Builder> builder_;
    Gtk::Window* main_window_;
    
    // Initialization panel
    Gtk::TreeView init_tree_view_;
    Glib::RefPtr<Gtk::ListStore> init_tree_model_;
    class InitModelColumns : public Gtk::TreeModel::ColumnRecord
    {
      public:
        InitModelColumns();
      public:
        Gtk::TreeModelColumn<Glib::ustring> col_type_;
        Gtk::TreeModelColumn<Glib::ustring> col_status_;
        Gtk::TreeModelColumn<Glib::ustring> col_comment_;
    };
    InitModelColumns init_columns_;
    
    
  
}; // class RobotApplication

//--------------------------------------------------------------------------------------------------

} // namespace application

#endif // APPLICATION_APPLICATION_HPP
