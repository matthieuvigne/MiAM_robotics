#include <application/menu_widget.hpp>

namespace application {

//--------------------------------------------------------------------------------------------------
// Constructor and destructor
//--------------------------------------------------------------------------------------------------

MenuWidget::MenuWidget()
: Gtk::Frame(),
  above_box_ (Gtk::ORIENTATION_HORIZONTAL),
  below_box_ (Gtk::ORIENTATION_HORIZONTAL)
{
  // Set the grid
  //~ grid_.set_margin(10);
  grid_.set_row_homogeneous();
  grid_.attach(above_box_, 0, 0, 1, 1);
  grid_.attach(below_box_, 0, 1, 1, 1);
  add(grid_);
  
  // Set the above box
  above_box_.set_border_width(10);
  above_box_.set_spacing(10);
  above_box_.set_homogeneous(true);
  above_box_.add(initiazation_);
  above_box_.add(preparation_);
  above_box_.add(scoring_);
  
  // Set the below box
  below_box_.set_border_width(10);
  below_box_.set_spacing(10);
  below_box_.set_homogeneous(true);
  below_box_.add(sensors_);
  below_box_.add(viewer_);
  
  // Set the buttons
  
}

//--------------------------------------------------------------------------------------------------

MenuWidget::~MenuWidget(){}

//--------------------------------------------------------------------------------------------------
// Methods
//--------------------------------------------------------------------------------------------------

//~ void MenuWidget::initButton(Gtk::Button* button_ptr)
//~ {
  
//~ }

//--------------------------------------------------------------------------------------------------
  
} // namespace application
